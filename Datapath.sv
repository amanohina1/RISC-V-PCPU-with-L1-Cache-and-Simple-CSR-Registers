import internal_regs_package::*;
import cache_pkg::*;
`timescale 1ns / 1ps
// Core datapath: fetch, decode, execute, memory, writeback and CSR handling
// Naming conventions used in this file (non-enforced):
// - Signals from decode stage prefixed with `dec_` (control outputs).
// - Pipeline registers named `<stage>_<reg>` (e.g., `id_ex_reg`).
// - Control and forwarding outputs end with `_o` (e.g., `ALU_A_o`).
// - CSR related signals include `csr_` prefix.
module Core #(
    parameter DATA_WIDTH = 32,
    parameter CAPACITY = 1024
    ) (
    input rst_pc,
    input rst_rf,
    input rst_ctrl,
    input clk,

    Mem_ift.Master imem_bus, 
    Mem_ift.Master dmem_bus,

    output wire [DATA_WIDTH-1:0] debug_pc_out,
    output wire [DATA_WIDTH-1:0] debug_instruction,
    output wire debug_mem_write_enable,
    output wire debug_reg_write_enable,
    output wire [$clog2(DATA_WIDTH) - 1:0] debug_rd_addr_out,
    output wire [DATA_WIDTH-1:0] debug_alu_result,
    output wire [DATA_WIDTH-1:0] debug_data_to_write,
    output reg [31:0] debug_all_regs [0:31],
    output wire [3:0] wmask,
    output wire [DATA_WIDTH-1:0] immediate
);
    wire [31:0] internal_reg_array [0:31];
    
    if_id_reg_t  if_id_reg,  if_id_next;
    id_ex_reg_t  id_ex_reg,  id_ex_next;
    ex_mem_reg_t ex_mem_reg, ex_mem_next;
    mem_wb_reg_t mem_wb_reg, mem_wb_next;

    cpu_req_bus_t  pipeline_to_cache_req;
    logic          cache_ready_for_pipeline;
    cpu_resp_bus_t cache_to_pipeline_resp;

    mem_w_req_bus_t  cache_to_mem_w_req;
    logic            cache_to_mem_w_valid;
    mem_w_resp_bus_t mem_to_cache_w_resp;
    logic            mem_to_cache_w_resp_valid;

    mem_r_req_bus_t  cache_to_mem_r_req;
    logic            cache_to_mem_r_valid;
    mem_r_resp_bus_t mem_to_cache_r_resp;
    logic            mem_to_cache_r_resp_valid;

    // ----- Decode outputs / control signals -----
    wire dec_RegWrite, dec_MWrite, dec_MRead, dec_B_enable, dec_J_taken;
    wire dec_exception_valid;
    wire [1:0] dec_ALU_A, dec_ALU_B, dec_WBSelect, dec_is_lstype;
    wire [3:0] dec_ALUOp;
    wire [2:0] dec_B_type, dec_ImmeSelect, dec_l_type, dec_csr_op;
    wire [1:0] dec_S_type;
    wire dec_is_csr, dec_is_mret;
    wire dec_need_rs1, dec_need_rs2;
    wire [1:0] dec_current_priv_level;

    // ----- Hazard / pipeline control -----
    wire B_taken, PC_taken, ex_alu_is_zero;
    wire stall_pc, flush_id_ex, flush_if_id, flush_mem_wb, flush_ex_mem;
    wire stall_if_id, stall_ex_mem, stall_id_ex, stall_mem_wb;
    wire mem_stall, is_valid;
    wire [1:0] ALU_A_o, ALU_B_o;
    wire [1:0] BC_A_o, BC_B_o;
    wire CSR_Sel;
    wire Cache_valid, Cache_hit;
    wire is_luh;
    wire mem_aligned;
    // ----- Register / ALU / forwarding data paths -----
    wire [DATA_WIDTH - 1:0] pc_in, pc_out, PC_4;
    wire [DATA_WIDTH - 1:0] instruction_form_imem;
    wire [DATA_WIDTH - 1:0] id_rs1_data, id_rs2_data;
    wire [DATA_WIDTH - 1:0] wb_write_data;
    wire [DATA_WIDTH - 1:0] id_imm_ext;
    wire [DATA_WIDTH - 1:0] ex_alu_src_a, ex_alu_src_b;
    wire [DATA_WIDTH - 1:0] forward_data_from_mem_wb;
    wire [DATA_WIDTH - 1:0] ex_alu_result, mem_pc_4, ex_alu_result_final;
    wire [DATA_WIDTH - 1:0] mem_read_data;
    wire [DATA_WIDTH - 1:0] mem_data_to_write_final;
    wire [DATA_WIDTH / 8 - 1:0] mem_write_mask;
    wire [DATA_WIDTH - 1:0] mem_data_after_transfer, branch_src_b, branch_src_a;
    wire [DATA_WIDTH - 1:0] aft_forward;

    // ----- CSR related -----
    wire [DATA_WIDTH - 1:0] csr_rdata_out;
    wire [DATA_WIDTH - 1:0] csr_rdata_from_exe;  // Data read from CSR file in EXE
    wire [DATA_WIDTH - 1:0] csr_handler_addr;    // Exception handler address from CSR file
    wire [DATA_WIDTH - 1:0] csr_mepc_val;        // MEPC value for MRET jump (read in EXE)
    wire [DATA_WIDTH - 1:0] next_pc_stage1; // Output of the first (lowest priority) MUX
    wire [DATA_WIDTH - 1:0] next_pc_stage2;
    wire [DATA_WIDTH - 1:0] mstatus_now;
    wire [DATA_WIDTH - 1:0] dec_exception_cause;
    wire [DATA_WIDTH - 1:0] mret_out;
    wire [DATA_WIDTH - 1:0] CSR_to_alusrc_c;
    wire [DATA_WIDTH - 1:0] CSR_to_alusrc_d;

    reg  [DATA_WIDTH - 1:0] csr_wdata_to_exe;    // Data to be written to CSR, calculated in EXE
    reg  csr_wen_from_exe;
    // ----- Local logic / temporaries -----
    logic [DATA_WIDTH-1:0] source_operand;
    logic is_ecall_inst;
    logic [1:0] exception_sel;
    // ----- Fetch stage: instruction memory read -----
    assign imem_bus.r_request_valid = 1'b1;
    assign imem_bus.r_request_bits.raddr = pc_out;
    assign instruction_form_imem = imem_bus.r_reply_bits.rdata[31:0];
    
    assign pipeline_to_cache_req.valid  = (ex_mem_reg.Mem_Read || ex_mem_reg.Mem_Write ) && ex_mem_reg.valid;
    assign pipeline_to_cache_req.offset = ex_mem_reg.alu_result[3:0];
    assign pipeline_to_cache_req.index  = ex_mem_reg.alu_result[5:4];
    assign pipeline_to_cache_req.tag    = ex_mem_reg.alu_result[31:6];
    assign pipeline_to_cache_req.wdata  = mem_data_to_write_final; // from DataPkg
    assign pipeline_to_cache_req.wmask  = mem_write_mask;
    assign pipeline_to_cache_req.write  = ex_mem_reg.Mem_Write == 1'b1;
    assign pipeline_to_cache_req.read   = ex_mem_reg.Mem_Read == 1'b1;
    assign pipeline_to_cache_req.S_type = ex_mem_reg.S_type;
    assign pipeline_to_cache_req.l_type = ex_mem_reg.l_type;

    assign mem_read_data = cache_to_pipeline_resp.rdata;
    assign Cache_valid   = cache_to_pipeline_resp.valid;
    assign Cache_hit     = cache_to_pipeline_resp.hit;
    assign mem_aligned   = ~cache_to_pipeline_resp.exception;
    assign mem_stall = !cache_ready_for_pipeline;

    assign dmem_bus.w_request_valid = cache_to_mem_w_valid;
    assign dmem_bus.w_request_bits.waddr = cache_to_mem_w_req.addr; // waddr <- addr
    assign dmem_bus.w_request_bits.wdata = cache_to_mem_w_req.data; // wdata <- data
    assign dmem_bus.w_request_bits.wmask = 'b1;

    assign dmem_bus.r_request_valid = cache_to_mem_r_valid;
    assign dmem_bus.r_request_bits.raddr = cache_to_mem_r_req.addr; 

    assign mem_to_cache_w_resp_valid = dmem_bus.w_reply_valid;
    assign mem_to_cache_w_resp.bresp = dmem_bus.w_reply_bits.bresp;

    assign mem_to_cache_r_resp_valid = dmem_bus.r_reply_valid;
    assign mem_to_cache_r_resp.rdata = dmem_bus.r_reply_bits.rdata;
    assign mem_to_cache_r_resp.rresp = dmem_bus.r_reply_bits.rresp;

    // CSR MEPC value available during EXE for handling MRET
    assign csr_mepc_val = mret_out;
    assign forward_data_from_mem_wb = wb_write_data;
    assign debug_pc_out = pc_out;

    // ----- EX/MEM/ID register update logic (next-state calculation) -----
    always_comb begin
        if (id_ex_reg.is_csr_inst && id_ex_reg.csr_op[2]) begin
            source_operand = { {DATA_WIDTH-5{1'b0}}, id_ex_reg.rs1_addr }; //rs1_addr holds the uimm
        end else 
            source_operand = ex_alu_src_a; 

        if (id_ex_reg.is_csr_inst) begin
            case (id_ex_reg.csr_op[1:0])
                2'b01: begin // CSRRW or CSRRWI (Write)
                    csr_wen_from_exe = 1'b1;
                    csr_wdata_to_exe = source_operand;
                end
                2'b10: begin // CSRRS or CSRRSI (Set)
                    csr_wen_from_exe = (source_operand != 0); // Write only if there are bits to set
                    csr_wdata_to_exe = csr_rdata_from_exe | source_operand;
                end
                2'b11: begin // CSRRC or CSRRCI (Clear)
                    csr_wen_from_exe = (source_operand != 0); // Write only if there are bits to clear
                    csr_wdata_to_exe = csr_rdata_from_exe & ~source_operand;
                end
                default:begin
                    csr_wen_from_exe = 1'b0;
                    csr_wdata_to_exe = csr_rdata_from_exe;
                end
            endcase
        end else begin
            csr_wen_from_exe = 1'b0;
            csr_wdata_to_exe = csr_rdata_from_exe;
        end
    end

    // Next-state logic for all pipeline registers
    always_comb begin
        if_id_next = '{32'h00000013,32'b0,1'b1};
        id_ex_next  = '{valid : 1'b1,default: '0}; 
        ex_mem_next = '{valid : 1'b1,default: '0};
        mem_wb_next = '{valid : 1'b1,default: '0};

        if_id_next.instruction = instruction_form_imem;
        if_id_next.PC = pc_out ;
        if_id_next.valid = 1'b1; 
        
        id_ex_next.Reg_Write = dec_RegWrite;  
        id_ex_next.Mem_Read  = dec_MRead;     
        id_ex_next.Mem_Write = dec_MWrite;    
        id_ex_next.ALU_A     = dec_ALU_A;     
        id_ex_next.ALU_B     = dec_ALU_B;  
        id_ex_next.ALU_op    = dec_ALUOp;       
        id_ex_next.WBSel     = dec_WBSelect;  
        id_ex_next.B_type    = dec_B_type;    
        id_ex_next.B_enable  = dec_B_enable;  
        id_ex_next.J_taken   = dec_J_taken;   
        id_ex_next.need_rs1  = dec_need_rs1;
        id_ex_next.need_rs2  = dec_need_rs2;
        id_ex_next.l_type    = dec_l_type;    
        id_ex_next.S_type    = dec_S_type;    
        id_ex_next.PC        = if_id_reg.PC;
        id_ex_next.rs1out    = id_rs1_data;
        id_ex_next.rs2out    = id_rs2_data;
        id_ex_next.imme      = id_imm_ext;
        id_ex_next.is_lstype = dec_is_lstype;
        id_ex_next.rd_addr   = if_id_reg.instruction[11:7];
        id_ex_next.rs1_addr  = if_id_reg.instruction[19:15];
        id_ex_next.rs2_addr  = if_id_reg.instruction[24:20];
        id_ex_next.valid     = is_valid;
        id_ex_next.inst_pc   = if_id_reg.PC;
        id_ex_next.exception_valid   = dec_exception_valid;   
        id_ex_next.exception_cause   = dec_exception_cause;   
        id_ex_next.inst_priv_level = dec_current_priv_level; 
        id_ex_next.is_csr_inst        = dec_is_csr;        
        id_ex_next.is_mret_inst       = dec_is_mret;       
        id_ex_next.csr_addr      = if_id_reg.instruction[31:20]; 
        id_ex_next.csr_op        = dec_csr_op;        
        id_ex_next.is_ecall_inst = is_ecall_inst;
        id_ex_next.csr_rdata = csr_rdata_from_exe;

        ex_mem_next.Mem_Read   = id_ex_reg.Mem_Read;
        ex_mem_next.Mem_Write  = id_ex_reg.Mem_Write;
        ex_mem_next.Reg_Write  = id_ex_reg.Reg_Write;
        ex_mem_next.rd_addr   = id_ex_reg.rd_addr;
        ex_mem_next.PC        = id_ex_reg.PC;
        ex_mem_next.alu_result     = ex_alu_result_final;
        ex_mem_next.valid = id_ex_reg.valid;
        ex_mem_next.data_to_mem = aft_forward;
        ex_mem_next.S_type = id_ex_reg.S_type;
        ex_mem_next.l_type = id_ex_reg.l_type;
        ex_mem_next.WBSel = id_ex_reg.WBSel;
        ex_mem_next.exception_valid   = id_ex_reg.exception_valid;   
        ex_mem_next.exception_cause   = id_ex_reg.exception_cause;   
        ex_mem_next.inst_pc           = id_ex_reg.inst_pc;           
        ex_mem_next.inst_priv_level   = id_ex_reg.inst_priv_level;   
        ex_mem_next.is_mret_inst      = id_ex_reg.is_mret_inst;
        ex_mem_next.is_ecall_inst     = id_ex_reg.is_ecall_inst;
        ex_mem_next.csr_wen = csr_wen_from_exe;
        ex_mem_next.csr_addr = id_ex_reg.csr_addr;
        ex_mem_next.csr_wdata = csr_wdata_to_exe;
        ex_mem_next.csr_rdata = id_ex_reg.csr_rdata;

        mem_wb_next.rd_addr = ex_mem_reg.rd_addr;
        mem_wb_next.alu_result = ex_mem_reg.alu_result;
        mem_wb_next.WBSel  = ex_mem_reg.WBSel;
        mem_wb_next.Reg_Write  = ex_mem_reg.Reg_Write;
        mem_wb_next.PC_4 = mem_pc_4;
        mem_wb_next.data_from_mem = mem_data_after_transfer;
        mem_wb_next.valid = ex_mem_reg.valid;
        mem_wb_next.Mem_Write = ex_mem_reg.Mem_Write;
        mem_wb_next.data_saved = mem_data_to_write_final;
        mem_wb_next.exception_valid   = ex_mem_reg.exception_valid;   
        mem_wb_next.exception_cause   = ex_mem_reg.exception_cause;   
        mem_wb_next.inst_pc           = ex_mem_reg.inst_pc;           
        mem_wb_next.inst_priv_level   = ex_mem_reg.inst_priv_level;   
        mem_wb_next.is_mret_inst      = ex_mem_reg.is_mret_inst; 
        mem_wb_next.is_ecall_inst     = ex_mem_reg.is_ecall_inst; 
        mem_wb_next.csr_wen = ex_mem_reg.csr_wen;
        mem_wb_next.csr_addr = ex_mem_reg.csr_addr;
        mem_wb_next.csr_wdata = ex_mem_reg.csr_wdata;
        mem_wb_next.csr_rdata = ex_mem_reg.csr_rdata;
        if (!mem_aligned && ex_mem_reg.Mem_Read) begin
            mem_wb_next.exception_valid = 1'b1;
            mem_wb_next.is_ecall_inst = 1'b1;
            mem_wb_next.exception_cause = 32'h00000004; // Example cause
        end
        if (!mem_aligned && ex_mem_reg.Mem_Write) begin
            mem_wb_next.exception_valid = 1'b1;
            mem_wb_next.is_ecall_inst = 1'b1;
            mem_wb_next.exception_cause = 32'h00000006; // Example cause
        end
    end

    // PC selection and branch/jump
    MUX2to1 pc_mux_branch (
        .out(next_pc_stage1),
        .A(PC_4),
        .B(ex_alu_result_final), // Target for Branches and JALR
        .sel(PC_taken)
    );

    // MRET override PC 
    MUX2to1 pc_mux_csr (
        .out(next_pc_stage2),
        .A(next_pc_stage1),    
        .B(csr_mepc_val),      
        .sel(mem_wb_reg.is_mret_inst)  //mem_wb
    );

    // Determine exception selection
    always_comb begin
        if (mem_wb_reg.is_ecall_inst) exception_sel = 2'b01;
        else if (mem_wb_reg.valid & mem_wb_reg.is_mret_inst) exception_sel = 2'b10;
        else exception_sel = 2'b00;
    end

    // Top-level PC MUX including exception handler and MRET targets
    MUX4to1 pc_mux_exception (
        .out(pc_in),           
        .A(next_pc_stage2),    
        .B(csr_handler_addr),  
        .C(mret_out),
        .D('b0),
        .sel(exception_sel) 
    );

    // Fetch: Program Counter
    PC ProgrammeCounter(.clk(clk),
                        .reset(rst_pc),
                        .next_pc(pc_in),
                        .halt(stall_pc),
                        .addr(pc_out));

    Add adder1(.a(pc_out),
               .b(32'h00000004),
               .y(PC_4));

    // Pipeline registers
    Pipeline_Regs pipe_regs (
        .clk(clk),
        .rst(rst_ctrl), 

        .stall_if_id(stall_if_id),
        .stall_id_ex(stall_id_ex),
        .stall_ex_mem(stall_ex_mem),
        .stall_mem_wb(stall_mem_wb),

        .flush_if_id(flush_if_id),
        .flush_id_ex(flush_id_ex),
        .flush_ex_mem(flush_ex_mem),
        .flush_mem_wb(flush_mem_wb),

        .if_id_next(if_id_next),
        .id_ex_next(id_ex_next),
        .ex_mem_next(ex_mem_next),
        .mem_wb_next(mem_wb_next),

        .if_id_reg_o(if_id_reg),
        .id_ex_reg_o(id_ex_reg),
        .ex_mem_reg_o(ex_mem_reg),
        .mem_wb_reg_o(mem_wb_reg)
    );

    // Register file
    Reg_Files regs(.clk(clk),
                   .addres_A(if_id_reg.instruction[19:15]),
                   .addres_B(if_id_reg.instruction[24:20]),
                   .addres_D(mem_wb_reg.rd_addr),
                   .W_en(mem_wb_reg.Reg_Write && mem_wb_reg.valid),
                   .data_in(wb_write_data),
                   .data_out_A(id_rs1_data),
                   .data_out_B(id_rs2_data),
                   .rst(rst_rf),
                   .debug_reg_array(internal_reg_array));

    // Decode / Control unit
    Control control(.funct7(if_id_reg.instruction[31:25]),
                    .funct3(if_id_reg.instruction[14:12]),
                    .opcode(if_id_reg.instruction[6:0]),

                    .RegWrite(dec_RegWrite),
                    .MW(dec_MWrite),
                    .A_Select(dec_ALU_A),
                    .B_Select(dec_ALU_B),
                    .jump(dec_J_taken),
                    .Op(dec_ALUOp),
                    .WBSelect(dec_WBSelect),
                    .ImmeSelect(dec_ImmeSelect),
                    .B_type(dec_B_type),
                    .S_type(dec_S_type),
                    .B_enable(dec_B_enable),
                    .l_type(dec_l_type),
                    .MRead(dec_MRead),
                    .need_rs1(dec_need_rs1),
                    .need_rs2(dec_need_rs2),
                    .is_lstype(dec_is_lstype),
                    .mret_out(mem_wb_reg.valid && mem_wb_reg.is_mret_inst),
                    .exception(mem_wb_reg.valid && mem_wb_reg.exception_valid),
                    .mstatus(mstatus_now),
                    .exception_valid(dec_exception_valid),
                    .exception_cause(dec_exception_cause),
                    .is_csr(dec_is_csr),
                    .is_mret(dec_is_mret),
                    .current_privilege(dec_current_priv_level),
                    .csr_op(dec_csr_op),
                    .clk(clk),
                    .rst(rst_ctrl),
                    .is_ecall_inst(is_ecall_inst),
                    .csr_addr(if_id_reg.instruction[31:20]),
                    .valid(is_valid));

    // Immediate generation
    Imme_Gen gene(.instruction(if_id_reg.instruction),
                  .mode(dec_ImmeSelect),
                  .out_imme(id_imm_ext));

    // Hazard detection & forwarding controls
    Hazard hazard(.id_ex_is_branch(id_ex_reg.B_enable),
                  .id_ex_is_jump(id_ex_reg.J_taken),
                  .id_stage_rs1_addr(if_id_reg.instruction[19:15]),
                  .id_stage_rs2_addr(if_id_reg.instruction[24:20]),
                  .id_ex_rd_addr(id_ex_reg.rd_addr),
                  .id_ex_rs1_addr(id_ex_reg.rs1_addr),
                  .id_ex_rs2_addr(id_ex_reg.rs2_addr),
                  .id_ex_lstype(id_ex_reg.is_lstype),
                  .mem_wb_reg_addr(mem_wb_reg.rd_addr),
                  .mem_wb_reg_write(mem_wb_reg.Reg_Write),
                  .ex_mem_reg_addr(ex_mem_reg.rd_addr),
                  .ex_mem_reg_write(ex_mem_reg.Reg_Write),
                  .need_rs1(id_ex_reg.need_rs1),
                  .need_rs2(id_ex_reg.need_rs2),
                  .ALU_A(id_ex_reg.ALU_A),
                  .ALU_B(id_ex_reg.ALU_B),
                  .ALU_A_o(ALU_A_o),
                  .ALU_B_o(ALU_B_o),
                  .stall_pc(stall_pc),
                  .stall_if_id(stall_if_id),
                  .flush_id_ex(flush_id_ex),
                  .B_taken(B_taken),
                  .PC_Sel(PC_taken),
                  .flush_if_id(flush_if_id),
                  .clk(clk),
                  .BC_A_o(BC_A_o),
                  .BC_B_o(BC_B_o),
                  .stall_ex_mem(stall_ex_mem),
                  .stall_id_ex(stall_id_ex),
                  .stall_mem_wb(stall_mem_wb),
                  .mem_stall(mem_stall),
                  .flush_ex_mem(flush_ex_mem),
                  .flush_mem_wb(flush_mem_wb),
                  .is_exception_commit(mem_wb_reg.exception_valid),
                  .is_mret_in_exe(id_ex_reg.is_mret_inst && id_ex_reg.valid),
                  .is_mret_in_wb(mem_wb_reg.is_mret_inst),
                  .CSR_Sel(CSR_Sel),
                  .CSR_addr_exe(id_ex_reg.csr_addr),
                  .CSR_addr_wb(mem_wb_reg.csr_addr),
                  .CSR_addr_mem(ex_mem_reg.csr_addr),
                  .id_ex_is_csr_inst(id_ex_reg.is_csr_inst),
                  .id_ex_reg_write(id_ex_reg.Reg_Write),
                  .ex_mem_reg_csr_wen(ex_mem_reg.csr_wen),
                  .mem_wb_reg_csr_wen(mem_wb_reg.csr_wen),
                  .is_luh(is_luh));

    // Execute stage: operand selection and ALU
    MUX4to1 mux2(.out(ex_alu_src_a),
                 .A(id_ex_reg.rs1out),
                 .B(id_ex_reg.PC),
                 .C(CSR_to_alusrc_c),
                 .D(CSR_to_alusrc_d),
                 .sel(ALU_A_o));
    
    MUX4to1 mux3(.out(ex_alu_src_b),
                 .A(id_ex_reg.rs2out),
                 .B(id_ex_reg.imme),
                 .C(CSR_to_alusrc_c),
                 .D(CSR_to_alusrc_d),
                 .sel(ALU_B_o));

    MUX2to1 CSR_mem(.out(CSR_to_alusrc_c),
                    .A(ex_mem_reg.alu_result),
                    .B(ex_mem_reg.csr_rdata),
                    .sel(ex_mem_reg.is_csr_inst));

    MUX2to1 CSR_wb(.out(CSR_to_alusrc_d),
                    .A(forward_data_from_mem_wb),
                    .B(mem_wb_reg.csr_rdata),
                    .sel(mem_wb_reg.is_csr_inst));

    ALU alu(.A(ex_alu_src_a),
            .B(ex_alu_src_b),
            .control(id_ex_reg.ALU_op),
            .result(ex_alu_result),
            .is_zero(ex_alu_is_zero));

    MUX2to1 mux7(.out(ex_alu_result_final),
                 .A(ex_alu_result),
                 .B(csr_rdata_from_exe),
                 .sel(id_ex_reg.is_csr_inst));

    // Branch comparison
    Branch_Comparision BC(.rs1(branch_src_a),
                          .rs2(branch_src_b),
                          .B_type(id_ex_reg.B_type),
                          .branch_taken(B_taken));

    Add adder2(.a(ex_mem_reg.PC),
               .b(32'h00000004),
               .y(mem_pc_4));

    // Memory stage
    Transfer transfer(.data_in(mem_read_data),
                      .data_out(mem_data_after_transfer),
                      .l_type(ex_mem_reg.l_type),
                      .data_address(ex_mem_reg.alu_result));
    
    DataPkg datapkg(.data_address(ex_mem_reg.alu_result),
                    .data_reg(ex_mem_reg.data_to_mem),
                    .data_mem(mem_data_to_write_final),
                    .op(ex_mem_reg.S_type),
                    .mask(mem_write_mask));

    DataCache dcache (
        .clk(clk),
        .rst(rst_ctrl),

        // CPU to Cache 
        .cpu_req(pipeline_to_cache_req),
        .cpu_req_ready(cache_ready_for_pipeline),
        .cpu_resp(cache_to_pipeline_resp),

        // Cache to Memory 
        .mem_w_req(cache_to_mem_w_req),
        .mem_w_req_valid(cache_to_mem_w_valid),
        .mem_w_req_ready(dmem_bus.w_request_ready), 
        
        .mem_w_resp(mem_to_cache_w_resp),
        .mem_w_resp_valid(mem_to_cache_w_resp_valid),

        // Cache to Memory Read
        .mem_r_req(cache_to_mem_r_req),
        .mem_r_req_valid(cache_to_mem_r_valid),
        .mem_r_req_ready(dmem_bus.r_request_ready), 

        .mem_r_resp(mem_to_cache_r_resp),
        .mem_r_resp_valid(mem_to_cache_r_resp_valid)
    );

    // Writeback stage
    MUX4to1 mux4(.A(mem_wb_reg.data_from_mem),
                 .B(mem_wb_reg.alu_result),
                 .C(mem_wb_reg.PC_4),
                 .D(32'b0),
                 .sel(mem_wb_reg.WBSel),
                 .out(wb_write_data));
    
    sw_forward sw_forward(
                 .mem_wb_rs2(wb_write_data),
                 .mem_wb_w(mem_wb_reg.Reg_Write),
                 .ex_mem_w(ex_mem_reg.Reg_Write),
                 .ex_mem_rs2(ex_mem_reg.alu_result),
                 .mem_wb_addr(mem_wb_reg.rd_addr),
                 .ex_mem_addr(ex_mem_reg.rd_addr),
                 .rs2_addr(id_ex_reg.rs2_addr),
                 .is_s(id_ex_reg.is_lstype[1]),
                 .aft_forward(aft_forward),
                 .rs2_in(id_ex_reg.rs2out));

    MUX4to1 mux5(.A(id_ex_reg.rs1out),
                 .B('b0),
                 .C(CSR_to_alusrc_c),
                 .D(CSR_to_alusrc_d),
                 .sel(BC_A_o),
                 .out(branch_src_a));

    MUX4to1 mux6(.A(id_ex_reg.rs2out),
                 .B('b0),
                 .C(CSR_to_alusrc_c),
                 .D(CSR_to_alusrc_d),
                 .sel(BC_B_o),
                 .out(branch_src_b));

    // CSR unit
    CSR_Regs CSR(.rst(rst_ctrl),
                 .clk(clk),
                 .priv_level(mem_wb_reg.inst_priv_level),
                 .exception_tval(mem_wb_reg.alu_result),
                 .exception_cause(mem_wb_reg.exception_cause),
                 .exception_pc(mem_wb_reg.inst_pc),
                 .exception_commit(mem_wb_reg.exception_valid),
                 .mret_commit(mem_wb_reg.is_mret_inst),
                 .inst_commit(mem_wb_reg.valid),
                 .handler_addr(csr_handler_addr),
                 .csr_raddr(id_ex_reg.csr_addr),    
                 .csr_rdata(csr_rdata_out),         
                 .csr_wen(mem_wb_reg.csr_wen),             
                 .csr_waddr(mem_wb_reg.csr_addr),   
                 .csr_wdata(mem_wb_reg.csr_wdata),
                 .mstatus_out(mstatus_now),
                 .mret_out(mret_out),
                 .mem_stall(mem_stall | is_luh));

    // CSR forward to EX 
    MUX2to1 CSR_Forwarding(.A(csr_rdata_out),
                           .B(ex_mem_reg.csr_wdata),
                           .sel(CSR_Sel),
                           .out(csr_rdata_from_exe));
    //debug outputs
    assign debug_instruction = instruction_form_imem;
    assign debug_mem_write_enable = dec_MWrite;
    assign debug_alu_result = ex_alu_result_final;
    assign debug_data_to_write = mem_data_to_write_final;
    assign debug_reg_write_enable = dec_RegWrite;
    assign debug_rd_addr_out = id_ex_reg.rd_addr;
    assign wmask = mem_write_mask;
    assign immediate = id_imm_ext;
    
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : core_debug_assign
            assign debug_all_regs[i] = internal_reg_array[i];
        end
    endgenerate
endmodule
