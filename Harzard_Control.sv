module Hazard #(
    parameter DATA_WIDTH = 32
) (
    input  logic clk,
    input  logic id_ex_is_branch,
    input  logic id_ex_is_jump,
    input  logic B_taken,

    input  logic [$clog2(DATA_WIDTH) - 1:0] id_stage_rs1_addr,
    input  logic [$clog2(DATA_WIDTH) - 1:0] id_stage_rs2_addr,

    input  logic [$clog2(DATA_WIDTH) - 1:0] id_ex_rd_addr,
    input  logic [$clog2(DATA_WIDTH) - 1:0] id_ex_rs1_addr,
    input  logic [$clog2(DATA_WIDTH) - 1:0] id_ex_rs2_addr,
    input  logic [1:0] id_ex_lstype,

    input  logic [$clog2(DATA_WIDTH) - 1:0] mem_wb_reg_addr,
    input  logic mem_wb_reg_write,

    input  logic [$clog2(DATA_WIDTH) - 1:0] ex_mem_reg_addr,
    input  logic ex_mem_reg_write,

    input  logic need_rs1,
    input  logic need_rs2,

    input  logic mem_stall,

    input  logic [1:0] ALU_A,
    input  logic [1:0] ALU_B,

    input  logic is_mret_in_exe,    // from id_ex_reg.is_mret
    input  logic is_exception_commit,
    input  logic is_mret_in_wb,

    input  logic [11:0] CSR_addr_exe,
    input  logic [11:0] CSR_addr_mem,
    input  logic [11:0] CSR_addr_wb,
    input  logic id_ex_is_csr_inst,
    input  logic id_ex_reg_write,
    input  logic ex_mem_reg_csr_wen,
    input  logic mem_wb_reg_csr_wen,

    output logic [1:0] ALU_A_o,
    output logic [1:0] ALU_B_o,
    output logic flush_ex_mem,
    output logic flush_mem_wb,
    output logic stall_pc,
    output logic stall_if_id,
    output logic stall_id_ex,
    output logic stall_ex_mem,
    output logic stall_mem_wb,
    output logic flush_id_ex,
    output logic flush_if_id,
    output logic PC_Sel,
    output logic [1:0] BC_A_o, 
    output logic [1:0] BC_B_o, 
    output logic CSR_Sel,
    output logic is_luh
);

    logic is_bh;               // branch/jump hazard
    logic is_mh;               // mret in exe
    logic csr_read_use_hazard; // CSR read used by following instruction

    always_comb begin
        ALU_A_o       = ALU_A;
        ALU_B_o       = ALU_B;
        stall_pc      = 1'b0;
        stall_if_id   = 1'b0;
        stall_id_ex   = 1'b0;
        stall_ex_mem  = 1'b0;
        stall_mem_wb  = 1'b0;
        
        flush_if_id   = 1'b0;
        flush_id_ex   = 1'b0;
        flush_ex_mem  = 1'b0;
        flush_mem_wb  = 1'b0;

        PC_Sel        = 1'b0;
        BC_A_o        = 2'b00;
        BC_B_o        = 2'b00;

        // hazard detection
        is_luh = id_ex_lstype[0]
                 && (id_stage_rs1_addr == id_ex_rd_addr || id_stage_rs2_addr == id_ex_rd_addr)
                 && (id_ex_rd_addr != 'b0);

        is_bh  = (B_taken && id_ex_is_branch) || id_ex_is_jump;
        is_mh  = is_mret_in_exe;

        csr_read_use_hazard = id_ex_is_csr_inst && id_ex_reg_write && (id_ex_rd_addr != 'b0)
                              && ((need_rs1 && (id_stage_rs1_addr == id_ex_rd_addr))
                                  || (need_rs2 && (id_stage_rs2_addr == id_ex_rd_addr)));

        // hazard handling logic
        if (is_exception_commit || is_mret_in_wb) begin
            flush_if_id  = 1'b1;
            flush_id_ex  = 1'b1;
            flush_ex_mem = 1'b1;
            flush_mem_wb = 1'b1;
        end else if (mem_stall) begin
            stall_pc    = 1'b1;
            stall_if_id = 1'b1;
            stall_id_ex = 1'b1;
            stall_ex_mem = 1'b1;
            stall_mem_wb = 1'b1;
        end else if (is_luh) begin
            stall_if_id = 1'b1;
            stall_pc    = 1'b1;
            flush_id_ex = 1'b1;
        end else if (csr_read_use_hazard) begin
            stall_pc    = 1'b1;
            stall_if_id = 1'b1;
            flush_id_ex = 1'b1;
        end else if (is_bh) begin
            PC_Sel      = 1'b1;
            flush_if_id = 1'b1;
            flush_id_ex = 1'b1;
        end else if (is_mh) begin
            flush_if_id = 1'b1;
            flush_id_ex = 1'b1;
        end

        // Forwarding for ALU A (rs1)
        if (need_rs1) begin
            if (id_ex_is_branch) begin
                ALU_A_o = 2'b01;
            end else if (ex_mem_reg_write && (ex_mem_reg_addr != 'b0) && (ex_mem_reg_addr == id_ex_rs1_addr)) begin
                ALU_A_o = 2'b10;
            end else if (mem_wb_reg_write && (mem_wb_reg_addr != 'b0) && (mem_wb_reg_addr == id_ex_rs1_addr)) begin
                ALU_A_o = 2'b11;
            end
        end

        // Forwarding for ALU B (rs2)
        if (need_rs2) begin
            if (id_ex_is_branch) begin
                ALU_B_o = 2'b01;
            end else if (ex_mem_reg_write && (ex_mem_reg_addr != 'b0) && (ex_mem_reg_addr == id_ex_rs2_addr)) begin
                if (id_ex_lstype != 2'b00)
                    ALU_B_o = 2'b01; // immediate selected
                else
                    ALU_B_o = 2'b10; // forward from EX/MEM ALU
            end else if (mem_wb_reg_write && (mem_wb_reg_addr != 'b0) && (mem_wb_reg_addr == id_ex_rs2_addr)) begin
                if (id_ex_lstype != 2'b00)
                    ALU_B_o = 2'b01; // immediate
                else
                    ALU_B_o = 2'b11; // forward from MEM/WB
            end
        end

        // Branch-comparison operand selects (BC_A_o, BC_B_o)
        if (need_rs1) begin
            if (ex_mem_reg_write && (ex_mem_reg_addr == id_ex_rs1_addr))
                BC_A_o = 2'b10;
            else if (mem_wb_reg_write && (mem_wb_reg_addr == id_ex_rs1_addr))
                BC_A_o = 2'b11;
        end

        if (need_rs2) begin
            if (ex_mem_reg_write && (ex_mem_reg_addr == id_ex_rs2_addr))
                BC_B_o = 2'b10;
            else if (mem_wb_reg_write && (mem_wb_reg_addr == id_ex_rs2_addr))
                BC_B_o = 2'b11;
        end

        // CSR forwarding select
        CSR_Sel = 1'b0;
        if (ex_mem_reg_csr_wen && (CSR_addr_exe == CSR_addr_mem)) begin
            CSR_Sel = 1'b1; // forward from MEM
        end else if (mem_wb_reg_csr_wen && (CSR_addr_exe == CSR_addr_wb)) begin
            CSR_Sel = 1'b0; // WB has final value (default)
        end
    end


endmodule
