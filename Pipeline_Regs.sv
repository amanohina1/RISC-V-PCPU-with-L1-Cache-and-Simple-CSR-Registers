import internal_regs_package::*;

module Pipeline_Regs #(
    parameter nop = 32'h00000013
) (
    input  logic clk,
    input  logic rst,

    input stall_if_id,
    input stall_id_ex,
    input stall_ex_mem,
    input stall_mem_wb,

    input flush_if_id,
    input flush_id_ex,
    input flush_ex_mem,
    input flush_mem_wb,

    input  if_id_reg_t if_id_next,
    input  id_ex_reg_t id_ex_next,
    input  ex_mem_reg_t ex_mem_next,
    input  mem_wb_reg_t mem_wb_next,

    output if_id_reg_t if_id_reg_o,
    output id_ex_reg_t id_ex_reg_o,
    output ex_mem_reg_t ex_mem_reg_o,
    output mem_wb_reg_t mem_wb_reg_o
);
    if_id_reg_t if_id_reg;
    id_ex_reg_t id_ex_reg;
    ex_mem_reg_t ex_mem_reg;
    mem_wb_reg_t mem_wb_reg;

    always_ff @(posedge clk) begin  
        if (rst) begin
            if_id_reg  <= '{32'h00000013,32'b0,1'b1};
            id_ex_reg  <= '{valid : 1'b1,default: '0};
            ex_mem_reg <= '{valid : 1'b1,default: '0};
            mem_wb_reg <= '{valid : 1'b1,default: '0};
        end else begin

            if (flush_if_id) begin
                if_id_reg <= '{instruction: nop, default: '0};
            end else if (stall_if_id) if_id_reg  <= if_id_reg;
            else if_id_reg  <= if_id_next;

            if (flush_id_ex) begin
                id_ex_reg <= '{valid : 1'b1,default: '0};
            end else if (stall_id_ex) if_id_reg  <= if_id_reg;
            else id_ex_reg  <= id_ex_next;
            
            if (flush_ex_mem) 
                ex_mem_reg <= '{valid : 1'b1,default: '0};
            else if (stall_ex_mem) if_id_reg  <= if_id_reg;
            else ex_mem_reg  <= ex_mem_next;
            
            if (flush_mem_wb) 
                mem_wb_reg <= '{valid : 1'b1,default: '0};
            else if (stall_mem_wb) if_id_reg  <= if_id_reg;
            else mem_wb_reg  <= mem_wb_next;

        end
    end

    assign if_id_reg_o  = if_id_reg;
    assign id_ex_reg_o  = id_ex_reg;
    assign ex_mem_reg_o = ex_mem_reg;
    assign mem_wb_reg_o = mem_wb_reg;

endmodule