`timescale 1ns / 1ps
module sw_forward(
        input [31:0] mem_wb_rs2,
        input [31:0] ex_mem_rs2,
        input [4:0] mem_wb_addr,
        input [4:0] ex_mem_addr,
        input [4:0] rs2_addr,
        input mem_wb_w,
        input ex_mem_w,
        input is_s,
        output reg [31:0] aft_forward,
        input [31:0] rs2_in
    );
    always_comb begin
        aft_forward = rs2_in;
        if (is_s) begin
            if (ex_mem_addr == rs2_addr && ex_mem_addr != 'b0 && ex_mem_w) begin
                aft_forward = ex_mem_rs2;
                end else if (mem_wb_addr == rs2_addr && ex_mem_addr != 'b0 && mem_wb_w) begin
                aft_forward = mem_wb_rs2;
                end
        end 
        end
endmodule
