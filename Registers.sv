module Reg_Files (
    input [4:0] addres_A,
    input [4:0] addres_B,
    input [4:0] addres_D,
    input W_en,
    input [31:0] data_in,
    output [31:0] data_out_A,
    output [31:0] data_out_B,
    input clk,
    input rst,
    output reg [31:0] debug_reg_array [0:31] //for debug
);
    reg [31:0] registers [0:31];
    always @(negedge clk , posedge rst) begin
        if (rst) begin
            integer i;
            for (i = 0;i < 32;i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (W_en == 1) begin
            if (addres_D != 5'b0) begin
                registers[addres_D] <= data_in;
            end
        end
    end

    assign data_out_A = (addres_A == 5'b0) ? 32'b0 : registers[addres_A];
    assign data_out_B = (addres_B == 5'b0) ? 32'b0 : registers[addres_B];
    assign debug_reg_array = registers;
endmodule