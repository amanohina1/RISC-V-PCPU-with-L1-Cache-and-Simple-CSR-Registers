`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/09/01 23:59:52
// Design Name: 
// Module Name: tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`timescale 1ns / 1ps
module tb_top;
    localparam DATA_WIDTH     = 32;
    localparam MEM_CAPACITY   = 4096; 
    localparam CLK_PERIOD   = 10;  
    localparam HEX_FILE_PATH = ""; 

    reg  clk;
    reg  rst;
    wire [DATA_WIDTH-1:0] debug_pc;
    wire [DATA_WIDTH-1:0] debug_inst;
    wire                      debug_reg_write;
    wire                      debug_mem_write;
    wire [4:0]                debug_rd_addr; 
    wire [DATA_WIDTH-1:0] debug_write_data;
    wire [DATA_WIDTH-1:0] debug_alu_result;
    wire [31:0] all_cpu_registers [0:31];
    wire [3:0] wmask;
    wire [DATA_WIDTH-1:0] immediate;
    wire [DATA_WIDTH-1:0] memory_state [0:MEM_CAPACITY / (DATA_WIDTH / 8) - 1];
    top #(
        .DATA_WIDTH (DATA_WIDTH),
        .CAPACITY   (MEM_CAPACITY),
        .FILE_PATH  (HEX_FILE_PATH)
    ) dut (
        .clk(clk),
        .rst(rst),

        .debug_pc(debug_pc),
        .debug_inst(debug_inst)
        //.debug_reg_write(debug_reg_write),
        //.debug_mem_write(debug_mem_write),
        //.debug_rd_addr(debug_rd_addr),
        //.debug_write_data(debug_write_data),
        //.ALU_result(debug_alu_result),
        //.debug_all_regs(),
        //.mask(wmask),
        //.imme(immediate),
        //.debug_mem_content()
    );

     always #((CLK_PERIOD)/2) clk = ~clk;
initial begin
    clk = 1'b0;
    rst = 1'b1;
    # (CLK_PERIOD * 10); 
    rst = 1'b0;
    // Simulation code here
end
endmodule


