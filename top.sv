`timescale 1ns / 1ps
`include "mem_ift.vh"

module top # (
    parameter DATA_WIDTH = 32,
    parameter CAPACITY = 1024,
    parameter FILE_PATH = "C:/Users/Amano/Desktop/PLCPU/project_1/test.hex"
) (
    input clk,
    input rst,

    output wire [DATA_WIDTH-1:0] debug_pc,
    output wire [DATA_WIDTH-1:0] debug_inst
    //output wire                      debug_reg_write,
    //output wire                      debug_mem_write,
    //output wire [4:0]                debug_rd_addr,
    //output wire [DATA_WIDTH-1:0] debug_write_data,
    //output wire [DATA_WIDTH-1:0] ALU_result,
    //output wire [3:0]                mask,
    //output wire [DATA_WIDTH-1:0] imme
    //output wire [DATA_WIDTH-1:0] debug_all_regs [0:31],
    //output wire [DATA_WIDTH-1:0] debug_mem_content [0:(CAPACITY / (DATA_WIDTH/8))-1]
);

    // Memory interfaces
    Mem_ift imem_bus();
    Mem_ift dmem_bus();

    // Internal arrays used for debug export
    wire [DATA_WIDTH-1:0] cpu_reg_array [0:31];
    wire [DATA_WIDTH-1:0] internal_mem_array [0:(CAPACITY / (DATA_WIDTH/8))-1];

    // ���� #2: ���ݲ����������ڴ�ĵ�������˿�
    MEM_Dram #(
        .FILE_PATH(FILE_PATH),
        .DATA_WIDTH(DATA_WIDTH),
        .CAPACITY(CAPACITY)
    ) mem (
        .clk(clk),
        .rst(rst),
        .imem_ift(imem_bus.Slave),
        .dmem_ift(dmem_bus.Slave)
    );

    // ���� #3: ����CPUģ��Ķ˿�����?
    Core cpu (
        .clk(clk),
        .rst_pc(rst),
        .rst_rf(rst),
        .rst_ctrl(rst),
        
        // ʹ������ȷ������ʵ������
        .imem_bus(imem_bus.Master),
        .dmem_bus(dmem_bus.Master),
        
        // ��CPU�ĵ����ź����ӵ�����ģ�������˿�
        .debug_pc_out(debug_pc),
        .debug_instruction(debug_inst),
        .debug_reg_write_enable(),
        .debug_mem_write_enable(),
        .debug_rd_addr_out(),
        .debug_alu_result(),
        .debug_data_to_write(),
        .debug_all_regs(), // ��CPU�Ĵ����������ӵ��ڲ�����
        .wmask(),
        .immediate()
    );
/*
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : reg_export
            assign debug_all_regs[i] = cpu_reg_array[i];
        end
    endgenerate

    genvar j;
    generate
        for (j = 0; j < (CAPACITY / (DATA_WIDTH/8)); j = j + 1) begin : mem_export
            assign debug_mem_content[j] = internal_mem_array[j];
        end
   endgenerate*/
endmodule