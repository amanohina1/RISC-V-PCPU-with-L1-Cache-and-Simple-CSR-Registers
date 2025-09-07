`timescale 1ns / 1ps

module MEM_Dram #(
    parameter FILE_PATH = "C:/Users/Amano/Desktop/PLCPU/project_1/test.hex",
    parameter DATA_WIDTH = 32,
    parameter CAPACITY = 4096
) (
    input clk,
    input rst,
    Mem_ift.Slave imem_ift,
    Mem_ift.Slave dmem_ift
);

    localparam BYTE_NUM = DATA_WIDTH / 8;
    localparam DEPTH = CAPACITY / BYTE_NUM;
    localparam ADDR_WIDTH = $clog2(DEPTH);
    localparam ADDR_OFFSET = $clog2(BYTE_NUM); 
    
    wire dram_data_r_request_ready;
    wire dram_data_r_reply_valid;
    wire dram_data_w_request_ready;
    wire dram_data_w_reply_valid;

    DRAM_data #(
        .FILE_PATH(),
        .DATA_WIDTH(128),
        .ADDR_WIDTH(32),
        .CAPACITY(CAPACITY)
    ) DRAM_data (
        .clk(clk),
        .rst(rst),        
        .wen(dmem_ift.w_request_valid),
        .ren(dmem_ift.r_request_valid),
        .waddr({dmem_ift.w_request_bits.waddr[ADDR_WIDTH-1:4],4'b0}),
        .wdata(dmem_ift.w_request_bits.wdata),
        .wmask({(128 / 8){1'b1}}),

        .raddr1({dmem_ift.r_request_bits.raddr[ADDR_WIDTH+4-1:4],4'b0}),
        .rdata1(dmem_ift.r_reply_bits.rdata),
        
        .r_req_ready_out(dram_data_r_request_ready), // <-- 新增输出
        .r_rep_valid_out(dram_data_r_reply_valid),   // <-- 新增输出
        .w_req_ready_out(dram_data_w_request_ready), // <-- 新增输出
        .w_rep_valid_out(dram_data_w_reply_valid)   // <-- 新增输出
    );

    DRAM_inst #(
        .FILE_PATH(FILE_PATH),
        .DATA_WIDTH(32),

        .CAPACITY(2048)
    ) DRAM_inst (
        .clk(clk),
        .wen(1'b0),
        .waddr1(imem_ift.w_request_bits.waddr[ADDR_WIDTH-1:ADDR_OFFSET]),
        .wdata1(imem_ift.w_request_bits.wdata[31:0]),
        .raddr0(imem_ift.r_request_bits.raddr[ADDR_WIDTH+ADDR_OFFSET-1:ADDR_OFFSET]),
        .rdata0(imem_ift.r_reply_bits.rdata[31:0])
    );

    // ready valid signals
    assign dmem_ift.w_request_ready = dram_data_w_request_ready;
    assign dmem_ift.w_reply_valid = dram_data_w_reply_valid;
    assign dmem_ift.w_reply_bits = '{bresp:2'b00};

    assign dmem_ift.r_request_ready = dram_data_r_request_ready;
    assign dmem_ift.r_reply_bits.rresp = 2'b00;
    assign dmem_ift.r_reply_valid = dram_data_r_reply_valid;

    //below keep unchanged
    assign imem_ift.w_request_ready = 1'b0;
    assign imem_ift.w_reply_valid = 1'b0;
    assign imem_ift.w_reply_bits = '{bresp:2'b00};

    assign imem_ift.r_request_ready = 1'b1;
    assign imem_ift.r_reply_bits.rresp = 2'b00;
    assign imem_ift.r_reply_valid = 1'b1;
    
endmodule

module DRAM_data #(
    parameter FILE_PATH = "",
    parameter ADDR_WIDTH = 32,
    parameter DATA_WIDTH = 128,
    parameter CAPACITY = 4096
) (
    input  clk,
    input  rst,
    input  wen, 
    input  ren, 

    input  [$clog2(CAPACITY/8) - 1:0] waddr,
    input  [DATA_WIDTH - 1:0] wdata,
    input  [DATA_WIDTH / 8 - 1:0] wmask,
    input  [$clog2(CAPACITY/8) - 1:0] raddr1,

    output reg [DATA_WIDTH-1:0] rdata1,
    output reg r_req_ready_out,
    output reg r_rep_valid_out,
    output reg w_req_ready_out,
    output reg w_rep_valid_out
    
);

    localparam BYTE_NUM = DATA_WIDTH / 8;
    localparam DEPTH = CAPACITY / BYTE_NUM;
    
    (* ram_style = "block" *) 
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [$clog2(DEPTH)-1:0] raddr_reg; 
    initial begin
        $readmemh(FILE_PATH, mem);
    end

    // 读状态机
    typedef enum logic { R_IDLE, R_DONE } r_state_t;
    r_state_t r_current_state, r_next_state;

    // 写状态机
    typedef enum logic { W_IDLE, W_DONE } w_state_t;
    w_state_t w_current_state, w_next_state;

    // 状�?�更�??? (时序逻辑)
    always @(posedge clk) begin
        if (rst) begin
            r_current_state <= R_IDLE;
            w_current_state <= W_IDLE;
            r_rep_valid_out <= 1'b0; 
            w_rep_valid_out <= 1'b0;
        end else begin
            r_current_state <= r_next_state;
            w_current_state <= w_next_state;
            
            r_rep_valid_out <= (r_current_state == R_DONE);
            w_rep_valid_out <= (w_current_state == W_DONE);
            
        if (r_current_state == R_IDLE && ren) begin
            raddr_reg <= raddr1;// 在数据准备好时读�???
        end
            rdata1 <= mem[raddr_reg];
            
        if (w_req_ready_out && wen) begin
           for (integer i = 0; i < BYTE_NUM; i = i + 1) begin
                if (wmask[i]) begin
                    mem[waddr][i*8 +: 8] <= wdata[i*8 +: 8];
                end
            end
        end
        end
    end

    // 下一状�?�和输出逻辑 (组合逻辑)
    always_comb begin
        r_next_state = r_current_state;
        w_next_state = w_current_state;

        r_req_ready_out = 1'b0; // 默认不准备好
        w_req_ready_out = 1'b0; // 默认不准备好

        // 读状态机逻辑
        case (r_current_state)
            R_IDLE: begin
                r_req_ready_out = 1'b1; // 空闲时可以接受请�???
                if (ren) begin // 接受请求
                    r_next_state = R_DONE;
                end
            end
            R_DONE: begin 
                r_next_state = R_IDLE;
            end
            default: r_next_state = R_IDLE;
        endcase

        // 写状态机逻辑
        case (w_current_state)
            W_IDLE: begin
                w_req_ready_out = 1'b1; // 空闲时可以接受请�???
                if (wen) begin // 接受请求
                    w_next_state = W_DONE;
                end
            end
            W_DONE: begin
                w_next_state = W_IDLE;
            end
            default: w_next_state = W_IDLE;
        endcase
    end

endmodule

module DRAM_inst #(
    parameter FILE_PATH = "C:/Users/Amano/Desktop/PLCPU/project_1/test.hex",
    parameter DATA_WIDTH = 32,
    parameter CAPACITY = 2048
) (
    input  clk,
    input  wen,
    input  [$clog2(CAPACITY/(DATA_WIDTH/8))-1:0] raddr0,
    input  [$clog2(CAPACITY/(DATA_WIDTH/8))-1:0] waddr1,
    input  [DATA_WIDTH-1:0] wdata1,
    output reg [DATA_WIDTH-1:0] rdata0
);

    localparam BYTE_NUM = DATA_WIDTH / 8;
    localparam DEPTH = CAPACITY / BYTE_NUM;
     
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    
    always_ff @(posedge clk) begin
        if (wen) begin // ֻ�ڽ�������ʱд��
         mem[waddr1] <= wdata1;
        end
    end
    
    initial begin
        $readmemh(FILE_PATH, mem);
    end

    always_comb begin
        rdata0 = mem[raddr0];
    end
endmodule
