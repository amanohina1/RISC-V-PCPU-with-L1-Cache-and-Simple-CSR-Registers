module DataPkg #(
    parameter DATA_WIDTH = 32
) (
    input [DATA_WIDTH - 1:0] data_address,
    input [DATA_WIDTH - 1:0] data_reg,
    input [2:0] op,
    output reg [DATA_WIDTH - 1:0] data_mem,
    output reg [DATA_WIDTH / 8 - 1:0] mask
);
    wire [DATA_WIDTH - 1:0] low_aligned_address;
    wire [1:0] offset;
    assign low_aligned_address = {data_address[DATA_WIDTH - 1:2],2'b00};
    assign offset = data_address - low_aligned_address;
    always@(*)begin
        case (op)
            3'b000: begin //sb
                data_mem = {data_reg[7:0],data_reg[7:0],data_reg[7:0],data_reg[7:0]};
                case (offset)
                    2'b00: mask = 4'b0001; 
                    2'b01: mask = 4'b0010; 
                    2'b10: mask = 4'b0100; 
                    2'b11: mask = 4'b1000;  
                endcase
            end
            3'b001: begin //sh
                data_mem = {data_reg[15:0],data_reg[15:0]};
                case (offset)
                    2'b00: mask = 4'b0011; 
                    2'b10: mask = 4'b1100;
                    default: mask = 4'b0000; 
                endcase
            end
            3'b010: begin //sw
                data_mem = data_reg;
                mask = 4'b1111;
            end
            default: begin
                mask = 4'b0000;
                data_mem = 32'h00000000;
            end
        endcase
    end 
endmodule