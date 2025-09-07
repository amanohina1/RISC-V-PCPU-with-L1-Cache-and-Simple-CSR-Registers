module MUX2to1 (
    input wire [31:0] A,
    input wire [31:0] B,
    input wire  sel,
    output reg [31:0] out
);
    always@(*) begin
        case(sel)
            1'b0: out = A;
            1'b1: out = B;
            default: out = 32'b0; // Default case to avoid latches
        endcase
    end
endmodule