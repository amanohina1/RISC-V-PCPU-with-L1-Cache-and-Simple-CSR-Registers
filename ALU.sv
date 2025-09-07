module ALU (
    input [31:0] A,
    input [31:0] B,
    input [3:0] control,
    output reg [31:0] result,
    output is_zero
);
    always @(*) begin
       case (control)
        4'b0000: result = A + B;
        4'b1000: result = A + (B ^ 32'hFFFFFFFF) + 1; 
        4'b0111: result = A & B;
        4'b0110: result = A | B;
        4'b0100: result = A ^ B;
        4'b0010: result = $signed(A) < $signed(B) ? 32'd1 : 32'd0; //slt
        4'b0011: result = (A < B) ? 32'h1 : 32'h0; // sltu
        4'b0001: result = A << B[4:0]; // sll
        4'b0101: result = A >> B[4:0]; // srl
        4'b1101: result = $signed(A) >>> B[4:0]; // sra
        4'b1110: result = B;//For lui
        default: result = 0;
    endcase
    end
    assign is_zero = result == 32'b0;
endmodule