//bne beq ,bge blt ,bltu bgeu
module Branch_Comparision #(
    parameter bne = 3'b001,
    parameter beq = 3'b000,
    parameter bge = 3'b101,
    parameter blt = 3'b100,
    parameter bltu = 3'b110,
    parameter bgeu = 3'b111
    )(
    input [31:0] rs1,
    input [31:0] rs2,
    input [2:0] B_type,
    output reg branch_taken
);
    always@(*)begin
        branch_taken = 1'b0;
        case (B_type)
        beq: branch_taken = rs1 == rs2;
        bge: branch_taken = $signed(rs1) >= $signed(rs2);
        bgeu: branch_taken = rs1 >= rs2;
        bne: branch_taken = rs1 != rs2;
        blt: branch_taken = $signed(rs1) < $signed(rs2);
        bltu: branch_taken = rs1 < rs2;
        default: branch_taken = 1'b0;
        endcase
    end
endmodule