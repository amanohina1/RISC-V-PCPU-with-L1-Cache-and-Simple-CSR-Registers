module Imme_Gen #(
    parameter TYPE_R = 3'b000,
    parameter TYPE_I = 3'b001,
    parameter TYPE_S = 3'b010,
    parameter TYPE_B = 3'b011,
    parameter TYPE_U = 3'b100,
    parameter TYPE_J = 3'b101
) (
    input [31:0] instruction,
    input [2:0] mode,
    output reg [31:0] out_imme
);
    // R-type instruction has no immediate
    // I-type instruction immediate is the upper 12 bits of the instruction
    // L-type instruction immediate is the upper 12 bits of the instruction
    // S-type instruction immediate is high 7 bits + bits 7..11
    // J-type instruction immediate is bits 21..30 + bit20 + bit31
    // B-type instruction immediate is bits 8..11 + 25..30 + bit7 + 12..19 + bit31
    // U-type instruction immediate is the upper 20 bits of the instruction (no sign-extension)
    always@(*)begin
        case (mode)
            TYPE_R: out_imme = 32'b0; 
            TYPE_I: out_imme = {{20{instruction[31]}}, instruction[31:20]};
            TYPE_S: out_imme = {{20{instruction[31]}},instruction[31:25], instruction[11:7]}; 
            TYPE_J: out_imme = {{11{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}; 
            TYPE_B: out_imme =  {{19{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0}; 
            TYPE_U: out_imme = {instruction[31:12], 12'b0};
            default: out_imme = 32'b0; 
        endcase
    end
endmodule