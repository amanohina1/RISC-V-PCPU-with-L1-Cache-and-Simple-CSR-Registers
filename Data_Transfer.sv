module Transfer #(
    parameter DATA_WIDTH = 32,
    parameter CAPACITY = 1024,

    parameter lw  = 3'b010,
    parameter lh  = 3'b001,
    parameter lb  = 3'b000,
    parameter lhu = 3'b101,
    parameter lbu = 3'b100
) (
    input  [DATA_WIDTH - 1:0]     data_in,
    input  [2:0]                  l_type,
    input  [DATA_WIDTH - 1:0]     data_address,
    output reg [DATA_WIDTH - 1:0] data_out
);

    wire [1:0]  offset;
    wire [7:0]  abyte;
    wire [15:0] halfword;

    assign offset   = data_address[1:0];
    assign abyte    = (offset == 2'b00) ? data_in[7:0]
                   : (offset == 2'b01) ? data_in[15:8]
                   : (offset == 2'b10) ? data_in[23:16]
                   : data_in[31:24];
    assign halfword = (offset[1] == 1'b0) ? data_in[15:0] : data_in[31:16];

    always @(*) begin
        case (l_type)
            default: data_out = {DATA_WIDTH{1'b0}};
            lw:      data_out = data_in;
            lh:      data_out = {{16{halfword[15]}}, halfword};
            lb:      data_out = {{24{abyte[7]}}, abyte};
            lhu:     data_out = {16'b0, halfword};
            lbu:     data_out = {24'b0, abyte};
        endcase
    end

endmodule