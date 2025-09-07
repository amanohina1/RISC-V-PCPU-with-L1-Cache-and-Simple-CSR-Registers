// SystemVerilog uses `logic` instead of `wire` and `reg`
module MDU #(
    parameter DATA_WIDTH = 32,
    // --- Supported funct3 values ---
    parameter OP_MUL    = 3'b000,
    parameter OP_DIV    = 3'b100,
    parameter OP_DIVU   = 3'b101,
    parameter OP_REM    = 3'b110,
    parameter OP_REMU   = 3'b111
) (
    // Port declarations updated to use 'logic'
    input                       clk,
    input                       rst,
    input                       start,
    input [2:0]                op,
    input [DATA_WIDTH - 1:0] A,
    input [DATA_WIDTH - 1:0] B,
    
    output reg [DATA_WIDTH - 1:0]   C,
    output busy
);

    // --- State Machine ---
    localparam S_BEGIN = 2'b00;
    localparam S_CALC  = 2'b01;
    localparam S_DONE  = 2'b10;

    // Internal registers are now 'logic'
    logic [1:0] state, next_state;
    logic [2:0] op_reg;
    logic signed [DATA_WIDTH*2-1:0] product;
    logic [DATA_WIDTH-1:0] multiplicand;
    logic [DATA_WIDTH-1:0]   divisor;
    logic [DATA_WIDTH*2:0]   rem_quot_reg;
    logic                    sign_q;
    logic                    sign_r;
    logic [5:0]              counter;

    // --- State Transition Logic ---
    // (SystemVerilog best practice is to use 'always_comb' for combinational logic)
    always_comb begin
        next_state = state;
        case (state)
            S_BEGIN: if (start) next_state = S_CALC;
            S_CALC:  if (counter == DATA_WIDTH) next_state = S_DONE;
            S_DONE:  next_state = S_BEGIN;
        endcase
    end
    
    // --- Pre-computation and Intermediate Wires (now also logic) ---
    logic is_mul_op;
    logic is_signed_div_rem;
    logic [DATA_WIDTH-1:0] abs_A;
    logic [DATA_WIDTH-1:0] abs_B;
    logic signed [DATA_WIDTH:0] current_rem;
    logic signed [DATA_WIDTH:0] shifted_rem;
    logic signed [DATA_WIDTH:0] next_rem_if_add;
    logic signed [DATA_WIDTH:0] next_rem_if_sub;
    logic [DATA_WIDTH-1:0]      next_quotient_bits;
    logic signed [DATA_WIDTH:0] next_rem;

    assign is_mul_op             = (op == OP_MUL);
    assign is_signed_div_rem     = (op == OP_DIV || op == OP_REM);
    assign abs_A                 = A[DATA_WIDTH-1] ? -A : A;
    assign abs_B                 = B[DATA_WIDTH-1] ? -B : B;
    
    assign current_rem           = rem_quot_reg[DATA_WIDTH*2:DATA_WIDTH];
    assign shifted_rem           = {current_rem[DATA_WIDTH-1:0], rem_quot_reg[DATA_WIDTH-1]};
    assign next_rem_if_add       = shifted_rem + {1'b0, divisor};
    assign next_rem_if_sub       = shifted_rem - {1'b0, divisor};
    assign next_quotient_bits    = {rem_quot_reg[DATA_WIDTH-2:0], !(current_rem[DATA_WIDTH] ? next_rem_if_add[DATA_WIDTH] : next_rem_if_sub[DATA_WIDTH])};
    assign next_rem              = current_rem[DATA_WIDTH] ? next_rem_if_add : next_rem_if_sub;

    // --- State and Data Register Update Logic ---
    // (SystemVerilog best practice is to use 'always_ff' for sequential logic)
    always_ff @(posedge clk) begin
        if (rst) begin
            state        <= S_BEGIN;
            counter      <= 0;
            product      <= 0;
            rem_quot_reg <= 0;
        end else begin
            state <= next_state;

            if (state == S_BEGIN && next_state == S_CALC) begin
                op_reg  <= op;
                counter <= 1;
                
                if (is_mul_op) begin
                    product      <= {{32{B[31]}}, B};
                    multiplicand <= A;
                end else begin
                    sign_q       <= is_signed_div_rem ? (A[DATA_WIDTH-1] ^ B[DATA_WIDTH-1]) : 1'b0;
                    sign_r       <= is_signed_div_rem ? A[DATA_WIDTH-1] : 1'b0;
                    rem_quot_reg <= {{(DATA_WIDTH+1){1'b0}}, (is_signed_div_rem ? abs_A : A)};
                    divisor      <= (is_signed_div_rem ? abs_B : B);
                end
            end
            
            if (state == S_CALC) begin
                if (op_reg == OP_MUL) begin
                    if (product[0]) begin
                         product[DATA_WIDTH*2-1:DATA_WIDTH] <= product[DATA_WIDTH*2-1:DATA_WIDTH] + multiplicand;
                    end
                    product <= $signed(product) >>> 1;
                end else begin 
                     rem_quot_reg[DATA_WIDTH*2:DATA_WIDTH] <= next_rem;
                     rem_quot_reg[DATA_WIDTH-1:0]          <= next_quotient_bits;
                end
                counter <= counter + 1;
            end
        end
    end
    
    // --- Final Correction and Output Logic ---
    logic [DATA_WIDTH-1:0]        unsigned_quotient;
    logic signed [DATA_WIDTH:0]   rem_raw;
    logic signed [DATA_WIDTH:0]   corrected_rem_full;
    logic signed [DATA_WIDTH-1:0] unsigned_remainder;
    logic [DATA_WIDTH-1:0]        signed_quotient;
    logic [DATA_WIDTH-1:0]        signed_remainder;

    assign unsigned_quotient      = rem_quot_reg[DATA_WIDTH-1:0];
    assign rem_raw                = rem_quot_reg[DATA_WIDTH*2:DATA_WIDTH];
    assign corrected_rem_full     = rem_raw + {1'b0, divisor};
    assign unsigned_remainder     = rem_raw[DATA_WIDTH] ? corrected_rem_full[DATA_WIDTH-1:0] : rem_raw[DATA_WIDTH-1:0];
    
    assign signed_quotient        = (sign_q) ? -unsigned_quotient : unsigned_quotient;
    assign signed_remainder       = (sign_r) ? -unsigned_remainder : signed_remainder;

    always_comb begin
        if (state == S_DONE) begin
            case (op_reg)
                OP_MUL:  C = product[DATA_WIDTH-1:0];
                OP_DIVU: C = (B == 0) ? ~0 : unsigned_quotient;
                OP_REMU: C = (B == 0) ? A  : unsigned_remainder;
                OP_DIV:  C = (B == 0) ? ~0 : signed_quotient;
                OP_REM:  C = (B == 0) ? A  : signed_remainder;
                default: C = {DATA_WIDTH{1'bx}};
            endcase
        end else begin
            C = {DATA_WIDTH{1'bx}};
        end
    end
    
    assign busy = (state != S_BEGIN);

endmodule