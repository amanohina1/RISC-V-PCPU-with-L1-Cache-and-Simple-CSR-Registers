module Control #(
        parameter R = 7'b0110011,
        parameter I = 7'b0010011,
        parameter L = 7'b0000011,
        parameter S = 7'b0100011,
        parameter B = 7'b1100011,
        parameter LUI = 7'b0110111,
        parameter AUIPC = 7'b0010111,
        parameter JAL = 7'b1101111,
        parameter JALR = 7'b1100111,
        parameter SYSTEM = 7'b1110011,

        parameter bne = 3'b001,
        parameter beq = 3'b000,
        parameter bge = 3'b101,
        parameter blt = 3'b100,
        parameter bltu = 3'b110,
        parameter bgeu = 3'b111,

        parameter mul = 3'b000,
        parameter mulh = 3'b001,
        parameter mulhsu = 3'b010,
        parameter mulhu = 3'b011,
        parameter div = 3'b100,
        parameter divu = 3'b101,
        parameter rem = 3'b110,
        parameter remu = 3'b111,

        parameter M_MODE = 2'b11,
        parameter U_MODE = 2'b00,

        parameter DATA_WIDTH = 32
    ) (
        input  logic clk,
        input  logic rst,

        input  [6:0] funct7,
        input  [2:0] funct3,
        input  [6:0] opcode,
        input  logic [11:0] csr_addr,

        input  [DATA_WIDTH - 1:0] mstatus,
        input  exception,
        input  mret_out,

        output reg RegWrite,
        output reg MW,

        output reg jump,
        output reg B_enable,
        output reg [1:0] is_lstype,
        output reg MRead,
        output reg need_rs1,
        output reg need_rs2,
        output reg valid,

        output reg [1:0] A_Select,
        output reg [1:0] B_Select,
        output reg [3:0] Op,
        output reg [1:0] WBSelect,
        output reg [2:0] ImmeSelect,
        output reg [2:0] B_type,
        output reg [1:0] S_type,
        output reg [2:0] l_type,

        output reg exception_valid,
        output reg is_csr,
        output reg is_mret,
        output reg [DATA_WIDTH - 1:0] exception_cause,
        output logic [1:0] current_privilege,
        output logic [2:0] csr_op,
        output reg is_ecall_inst
    );
    // A_Select : 0 for rs1 ,1 for PC
    // B_Select : 0 for rs2 ,1 for imme
    always_ff @(posedge clk) begin
        if (rst) begin
            current_privilege <= M_MODE; // CPU starts in Machine mode (highest privilege)
        end else if (exception) begin
            current_privilege <= M_MODE; // Any exception forces entry into Machine mode
        end else if (mret_out) begin // when an MRET instruction is committed
            // Return to the previously recorded privilege
            current_privilege <= mstatus[12:11]; // Read MPP bits
        end
    end
    always @(*) begin

        jump = 1'b0;
        B_type = 3'b010;
        MW = 1'b0;
        RegWrite = 1'b0;
        WBSelect = 2'bxx;
        ImmeSelect = 3'bxxx;
        B_Select = 2'bxx;
        A_Select = 2'bxx;
        Op = 4'b1111;
        S_type = 2'b11;
        B_enable = 1'b0;
        l_type = 3'b111;
        MRead = 1'b0;
        need_rs1 = 1'b0;
        need_rs2 = 1'b0;
        is_lstype = 1'b0;
        is_csr = 1'b0;
        is_mret = 1'b0;
        exception_valid = 1'b0;
        csr_op = 3'bxxx;
        exception_cause = 32'd0;
        is_ecall_inst = 1'b0;
        valid = 1'b1;

        case (opcode)
            R:begin
                Op = {funct7[5], funct3};
                MW = 1'b0;
                RegWrite = 1'b1;
                WBSelect = 2'b01;
                ImmeSelect = 3'b000;
                A_Select = 2'b00; //rs1
                B_Select = 2'b00; //rs2
                need_rs1 = 1'b1;
                need_rs2 = 1'b1;
            end
            I:begin
                B_Select = 2'b01; //imme
                A_Select = 2'b00; //rs1
                need_rs1 = 1'b1;
                MW = 1'b0;
                RegWrite = 1'b1;
                WBSelect = 2'b01;
                ImmeSelect = 3'b001; 
                case (funct3)
                    3'b000: Op = 4'b0000; // addi
                    3'b010: Op = 4'b0010; // slti
                    3'b011: Op = 4'b0011; // sltiu
                    3'b100: Op = 4'b0100; // xori
                    3'b110: Op = 4'b0110; // ori
                    3'b111: Op = 4'b0111; // andi
                    3'b001: Op = 4'b0001; // slli
                    3'b101: Op = {funct7[5], funct3}; // srli/srai
                    default: Op = 4'bxxxx;
                endcase
            end
            L:begin
                B_Select = 2'b01; //imme
                A_Select = 2'b00; //rs1
                need_rs1 = 1'b1;
                MW = 1'b0;
                RegWrite = 1'b1;
                WBSelect = 2'b00;//from mem
                ImmeSelect = 3'b001; 
                Op = 4'b0000; //add
                l_type = funct3;
                MRead = 1'b1;
                is_lstype = 2'b01;
            end
            S:begin
                B_Select = 2'b01; //imme
                A_Select = 2'b00; //rs1
                need_rs1 = 1'b1;
                MW = 1'b1;
                RegWrite = 1'b0;
                WBSelect = 2'b11;
                ImmeSelect = 3'b010; 
                Op = 4'b0000; //add
                case (funct3) 
                    3'b000: S_type = 2'b00;
                    3'b001: S_type = 2'b01;
                    3'b010: S_type = 2'b10;
                    default: S_type = 2'b11;
                endcase
                need_rs2 = 1'b1;
                is_lstype = 2'b10;
            end
            B:begin
                B_type = funct3;
                B_Select = 2'b01; //imme
                A_Select = 2'b01; //pc
                MW = 1'b0;
                RegWrite = 1'b0;
                WBSelect = 2'b11;
                ImmeSelect = 3'b011; 
                Op = 4'b0000; 
                B_enable = 1'b1;
                need_rs2 = 1'b1;
                need_rs1 = 1'b1;
            end
            LUI:begin
                B_Select = 2'b01; //imme
                A_Select = 2'b00; //dont care
                MW = 1'b0;
                RegWrite = 1'b1;
                WBSelect = 2'b01;
                ImmeSelect = 3'b100; 
                Op = 4'b1110; //B
            end
            AUIPC:begin
                B_Select = 2'b01; //imme
                A_Select = 2'b01; //PC
                MW = 1'b0;
                RegWrite = 1'b1;
                WBSelect = 2'b01;
                ImmeSelect = 3'b100; 
                Op = 4'b0000; //add
            end
            JAL:begin
                B_Select = 2'b01; //imme
                A_Select = 2'b01; //PC
                MW = 1'b0;
                RegWrite = 1'b1;
                WBSelect = 2'b10;
                ImmeSelect = 3'b101; 
                Op = 4'b0000; //add
                jump = 1'b1;
            end
            JALR: begin
                B_Select = 2'b01; //imme
                A_Select = 2'b00; //rs1
                need_rs1 = 1'b1;
                MW = 1'b0;
                RegWrite = 1'b1;
                WBSelect = 2'b10;
                ImmeSelect = 3'b001; //I
                Op = 4'b0000; //add
                jump = 1'b1;
            end

            SYSTEM: begin
                case (funct3)
                    3'b000: begin //ECALL, EBREAK, and MRET
                    case (csr_addr) 
                        12'h000: begin //ECALL
                        exception_valid = 1'b1;
                        is_ecall_inst   = 1'b1;
                        case(current_privilege)
                            U_MODE: exception_cause = 32'd8;  
                            default: exception_cause = 32'd11;
                        endcase
                        end
                
                        12'h001: begin //EBREAK
                            exception_valid = 1'b1;
                            exception_cause = 32'd3; // Breakpoint
                            is_ecall_inst   = 1'b1;
                        end
                
                        12'h302: begin //MRET
                            if (current_privilege == U_MODE) begin
                                exception_valid = 1'b1;
                                valid = 1'b0;
                                is_ecall_inst = 1'b1;
                                exception_cause = 32'd2; // Illegal Instruction
                            end else begin
                                exception_valid = 1'b0;
                                is_mret = 1'b1;
                            end
                        end
                        default: begin
                            exception_valid = 1'b1;
                            valid = 1'b0;
                            is_ecall_inst = 1'b1;
                            exception_cause = 32'd2; // Illegal Instruction
                        end
                    endcase
                    end

                3'b001, 3'b010, 3'b011, 3'b101, 3'b110, 3'b111: begin // These are all CSR instructions
                    if (current_privilege == U_MODE) begin
                        exception_valid = 1'b1;
                        valid = 1'b0;
                        is_ecall_inst = 1'b1;
                        exception_cause = 32'd2; // Illegal Instruction
                    end else begin
                        // Valid CSR instruction in M-Mode
                        exception_valid = 1'b0;
                        is_csr = 1'b1;
                        csr_op = funct3;
                        RegWrite = 1'b1; // Writes CSR old value to rd
                        need_rs1 = 1'b1;
                        A_Select = 2'b00;
                        WBSelect = 2'b01;
                        Op = 4'b0000;
                    end
                end

                default: begin
                    exception_valid = 1'b1;              
                    valid = 1'b0;
                    exception_cause = 32'd2; // Illegal Instruction
                    is_ecall_inst = 1'b1;
                end
                endcase
            end

            default: begin
                is_ecall_inst = 1'b1;
                valid = 1'b0;
                exception_valid = 1'b1;
                exception_cause = 32'd2;
            end
        endcase
    end
endmodule