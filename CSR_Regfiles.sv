module CSR_Regs #(
    parameter DATA_WIDTH = 32,
    parameter CAPACITY = 1024,
    parameter HART_ID = 0
) (
    input  logic rst,
    input  logic clk,

    input  logic [1:0] priv_level, // Current instruction privilege level (from pipeline registers)
    input  logic [11:0] csr_raddr,
    output logic [DATA_WIDTH - 1:0] csr_rdata,

    input  logic csr_wen,
    input  logic [11:0] csr_waddr,
    input  logic [DATA_WIDTH - 1:0] csr_wdata,
    input  logic [DATA_WIDTH - 1:0] exception_tval,

    input  logic exception_commit, // Exception commit signal
    input  logic mret_commit,      // MRET commit signal
    input  logic [DATA_WIDTH - 1:0] exception_pc,    // Exception program counter
    input  logic [DATA_WIDTH - 1:0] exception_cause,
    input  logic inst_commit,
    input  logic mem_stall,

    output logic [DATA_WIDTH - 1:0] handler_addr, // Exception handler entry address (mtvec)
    output logic [DATA_WIDTH - 1:0] mstatus_out,
    output logic [DATA_WIDTH - 1:0] mret_out
);

    reg [DATA_WIDTH - 1:0] mtvec;   // 0x305
    reg [DATA_WIDTH - 1:0] mepc;    // 0x341
    reg [DATA_WIDTH - 1:0] mcause;  // 0x342
    reg [DATA_WIDTH - 1:0] mstatus; // 0x300
    reg [DATA_WIDTH - 1:0] mtval;   // 0x343
    reg [DATA_WIDTH - 1:0] new_mstatus;
    reg [63:0] mcycle;  // 0xB00
    reg [63:0] minstret; // 0xB02
    reg [DATA_WIDTH - 1:0] misa;    // 0x301
    reg [DATA_WIDTH - 1:0] mie;     // 0x304
    reg [DATA_WIDTH - 1:0] mip;     // 0x344 (Partially writable)
    reg [DATA_WIDTH - 1:0] mscratch;// 0x340
    reg [DATA_WIDTH - 1:0] stall_count;

    logic [DATA_WIDTH-1:0] mstatus_mask = 32'h00001888;

    localparam MSTATUS = 12'h300;
    localparam MISA      = 12'h301; 
    localparam MIE       = 12'h304;
    localparam MSCRATCH  = 12'h340; 
    localparam MIP       = 12'h344;
    localparam MTVEC   = 12'h305;
    localparam MEPC    = 12'h341;
    localparam MCAUSE  = 12'h342;
    localparam MTVAL   = 12'h343;
    localparam MCYCLE  = 12'hB00;
    localparam MINSTRET= 12'hB02;

    localparam MVENDORID = 12'hF11; 
    localparam MARCHID   = 12'hF12; 
    localparam MIMPID    = 12'hF13;
    localparam MHARTID   = 12'hF14; 

    localparam CYCLE     = 12'hC00; 
    localparam INSTRET   = 12'hC02; 

    localparam MCYCLEH   = 12'hB80;
    localparam MINSTRETH = 12'hB82;
    
    localparam MEM_STALL = 12'hB83;
    always_comb begin
        logic [DATA_WIDTH - 1:0] rdata_from_regs;
        mstatus_out = mstatus;

        case (csr_raddr)
            MSTATUS:   rdata_from_regs = mstatus;
            MTVEC:     rdata_from_regs = mtvec;
            MEPC:      rdata_from_regs = mepc;
            MCAUSE:    rdata_from_regs = mcause;
            MTVAL:     rdata_from_regs = mtval;
            MCYCLE:    rdata_from_regs = mcycle;
            MINSTRET:  rdata_from_regs = minstret;

            MISA:      rdata_from_regs = misa;
            MIE:       rdata_from_regs = mie;
            MIP:       rdata_from_regs = mip;
            MSCRATCH:  rdata_from_regs = mscratch;

            MVENDORID: rdata_from_regs = 32'hDEADBEEF; 
            MARCHID:   rdata_from_regs = 32'h00000001;
            MIMPID:    rdata_from_regs = 32'h00000001; 
            MHARTID:   rdata_from_regs = HART_ID;

            MCYCLEH:   rdata_from_regs = mcycle[63:32];
            MINSTRETH: rdata_from_regs = minstret[63:32];

            CYCLE:     rdata_from_regs = mcycle[31:0];
            INSTRET:   rdata_from_regs = minstret[31:0]; 
            MEM_STALL: rdata_from_regs = stall_count[31:0]; 
            default:   rdata_from_regs = {DATA_WIDTH{1'b0}};
        endcase

        if (csr_wen && (csr_waddr == csr_raddr))
            csr_rdata = csr_wdata;
        else
            csr_rdata = rdata_from_regs;
    end

    assign handler_addr = mtvec;
    assign mret_out     = mepc;

    // inst_commit: increment minstret when an instruction is committed (connected from Datapath)

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            mstatus <= '0;
            mtvec   <= '0;
            mepc    <= '0;
            mcause  <= '0;
            mtval   <= '0;
            mcycle  <= '0;
            minstret<= '0;
            mie     <= '0;
            mip     <= '0;
            mscratch<= '0;
            misa    <= 32'h40000104; // RV32IM
            stall_count <= '0;
        end else begin
            // always count cycles
            mcycle <= mcycle + 1;
            if (mem_stall)
                stall_count <= stall_count + 1;
            // increment retired-instruction counter when datapath signals a commit
            if (inst_commit)
                minstret <= minstret + 1;

            if (exception_commit) begin
                mepc   <= exception_pc;
                mcause <= exception_cause;
                mtval  <= exception_tval;

                // update mstatus on exception entry
                new_mstatus <= mstatus;
                new_mstatus[12:11] <= priv_level; // set MPP
                new_mstatus[7]      <= mstatus[3]; // MPIE <- MIE
                new_mstatus[3]      <= 1'b0;       // MIE <- 0
                mstatus <= new_mstatus;

            end else if (mret_commit) begin
                new_mstatus        <= mstatus;
                new_mstatus[3]     <= mstatus[7];
                new_mstatus[7]     <= 1'b1;
                new_mstatus[12:11] <= 2'b00;

                mstatus <= new_mstatus;

            end

            // CSR write (write takes precedence over auto-increment)
            if (csr_wen && priv_level >= csr_waddr[9:8]) begin
                case (csr_waddr)
                    MSTATUS: mstatus   <= (csr_wdata & mstatus_mask) | (mstatus & ~mstatus_mask);
                    MTVEC:   mtvec     <= csr_wdata;
                    MEPC:    mepc      <= csr_wdata;
                    MCAUSE:  mcause    <= csr_wdata;
                    MTVAL:   mtval     <= csr_wdata;
                    MCYCLE:  mcycle    <= csr_wdata;
                    MINSTRET: minstret <= csr_wdata;
                    MISA:     ; 
                    MIE:      mie       <= csr_wdata;
                    MSCRATCH: mscratch  <= csr_wdata;
                    MIP:      mip[3]    <= csr_wdata[3];
                    MEM_STALL: stall_count <= csr_wdata;
                    default: ; // do nothing for read-only or unsupported CSRs
                endcase
            end
        end
    end

endmodule