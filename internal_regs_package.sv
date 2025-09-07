package internal_regs_package;
    localparam DATA_WIDTH = 32;

typedef struct packed {
  logic [DATA_WIDTH - 1:0] instruction;
  logic [DATA_WIDTH - 1:0] PC;
  logic valid;
} if_id_reg_t;

typedef struct packed {
  logic [$clog2(DATA_WIDTH) - 1:0] rd_addr;
  logic [$clog2(DATA_WIDTH) - 1:0] rs2_addr;
  logic [$clog2(DATA_WIDTH) - 1:0] rs1_addr;

  logic [DATA_WIDTH - 1:0] rs1out;
  logic [DATA_WIDTH - 1:0] rs2out;
  logic [DATA_WIDTH - 1:0] PC;
  logic [DATA_WIDTH - 1:0] imme;

  logic [1:0] ALU_A;
  logic [1:0] ALU_B;
  logic [3:0] ALU_op;
  logic [1:0] WBSel;

  logic Mem_Read;
  logic Mem_Write;
  logic Reg_Write;

  // branch/load/store types (see BC,data_package and transfer for definiton)
  logic [2:0] B_type;
  logic [2:0] l_type;
  logic [2:0] S_type;

  // Auxiliary load/store classification and branch signals
  logic [1:0] is_lstype;
  logic B_enable;
  logic J_taken;

  // Operand needs (for hazard unit) 
  logic need_rs1;
  logic need_rs2;

  logic valid;
  logic exception_valid;

  logic is_csr_inst;
  logic is_mret_inst;
  logic [DATA_WIDTH - 1:0] exception_cause;
  logic [1:0] inst_priv_level;
  // CSR operation code (read/write/clear/set)
  logic [2:0] csr_op;
  logic [DATA_WIDTH - 1:0] inst_pc;

  logic [11:0] csr_addr;
  logic is_ecall_inst;
  logic [DATA_WIDTH - 1:0] csr_rdata;
  logic csr_wen;
  logic [DATA_WIDTH - 1:0] csr_wdata;
} id_ex_reg_t;

typedef struct packed {
  logic [$clog2(DATA_WIDTH) - 1:0] rd_addr;


  logic [DATA_WIDTH - 1:0] data_to_mem;
  // PC forwarded into MEM stage
  logic [DATA_WIDTH - 1:0] PC;
  logic [DATA_WIDTH - 1:0] alu_result;

  logic [DATA_WIDTH / 8 - 1:0] mask;

  logic [1:0] WBSel;

  logic [2:0] l_type;
  logic [2:0] S_type;

  logic Mem_Read;
  logic Mem_Write;
  logic Reg_Write;

  logic valid;

  logic exception_valid;
  logic is_mret_inst;
  logic [DATA_WIDTH - 1:0] exception_cause;
  logic [1:0] inst_priv_level;
  logic [DATA_WIDTH - 1:0] inst_pc;
  logic is_ecall_inst;

  logic csr_wen;                      
  logic [11:0] csr_addr;                  
  logic [DATA_WIDTH - 1:0] csr_wdata;   
  logic [DATA_WIDTH - 1:0] csr_rdata; 
  logic is_csr_inst;  
} ex_mem_reg_t;

typedef struct packed {
  logic [$clog2(DATA_WIDTH) - 1:0] rd_addr;

  logic [DATA_WIDTH - 1:0] data_saved;     // saved register value (rs2 or forwarded)
  logic [DATA_WIDTH - 1:0] alu_result;     
  logic [DATA_WIDTH - 1:0] data_from_mem;  
  logic [DATA_WIDTH - 1:0] PC_4;

  logic [1:0]  WBSel;
  logic Reg_Write;
  logic Mem_Write;

  logic valid;

  logic exception_valid;
  logic [DATA_WIDTH - 1:0] exception_cause;
  logic [1:0] inst_priv_level;
  logic [DATA_WIDTH - 1:0] inst_pc;
  logic is_mret_inst;
  logic is_ecall_inst;

  logic csr_wen;                                           
  logic [11:0] csr_addr;                  
  logic [DATA_WIDTH - 1:0] csr_wdata;   
  logic [DATA_WIDTH - 1:0] csr_rdata;   
  logic is_csr_inst;
} mem_wb_reg_t;
    
endpackage : internal_regs_package