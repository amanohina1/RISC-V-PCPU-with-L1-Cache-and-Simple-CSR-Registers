package cache_pkg;

    localparam ADDR_WIDTH     = 8;
    localparam DATA_WIDTH_M   = 128; // datawidth of mem
    localparam DATA_WIDTH_CPU = 32;  // datawidth of CPU

    localparam TAG_WIDTH      = 6;
    localparam INDEX_WIDTH    = 2;
    localparam OFFSET_WIDTH   = 4;

    typedef struct packed {
        logic                  valid;
        logic [TAG_WIDTH - 1:0]  tag;
        logic [INDEX_WIDTH - 1:0] index;
        logic [OFFSET_WIDTH - 1:0] offset;
        logic [DATA_WIDTH_CPU - 1:0] wdata;
        logic [DATA_WIDTH_CPU/8 - 1:0] wmask;
        logic                  write;
        logic                  read;
        logic [2:0]                 S_type;
        logic [2:0]                 l_type;
    } cpu_req_bus_t;

    typedef struct packed {
        logic                  valid;
        logic [DATA_WIDTH_CPU - 1:0] rdata;
        logic                  hit;
        logic                  exception;
    } cpu_resp_bus_t;

    typedef struct packed {
        logic [ADDR_WIDTH - 1:0]   addr;
        logic [DATA_WIDTH_M - 1:0] data;
        logic [DATA_WIDTH_M/8 - 1:0] wmask; 
    } mem_w_req_bus_t;

    typedef struct packed {
        logic [1:0] bresp; // omitted
    } mem_w_resp_bus_t;

    typedef struct packed {
        logic [ADDR_WIDTH - 1:0] addr;
    } mem_r_req_bus_t;

    typedef struct packed {
        logic [DATA_WIDTH_M - 1:0] rdata;
        logic [1:0]                rresp; // omitted
    } mem_r_resp_bus_t;

endpackage