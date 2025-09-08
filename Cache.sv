import cache_pkg::*; 

// DataCache Module (2-way Set-Associative, LRU, Write-Back, Single-Cycle Hit)
// state machine : IDLE -> (WB_REQ -> WB_WAIT) -> ALLOC_REQ -> ALLOC_WAIT
// may stay in WAIT states for multiple cycles if memory is not ready
module DataCache #(
  parameter CACHE_CAPACITY = 128, // Cache capacity in bytes
  parameter BLOCK_SIZE     = 16,   // Cache line size in bytes
  parameter DATA_WIDTH_CPU = 32
) (
  input  logic clk,
  input  logic rst,

  input  cpu_req_bus_t  cpu_req,
  output logic          cpu_req_ready,
  output cpu_resp_bus_t cpu_resp,

  output mem_w_req_bus_t mem_w_req,
  output logic           mem_w_req_valid,
  input  logic           mem_w_req_ready,

  input  mem_w_resp_bus_t mem_w_resp,
  input  logic            mem_w_resp_valid,

  output mem_r_req_bus_t mem_r_req,
  output logic           mem_r_req_valid,
  input  logic           mem_r_req_ready,

  input  mem_r_resp_bus_t mem_r_resp,
  input  logic            mem_r_resp_valid
);

  localparam NUM_WAYS       = 2;
  localparam NUM_SETS       = CACHE_CAPACITY / (NUM_WAYS * BLOCK_SIZE);
  localparam BYTES_PER_WORD = DATA_WIDTH_CPU / 8;
  localparam WORDS_PER_BLOCK = BLOCK_SIZE / BYTES_PER_WORD;

  typedef struct packed {
    logic                  valid;
    logic                  dirty;
    logic [TAG_WIDTH-1:0]  tag;
    logic [BLOCK_SIZE*8-1:0] data;
  } cache_line_t; // for a single cache line

  cache_line_t cache_ways [NUM_WAYS][NUM_SETS];
  logic        lru_bits   [NUM_SETS]; // 0: way 0 is LRU, 1: way 1 is LRU

  cache_line_t next_cache_ways [NUM_WAYS][NUM_SETS];
  logic        next_lru_bits   [NUM_SETS];
  logic is_hit;
  logic is_hit_way0, is_hit_way1;

  logic [BLOCK_SIZE*8-1:0] data_from_way0;
  logic [BLOCK_SIZE*8-1:0] data_from_way1;
  logic [BLOCK_SIZE*8-1:0] hit_data_block;
  logic [DATA_WIDTH_CPU-1:0] read_data_word;
  logic [BLOCK_SIZE*8-1:0] write_data_block;

  // State machine
  typedef enum logic [2:0] {
    IDLE, WB_REQ, WB_WAIT, ALLOC_REQ, ALLOC_WAIT
  } cache_state_t;

  cache_state_t current_state, next_state;


  cpu_req_bus_t req_reg; // used for latching req from cpu when miss occurs

  always_ff @(posedge clk or posedge rst) begin
    if (rst) begin
      current_state <= IDLE;
      req_reg <= '{default:'b0};
      for (int i = 0; i < NUM_SETS; i++) begin
        cache_ways[0][i].valid <= 1'b0;
        cache_ways[0][i].data <= 'b0;
        cache_ways[0][i].tag <= 'b0;
        cache_ways[0][i].dirty <= 1'b0;
        cache_ways[1][i].data <= 'b0;
        cache_ways[1][i].tag <= 'b0;
        cache_ways[1][i].dirty <= 1'b0;
        cache_ways[1][i].valid <= 1'b0;
        lru_bits[i]          <= 1'b0;
      end
    end else begin
      current_state <= next_state;
      cache_ways    <= next_cache_ways;
      lru_bits      <= next_lru_bits;

      if (current_state == IDLE && cpu_req.valid && !is_hit) begin
        req_reg <= cpu_req;
      end
    end
  end

 always_comb begin
      is_hit_way0  = 1'b0;
      is_hit_way1  = 1'b0;
      is_hit       = 1'b0;

      is_hit_way0 = cache_ways[0][cpu_req.index].valid && (cache_ways[0][cpu_req.index].tag == cpu_req.tag);
      is_hit_way1 = cache_ways[1][cpu_req.index].valid && (cache_ways[1][cpu_req.index].tag == cpu_req.tag);
      is_hit = is_hit_way0 || is_hit_way1;

      data_from_way0 = cache_ways[0][cpu_req.index].data;
      data_from_way1 = cache_ways[1][cpu_req.index].data;
    
      hit_data_block = is_hit_way0 ? data_from_way0 : data_from_way1;
      read_data_word = hit_data_block >> (cpu_req.offset * 8); 
    
 end

// this is for replacing the data block when write occurs
generate
    for (genvar i = 0; i < WORDS_PER_BLOCK; i++) begin
        for (genvar j = 0; j < BYTES_PER_WORD; j++) begin
            assign write_data_block[(i*BYTES_PER_WORD + j)*8 +: 8] = ( (cpu_req.offset / BYTES_PER_WORD == i) && cpu_req.wmask[j] ) ? cpu_req.wdata[j*8 +: 8] : hit_data_block[(i*BYTES_PER_WORD + j)*8 +: 8];
        end
    end
endgenerate

  logic replace_line,valid_access;
  cache_line_t victim_line;
  
  always_comb begin
    next_state = current_state;
    next_cache_ways = cache_ways;
    next_lru_bits   = lru_bits;
    replace_line = lru_bits[req_reg.index];
    victim_line = replace_line ? cache_ways[1][req_reg.index] : cache_ways[0][req_reg.index];
  
    valid_access = 1'b1;
    cpu_req_ready = 1'b0;
    cpu_resp.valid = 1'b0;
    cpu_resp.hit   = 1'b0;
    cpu_resp.rdata = '0;
    cpu_resp.exception = 1'b0;

    mem_w_req_valid = 1'b0;
    mem_w_req.addr  = 'b0;
    mem_w_req.data  = 'b0;
    mem_w_req.wmask = 'b0;
    mem_r_req_valid = 1'b0;
    mem_r_req.addr  = 'b0;

    case (current_state)
      IDLE: begin
        cpu_req_ready = 1'b1;

        if (cpu_req.valid) begin
          if (cpu_req.read) begin
            case (cpu_req.l_type)
              3'b000,3'b100: begin
                  valid_access = 1'b1;
              end
              3'b001,3'b101: begin
                  valid_access = cpu_req.offset[0] == 1'b0; // half-word aligned
              end
              3'b010: begin
                  valid_access = cpu_req.offset[0] == 1'b0 && cpu_req.offset[1] == 1'b0; // word aligned
              end
              default: begin
                cpu_resp.valid = 1'b1;
                cpu_resp.exception = 1'b1; // unaligned access
                next_state = IDLE;
              end
            endcase
          end else if(cpu_req.write) begin
              case (cpu_req.S_type)
                  3'b000: begin
                      valid_access = 1'b1;
                  end
                  3'b001: begin
                      valid_access = cpu_req.offset[0] == 1'b0; // half-word aligned
                  end
                  3'b010: begin
                      valid_access = cpu_req.offset[0] == 1'b0 && cpu_req.offset[1] == 1'b0; // word aligned
                  end
                  default: begin
                      cpu_resp.valid = 1'b1;
                      cpu_resp.exception = 1'b1; // unaligned access
                      next_state = IDLE;
                  end
              endcase
          end
            if (valid_access) begin
              if (is_hit) begin
                cpu_resp.valid = 1'b1;
                cpu_resp.hit   = 1'b1;
                if (cpu_req.read) begin
                    cpu_resp.rdata = read_data_word;
                end

                next_lru_bits[cpu_req.index] = is_hit_way0 ? 1 : 0;
                
                if (cpu_req.write) begin
                    if (is_hit_way0) begin
                        next_cache_ways[0][cpu_req.index].data  = write_data_block;
                        next_cache_ways[0][cpu_req.index].dirty = 1'b1;
                    end else begin // is_hit_way1
                        next_cache_ways[1][cpu_req.index].data  = write_data_block;
                        next_cache_ways[1][cpu_req.index].dirty = 1'b1;
                    end
                  end
                next_state = IDLE;
              end else begin // miss
                cpu_req_ready = 1'b0; // stop accepting new requests

                if (victim_line.valid && victim_line.dirty) begin
                  next_state = WB_REQ;
                end else begin
                  next_state = ALLOC_REQ;
                end
              end
            end else begin
                cpu_resp.valid = 1'b1;
                cpu_resp.exception = 1'b1; // unaligned access
                next_state = IDLE;
            end
        end
      end

      WB_REQ: begin
        if (req_reg.valid) begin
            mem_w_req_valid = 1'b1;
            mem_w_req.addr = {victim_line.tag, req_reg.index, {OFFSET_WIDTH{1'b0}}};
            mem_w_req.data = victim_line.data;
            mem_w_req.wmask = '1;
        end
        
        if (mem_w_req_ready) begin
            next_state = WB_WAIT;
        end
      end

      WB_WAIT: begin
        if (mem_w_resp_valid) begin
            next_state = ALLOC_REQ;
        end
      end

      ALLOC_REQ: begin
        mem_r_req_valid = 1'b1;
        mem_r_req.addr = {req_reg.tag, req_reg.index, {OFFSET_WIDTH{1'b0}}};
        
        if (mem_r_req_ready) begin
            next_state = ALLOC_WAIT;
        end
      end

      ALLOC_WAIT: begin
        if (mem_r_resp_valid) begin
            logic [BLOCK_SIZE*8-1:0] new_data_block;
            new_data_block = mem_r_resp.rdata;
           
            if (req_reg.write) begin
                for (int i = 0; i < BYTES_PER_WORD; i++) begin
                    if (req_reg.wmask[i]) begin
                        new_data_block[(req_reg.offset + i)*8 +: 8] = req_reg.wdata[i*8 +: 8];
                    end
                end
             end
            if (replace_line == 0) begin
                next_cache_ways[0][req_reg.index].valid = 1'b1;
                next_cache_ways[0][req_reg.index].tag   = req_reg.tag;
                next_cache_ways[0][req_reg.index].data  = new_data_block;
                next_cache_ways[0][req_reg.index].dirty = req_reg.write;
            end else begin
                next_cache_ways[1][req_reg.index].valid = 1'b1;
                next_cache_ways[1][req_reg.index].tag   = req_reg.tag;
                next_cache_ways[1][req_reg.index].data  = new_data_block;
                next_cache_ways[1][req_reg.index].dirty = req_reg.write;
            end
            
            next_lru_bits[req_reg.index] = ~replace_line;

            cpu_resp.valid = 1'b1;
            cpu_resp.hit   = 1'b0;
            if (req_reg.read) begin
                cpu_resp.rdata = new_data_block >> (req_reg.offset * 8);
            end

            next_state = IDLE; 
        end
      end

    endcase
  end

endmodule