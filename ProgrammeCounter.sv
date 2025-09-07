module PC (
    input clk,
    input reset,
    input [31:0] next_pc,
    input halt,
    output reg [31:0] addr
);
    reg [31:0] present_addr;
    always@(posedge clk)begin
        if (reset) begin
            present_addr <= 32'h00000000; // Reset address to 0
        end else if (halt) begin
            present_addr <= present_addr; 
        end else begin
            present_addr <= next_pc; 
        end
    end
    assign addr = present_addr; // Output the current address
endmodule