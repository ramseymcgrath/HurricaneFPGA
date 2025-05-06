module bram_dual_port #(
    parameter ADDR_WIDTH = 14, // 16KB = 2^14
    parameter DATA_WIDTH = 8
)(
    input wire                  clk,
    // Port A (Write)
    input wire                  we_a,
    input wire  [ADDR_WIDTH-1:0] addr_a,
    input wire  [DATA_WIDTH-1:0] din_a,

    // Port B (Read)
    input wire  [ADDR_WIDTH-1:0] addr_b,
    output reg  [DATA_WIDTH-1:0] dout_b
);

    (* ram_style = "block" *)
    reg [DATA_WIDTH-1:0] mem [(2**ADDR_WIDTH)-1:0];

    always @(posedge clk) begin
        if (we_a)
            mem[addr_a] <= din_a;
        dout_b <= mem[addr_b];
    end

endmodule
