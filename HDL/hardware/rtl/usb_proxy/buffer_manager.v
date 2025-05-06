// Updated buffer_manager.v using bram_dual_port modules for BRAM inference

module buffer_manager (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [7:0]  write_data,
    input  wire        write_valid,
    input  wire [63:0] write_timestamp,
    input  wire [7:0]  write_flags,
    output wire        write_ready,
    output reg  [7:0]  read_data,
    output reg         read_valid,
    input  wire        read_req,
    output reg  [63:0] read_timestamp,
    output reg  [7:0]  read_flags,
    output wire        read_packet_start,
    output wire        read_packet_end,
    input  wire        buffer_clear,
    input  wire [15:0] high_watermark,
    input  wire [15:0] low_watermark,
    output reg  [15:0] buffer_used,
    output reg  [15:0] buffer_free,
    output reg         buffer_empty,
    output reg         buffer_full,
    output reg         buffer_overflow,
    output reg         buffer_underflow,
    output reg  [31:0] packet_count,
    input  wire        enable_overflow_protection,
    input  wire [1:0]  buffer_mode
);

    localparam BUFFER_SIZE = 32768;
    localparam BUFFER_SIZE_PER_DIR = 16384;
    localparam ADDR_WIDTH = 14;
    localparam HEADER_SIZE = 12;
    localparam MAGIC_BYTE = 8'hA5;

    localparam PKT_IDLE     = 2'b00;
    localparam PKT_HEADER   = 2'b01;
    localparam PKT_DATA     = 2'b10;

    // Host BRAM interface
    wire [7:0] mem_host_read_data;
    reg  [7:0] mem_host_write_data;
    reg  [ADDR_WIDTH-1:0] mem_host_write_addr, mem_host_read_addr;
    reg        mem_host_we;

    bram_dual_port #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(8)) host_ram (
        .clk(clk),
        .we_a(mem_host_we),
        .addr_a(mem_host_write_addr),
        .din_a(mem_host_write_data),
        .addr_b(mem_host_read_addr),
        .dout_b(mem_host_read_data)
    );

    // Device BRAM interface
    wire [7:0] mem_dev_read_data;
    reg  [7:0] mem_dev_write_data;
    reg  [ADDR_WIDTH-1:0] mem_dev_write_addr, mem_dev_read_addr;
    reg        mem_dev_we;

    bram_dual_port #(.ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(8)) device_ram (
        .clk(clk),
        .we_a(mem_dev_we),
        .addr_a(mem_dev_write_addr),
        .din_a(mem_dev_write_data),
        .addr_b(mem_dev_read_addr),
        .dout_b(mem_dev_read_data)
    );

    reg [ADDR_WIDTH-1:0] write_ptr_host = 0;
    reg [ADDR_WIDTH-1:0] read_ptr_host = 0;
    reg [ADDR_WIDTH-1:0] write_ptr_dev = 0;
    reg [ADDR_WIDTH-1:0] read_ptr_dev = 0;
    reg [1:0] write_state = PKT_IDLE;
    reg [3:0] header_idx = 0;
    reg [15:0] write_len = 0;
    reg direction = 0;
    reg packet_start = 0;
    reg packet_end = 0;

    assign write_ready = !buffer_full;
    assign read_packet_start = packet_start;
    assign read_packet_end = packet_end;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_state <= PKT_IDLE;
            mem_host_we <= 0;
            mem_dev_we <= 0;
            buffer_overflow <= 0;
        end else begin
            mem_host_we <= 0;
            mem_dev_we <= 0;

            if (buffer_clear) begin
                write_ptr_host <= 0;
                write_ptr_dev <= 0;
                write_state <= PKT_IDLE;
                header_idx <= 0;
                write_len <= 0;
            end else begin
                case (write_state)
                    PKT_IDLE: begin
                        if (write_valid && write_ready) begin
                            write_state <= PKT_HEADER;
                            header_idx <= 0;
                            write_len <= 0;
                            direction <= write_flags[0];
                        end
                    end
                    PKT_HEADER: begin
                        if (header_idx < HEADER_SIZE) begin
                            case (header_idx)
                                0: begin
                                    if (!direction) begin
                                        mem_host_write_addr <= write_ptr_host;
                                        mem_host_write_data <= MAGIC_BYTE;
                                        mem_host_we <= 1;
                                    end else begin
                                        mem_dev_write_addr <= write_ptr_dev;
                                        mem_dev_write_data <= MAGIC_BYTE;
                                        mem_dev_we <= 1;
                                    end
                                end
                                default: begin
                                    if (!direction) begin
                                        mem_host_write_addr <= write_ptr_host;
                                        mem_host_write_data <= 8'hAA;
                                        mem_host_we <= 1;
                                    end else begin
                                        mem_dev_write_addr <= write_ptr_dev;
                                        mem_dev_write_data <= 8'hAA;
                                        mem_dev_we <= 1;
                                    end
                                end
                            endcase

                            if (!direction)
                                write_ptr_host <= write_ptr_host + 1;
                            else
                                write_ptr_dev <= write_ptr_dev + 1;

                            header_idx <= header_idx + 1;
                        end else begin
                            write_state <= PKT_DATA;
                        end
                    end
                    PKT_DATA: begin
                        if (write_valid) begin
                            if (!direction) begin
                                mem_host_write_addr <= write_ptr_host;
                                mem_host_write_data <= write_data;
                                mem_host_we <= 1;
                                write_ptr_host <= write_ptr_host + 1;
                            end else begin
                                mem_dev_write_addr <= write_ptr_dev;
                                mem_dev_write_data <= write_data;
                                mem_dev_we <= 1;
                                write_ptr_dev <= write_ptr_dev + 1;
                            end
                            write_len <= write_len + 1;
                        end else begin
                            write_state <= PKT_IDLE;
                        end
                    end
                endcase
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_valid <= 0;
            read_data <= 0;
            packet_start <= 0;
            packet_end <= 0;
        end else begin
            read_valid <= 0;
            packet_start <= 0;
            packet_end <= 0;

            if (read_req) begin
                if (!direction) begin
                    mem_host_read_addr <= read_ptr_host;
                    read_data <= mem_host_read_data;
                    read_ptr_host <= read_ptr_host + 1;
                end else begin
                    mem_dev_read_addr <= read_ptr_dev;
                    read_data <= mem_dev_read_data;
                    read_ptr_dev <= read_ptr_dev + 1;
                end
                read_valid <= 1;
            end
        end
    end
endmodule
