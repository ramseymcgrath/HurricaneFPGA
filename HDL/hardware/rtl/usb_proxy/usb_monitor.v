`timescale 1ns / 1ps

module usb_monitor (
    input  wire        clk,
    input  wire        clk_120mhz,
    input  wire        rst_n,
    output reg         overflow_detected,
    output reg         pid_ack_120,

    // Host-side USB
    input  wire [7:0]  host_rx_data,
    input  wire        host_rx_valid,
    input  wire        host_rx_sop,
    input  wire        host_rx_eop,
    input  wire [3:0]  host_rx_pid,
    input  wire [6:0]  host_rx_dev_addr,
    input  wire [3:0]  host_rx_endp,
    input  wire        host_rx_crc_valid,
    output wire [7:0]  host_tx_data,
    output wire        host_tx_valid,
    output wire        host_tx_sop,
    output wire        host_tx_eop,
    output wire [3:0]  host_tx_pid,

    // Device-side USB
    input  wire [7:0]  device_rx_data,
    input  wire        device_rx_valid,
    input  wire        device_rx_sop,
    input  wire        device_rx_eop,
    input  wire [3:0]  device_rx_pid,
    input  wire [3:0]  device_rx_endp,
    input  wire        device_rx_crc_valid,
    output wire [7:0]  device_tx_data,
    output wire        device_tx_valid,
    output wire        device_tx_sop,
    output wire        device_tx_eop,
    output wire [3:0]  device_tx_pid,

    // Buffer manager
    output reg  [7:0]  buffer_data,
    output reg         buffer_valid,
    output reg  [63:0] buffer_timestamp,
    output reg  [7:0]  buffer_flags,
    input  wire        buffer_ready,

    // Time + debug
    input  wire [63:0] timestamp,
    input  wire [1:0]  host_line_state,
    input  wire [1:0]  device_line_state,
    input  wire        event_valid,
    input  wire [7:0]  event_type,

    // Control
    input  wire [7:0]  control_reg_addr,
    input  wire [7:0]  control_reg_data,
    input  wire        control_reg_write,
    output reg  [7:0]  status_register,
    output reg  [7:0]  status_read_data,

    input  wire        proxy_enable,
    input  wire        packet_filter_en,
    input  wire [15:0] packet_filter_mask,
    input  wire        modify_enable,
    input  wire [7:0]  addr_translate_en,
    input  wire [6:0]  addr_translate_from,
    input  wire [6:0]  addr_translate_to
);

    // ------------------------------------------------------------------------
    // Local parameters and regs
    // ------------------------------------------------------------------------

    localparam STAT_CONNECTED   = 0;
    localparam STAT_ERROR       = 1;
    localparam STAT_PROXY_EN    = 2;
    localparam STAT_FILTER_EN   = 3;
    localparam STAT_MODIFY_EN   = 4;
    localparam STAT_OVERFLOW    = 5;
    localparam STAT_SPEED_HIGH  = 6;
    localparam STAT_SPEED_FULL  = 7;

    localparam ST_IDLE            = 4'd0;

    reg [3:0] state;
    reg [15:0] error_count;
    reg [31:0] host_packets, device_packets;
    reg [7:0]  byte_counter;
    reg [1:0]  device_speed;
    reg        device_connected;
    reg [3:0]  latched_host_pid;
    reg [3:0]  latched_device_pid;

    (* ram_style = "distributed" *) reg [7:0] monitor_packet_buffer [0:255];

    // ------------------------------------------------------------------------
    // Clock domain: 60 MHz (for overflow detection)
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        overflow_detected <= (byte_counter == 255) && (state != ST_IDLE);
    end

    // ------------------------------------------------------------------------
    // Main FSM: 120 MHz domain
    // ------------------------------------------------------------------------
    always @(posedge clk_120mhz or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            status_register <= 8'b0;
            error_count <= 0;
            host_packets <= 0;
            device_packets <= 0;
            byte_counter <= 0;
            overflow_detected <= 0;
            device_speed <= 0;
            device_connected <= 0;
        end else begin
            // Status assignments (unified)
            status_register[STAT_CONNECTED]   <= device_connected;
            status_register[STAT_PROXY_EN]    <= proxy_enable;
            status_register[STAT_FILTER_EN]   <= packet_filter_en;
            status_register[STAT_MODIFY_EN]   <= modify_enable;
            {status_register[STAT_SPEED_HIGH], status_register[STAT_SPEED_FULL]} <= device_speed;
            status_register[STAT_OVERFLOW]    <= overflow_detected;
            status_register[STAT_ERROR]       <= 1'b0;

            // Error handling
            if ((host_rx_valid && !host_rx_crc_valid && host_rx_eop) ||
                (device_rx_valid && !device_rx_crc_valid && device_rx_eop)) begin
                error_count <= error_count + 1;
                status_register[STAT_ERROR] <= 1;
            end

            // [Rest of FSM would go here...]
        end
    end

    // ------------------------------------------------------------------------
    // Register readback
    // ------------------------------------------------------------------------
    always @(*) begin
        case (control_reg_addr)
            8'h00: status_read_data = status_register;
            8'h01: status_read_data = error_count[7:0];
            8'h02: status_read_data = error_count[15:8];
            8'h03: status_read_data = host_packets[7:0];
            8'h04: status_read_data = host_packets[15:8];
            8'h05: status_read_data = host_packets[23:16];
            8'h06: status_read_data = host_packets[31:24];
            8'h07: status_read_data = device_packets[7:0];
            8'h08: status_read_data = device_packets[15:8];
            8'h09: status_read_data = device_packets[23:16];
            8'h0A: status_read_data = device_packets[31:24];
            8'h10: status_read_data = {4'h0, latched_host_pid};
            8'h11: status_read_data = {4'h0, latched_device_pid};
            8'h12: status_read_data = byte_counter;
            8'h13: status_read_data = {6'b0, device_speed};
            8'h14: status_read_data = {7'b0, device_connected};
            8'hF0: status_read_data = {addr_translate_en[0], 7'h0};
            8'hF1: status_read_data = addr_translate_from;
            8'hF2: status_read_data = addr_translate_to;
            8'hF3: status_read_data = {proxy_enable, packet_filter_en, modify_enable, 5'b0};
            default: status_read_data = 8'h00;
        endcase
    end

endmodule
