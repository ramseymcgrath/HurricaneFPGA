///////////////////////////////////////////////////////////////////////////////
// File: top.v
// Description: Top-level module for Cynthion USB Transparent Proxy
//
// This module is the top-level integration for the Cynthion USB sniffer,
// connecting all components and providing the main interfaces.
//
// Target: Lattice ECP5 on Cynthion device
///////////////////////////////////////////////////////////////////////////////

module top #(
    parameter DEBUG_OFF = 1'b0,
    parameter DEBUG_ON = 1'b1   // Add consistent parameter definition
)(
    // Clock and Reset
    input  wire        clk_60mhz,       // 60MHz input clock
    input  wire        reset_n,         // Active low reset
    
    // USB PHY 0 - CONTROL (Internal MCU Access)
    inout              usb0_dp,         // USB D+ (bidirectional)
    inout              usb0_dn,         // USB D- (bidirectional)
    output wire        usb0_pullup,     // USB pullup control
    
    // USB PHY 1 - TARGET A/C (Shared PHY)
    inout              usb1_dp,         // USB D+ (bidirectional)
    inout              usb1_dn,         // USB D- (bidirectional)
    output wire        usb1_pullup,     // USB pullup control
    
    // USB PHY 2 - TARGET B (Dedicated)
    inout              usb2_dp,         // USB D+ (bidirectional)
    inout              usb2_dn,         // USB D- (bidirectional)
    output wire        usb2_pullup,     // USB pullup control
    
    // Status LEDs
    output wire [7:0]  led,             // Status LEDs
    
    // Debug Interface
    output wire [3:0]  debug,           // Debug signals
    output wire        pid_ack          // PID Acknowledge
);

    // Clock generation
    wire        clk;                // System clock (60 MHz)
    wire        clk_120mhz;         // 120MHz clock for fast path
    wire        clk_240mhz;         // 240MHz clock for PHY
    wire        pll_locked;         // PLL lock indicator
    wire        rst_n;              // Global reset (active low)
    wire        pid_ack_120;        // PID ACK from monitor (120MHz domain)
    
    // Output enable signals for USB PHYs
    wire usb0_dp_oe;        // USB0 D+ output enable
    wire usb0_dn_oe;        // USB0 D- output enable
    wire usb1_dp_oe;        // USB1 D+ output enable
    wire usb1_dn_oe;        // USB1 D- output enable
    wire usb2_dp_oe;        // USB2 D+ output enable
    wire usb2_dn_oe;        // USB2 D- output enable
    
    // USB PHY signals for the three interfaces
    // PHY 0 - Control
    wire [1:0]  phy0_line_state;    // USB0 line state
    wire [7:0]  phy0_rx_data;       // USB0 received data
    wire        phy0_rx_valid;      // USB0 data valid
    wire        phy0_rx_active;     // USB0 receiving
    wire        phy0_rx_error;      // USB0 error
    wire [7:0]  phy0_tx_data;       // USB0 transmit data
    wire        phy0_tx_valid;      // USB0 transmit valid
    wire        phy0_tx_ready;      // USB0 ready for transmit
    wire [1:0]  phy0_tx_op_mode;    // USB0 operation mode
    
    // PHY 1 - Target A/C
    wire [1:0]  phy1_line_state;    // USB1 line state
    wire [7:0]  phy1_rx_data;       // USB1 received data
    wire        phy1_rx_valid;      // USB1 data valid
    wire        phy1_rx_active;     // USB1 receiving
    wire        phy1_rx_error;      // USB1 error
    wire [7:0]  phy1_tx_data;       // USB1 transmit data
    wire        phy1_tx_valid;      // USB1 transmit valid
    wire        phy1_tx_ready;      // USB1 ready for transmit
    wire [1:0]  phy1_tx_op_mode;    // USB1 operation mode
    
    // PHY 2 - Target B
    wire [1:0]  phy2_line_state;    // USB2 line state
    wire [7:0]  phy2_rx_data;       // USB2 received data
    wire        phy2_rx_valid;      // USB2 data valid
    wire        phy2_rx_active;     // USB2 receiving
    wire        phy2_rx_error;      // USB2 error
    wire [7:0]  phy2_tx_data;       // USB2 transmit data
    wire        phy2_tx_valid;      // USB2 transmit valid
    wire        phy2_tx_ready;      // USB2 ready for transmit
    wire [1:0]  phy2_tx_op_mode;    // USB2 operation mode
    
    // PHY configuration
    wire [1:0]  phy0_speed_ctrl;    // USB0 speed select
    wire [1:0]  phy1_speed_ctrl;    // USB1 speed select
    wire [1:0]  phy2_speed_ctrl;    // USB2 speed select
    wire        phy0_reset;         // USB0 PHY reset
    wire        phy1_reset;         // USB1 PHY reset
    wire        phy2_reset;         // USB2 PHY reset
    
    // Protocol handler signals
    // Host side protocol
    wire [7:0]  host_decoded_data;  // Decoded data from host
    wire        host_decoded_valid; // Host data valid
    wire        host_decoded_sop;   // Start of host packet
    wire        host_decoded_eop;   // End of host packet
    wire [3:0]  host_pid;           // Host packet ID
    wire [6:0]  host_dev_addr;      // Device address from host
    wire [3:0]  host_endp;          // Endpoint from host
    wire        host_crc_valid;     // Host CRC valid
    
    // Device side protocol
    wire [7:0]  device_decoded_data;// Decoded data from device
    wire        device_decoded_valid;// Device data valid
    wire        device_decoded_sop; // Start of device packet
    wire        device_decoded_eop; // End of device packet
    wire [3:0]  device_pid;         // Device packet ID
    wire        device_crc_valid;   // Device CRC valid
    
    // Control protocol signals
    wire [7:0]  host_tx_data;       // Data to transmit to host
    wire        host_tx_valid;      // Data valid for host
    wire        host_tx_sop;        // Start of packet to host
    wire        host_tx_eop;        // End of packet to host
    wire [3:0]  host_tx_pid;        // PID to send to host
    
    wire [7:0]  device_tx_data;     // Data to transmit to device
    wire        device_tx_valid;    // Data valid for device
    wire        device_tx_sop;      // Start of packet to device
    wire        device_tx_eop;      // End of packet to device
    wire [3:0]  device_tx_pid;      // PID to send to device
    
    // Packet proxy signals
    wire [7:0]  packet_data;        // Packet data
    wire        packet_valid;       // Packet valid
    wire        packet_sop;         // Packet start
    wire        packet_eop;         // Packet end
    wire [3:0]  packet_pid;         // Packet ID
    wire        is_token_packet;    // Is token packet
    wire        is_data_packet;     // Is data packet
    
    // Buffer manager signals
    wire [7:0]  buffer_data;        // Data for buffer
    wire        buffer_valid;       // Buffer data valid
    wire [63:0] buffer_timestamp;   // Timestamp for buffer
    wire [7:0]  buffer_flags;       // Buffer flags
    wire        buffer_ready;       // Buffer ready
    wire [7:0]  read_data;          // Read data from buffer
    wire        read_valid;         // Read valid
    wire        read_req;           // Read request
    
    // Timestamp signals
    wire [63:0] timestamp;          // Current timestamp
    wire [31:0] timestamp_ms;       // Millisecond timestamp
    wire [15:0] sof_frame_num;      // SOF frame number
    wire        timestamp_valid;    // Timestamp valid
    
    // Control registers
    reg         proxy_enable;       // Enable proxy
    reg  [15:0] packet_filter_mask; // Packet filter
    reg         packet_filter_en;   // Enable filtering
    reg         modify_enable;      // Enable modification
    reg  [7:0]  modify_flags;       // Modification flags
    reg  [3:0]  resolution_ctrl;    // Timestamp resolution
    
    // Status signals
    wire        buffer_overflow;    // Buffer overflow
    wire        buffer_underflow;   // Buffer underflow
    wire [15:0] buffer_used;        // Buffer usage
    wire [31:0] packet_count;       // Packet count
    wire [15:0] error_count;        // Error counter
    
    // Control bus
    reg  [7:0]  control_reg_addr;   // Control register address
    reg  [7:0]  control_reg_data;   // Control register data
    reg         control_reg_write;  // Control register write
    
    // Debug interface signals
    wire [7:0]  debug_cmd;          // Debug command input
    wire        debug_cmd_valid;    // Debug command valid
    wire [7:0]  debug_resp;         // Debug response output
    wire        debug_resp_valid;   // Debug response valid
    wire [7:0]  debug_leds;         // Debug LED outputs
    wire [7:0]  debug_probe;        // Debug probe outputs
    wire        force_reset;        // Force system reset
    
    // PHY monitoring
    wire        event_valid;        // PHY event valid
    wire [7:0]  event_type;         // PHY event type
    wire [63:0] event_timestamp;    // PHY event timestamp
    
    // Connection status
    wire        host_conn_detect;   // Host connection detected
    wire [1:0]  host_conn_speed;    // Host connection speed
    wire        device_conn_detect; // Device connection detected
    wire [1:0]  device_conn_speed;  // Device connection speed
    
    // Reset logic
    reg [3:0]   reset_sync;         // Reset synchronizer
    wire        system_rst_n;       // System reset with debug force
    
    // PLL for clock generation
    pll_60_to_240 pll_inst (
        .clkin(clk_60mhz),
        .clkout0(clk),          // 60MHz
        .clkout1(clk_120mhz),   // 120MHz
        .clkout2(clk_240mhz),   // 240MHz
        .locked(pll_locked)
    );
    
    // Reset synchronization
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            reset_sync <= 4'b0000;
        end else begin
            reset_sync <= {reset_sync[2:0], 1'b1};
        end
    end
    
    // Declaration of pid_ack_sync signals
    reg pid_ack_sync1, pid_ack_sync2;
    
    // Global reset signal
    assign system_rst_n = reset_sync[3] & pll_locked;
    assign rst_n = system_rst_n & ~force_reset;  // Add debug forced reset
    assign pid_ack = pid_ack_sync2;  // Connect synchronized PID_ACK to output
    
    // USB PHY 0 - CONTROL (Internal MCU Access)
    usb_phy_wrapper phy0 (
        .clk(clk),
        .clk_480mhz(clk_240mhz), // Using 240MHz as the nearest available
        .rst_n(rst_n),
        
        // PHY pins
        .usb_dp_i(usb0_dp),
        .usb_dn_i(usb0_dn),
        .usb_dp_o(usb0_dp),
        .usb_dn_o(usb0_dn),
        .usb_dp_oe(usb0_dp_oe),
        .usb_dn_oe(usb0_dn_oe),
        .usb_pullup_en(usb0_pullup),
        
        // UTMI interface
        .utmi_line_state(phy0_line_state),
        .utmi_rx_data(phy0_rx_data),
        .utmi_rx_valid(phy0_rx_valid),
        .utmi_rx_active(phy0_rx_active),
        .utmi_rx_error(phy0_rx_error),
        .utmi_tx_data(phy0_tx_data),
        .utmi_tx_valid(phy0_tx_valid),
        .utmi_tx_ready(phy0_tx_ready),
        .utmi_tx_op_mode(phy0_tx_op_mode),
        .utmi_xcvr_select(2'b01), // Default to full-speed
        .utmi_termselect(1'b1),
        .utmi_dppulldown(1'b0),
        .utmi_dmpulldown(1'b0),
        
        // PHY monitoring
        .phy_line_state(),
        .phy_rx_carrier(),
        .phy_rx_clock(),
        
        // Control
        .usb_speed_ctrl(phy0_speed_ctrl),
        .phy_reset(phy0_reset),
        .phy_status()
    );
    
    // USB PHY 1 - TARGET A/C (Shared PHY)
    usb_phy_wrapper phy1 (
        .clk(clk),
        .clk_480mhz(clk_240mhz),
        .rst_n(rst_n),
        
        // PHY pins
        .usb_dp_i(usb1_dp),
        .usb_dn_i(usb1_dn),
        .usb_dp_o(usb1_dp),
        .usb_dn_o(usb1_dn),
        .usb_dp_oe(usb1_dp_oe),
        .usb_dn_oe(usb1_dn_oe),
        .usb_pullup_en(usb1_pullup),
        
        // UTMI interface
        .utmi_line_state(phy1_line_state),
        .utmi_rx_data(phy1_rx_data),
        .utmi_rx_valid(phy1_rx_valid),
        .utmi_rx_active(phy1_rx_active),
        .utmi_rx_error(phy1_rx_error),
        .utmi_tx_data(phy1_tx_data),
        .utmi_tx_valid(phy1_tx_valid),
        .utmi_tx_ready(phy1_tx_ready),
        .utmi_tx_op_mode(phy1_tx_op_mode),
        .utmi_xcvr_select(2'b01),
        .utmi_termselect(1'b1),
        .utmi_dppulldown(1'b0),
        .utmi_dmpulldown(1'b0),
        
        // PHY monitoring
        .phy_line_state(),
        .phy_rx_carrier(),
        .phy_rx_clock(),
        
        // Control
        .usb_speed_ctrl(phy1_speed_ctrl),
        .phy_reset(phy1_reset),
        .phy_status()
    );
    
    // USB PHY 2 - TARGET B (Dedicated)
    usb_phy_wrapper phy2 (
        .clk(clk),
        .clk_480mhz(clk_240mhz),
        .rst_n(rst_n),
        
        // PHY pins
        .usb_dp_i(usb2_dp),
        .usb_dn_i(usb2_dn),
        .usb_dp_o(usb2_dp),
        .usb_dn_o(usb2_dn),
        .usb_dp_oe(usb2_dp_oe),
        .usb_dn_oe(usb2_dn_oe),
        .usb_pullup_en(usb2_pullup),
        
        // UTMI interface
        .utmi_line_state(phy2_line_state),
        .utmi_rx_data(phy2_rx_data),
        .utmi_rx_valid(phy2_rx_valid),
        .utmi_rx_active(phy2_rx_active),
        .utmi_rx_error(phy2_rx_error),
        .utmi_tx_data(phy2_tx_data),
        .utmi_tx_valid(phy2_tx_valid),
        .utmi_tx_ready(phy2_tx_ready),
        .utmi_tx_op_mode(phy2_tx_op_mode),
        .utmi_xcvr_select(2'b01),
        .utmi_termselect(1'b1),
        .utmi_dppulldown(1'b0),
        .utmi_dmpulldown(1'b0),
        
        // PHY monitoring
        .phy_line_state(),
        .phy_rx_carrier(),
        .phy_rx_clock(),
        
        // Control
        .usb_speed_ctrl(phy2_speed_ctrl),
        .phy_reset(phy2_reset),
        .phy_status()
    );
    
    // USB protocol handler for device side (PHY1)
    usb_protocol_handler device_protocol (
        .clk(clk),
        .rst_n(rst_n),
        
        // UTMI Interface
        .utmi_rx_data(phy1_rx_data),
        .utmi_rx_valid(phy1_rx_valid),
        .utmi_rx_active(phy1_rx_active),
        .utmi_rx_error(phy1_rx_error),
        .utmi_line_state(phy1_line_state),
        .utmi_tx_data(phy1_tx_data),
        .utmi_tx_valid(phy1_tx_valid),
        .utmi_tx_ready(phy1_tx_ready),
        .utmi_tx_op_mode(phy1_tx_op_mode),
        .utmi_xcvr_select(phy1_speed_ctrl),
        .utmi_termselect(),
        .utmi_dppulldown(),
        .utmi_dmpulldown(),
        
        // Protocol Interface
        .packet_data(device_decoded_data),
        .packet_valid(device_decoded_valid),
        .packet_sop(device_decoded_sop),
        .packet_eop(device_decoded_eop),
        .pid(device_pid),
        .dev_addr(),
        .endp(),
        .crc_valid(device_crc_valid),
        
        // Transmit Interface
        .tx_packet_data(device_tx_data),
        .tx_packet_valid(device_tx_valid),
        .tx_packet_sop(device_tx_sop),
        .tx_packet_eop(device_tx_eop),
        .tx_packet_ready(),
        .tx_pid(device_tx_pid),
        
        // Configuration
        .device_address(7'h01),  // Default device address
        .usb_speed(phy1_speed_ctrl),
        .conn_detect(device_conn_detect),
        .conn_speed(device_conn_speed),
        .reset_detect(),
        .suspend_detect(),
        .resume_detect()
    );
    
    // USB protocol handler for host side (PHY2)
    usb_protocol_handler host_protocol (
        .clk(clk),
        .rst_n(rst_n),
        
        // UTMI Interface
        .utmi_rx_data(phy2_rx_data),
        .utmi_rx_valid(phy2_rx_valid),
        .utmi_rx_active(phy2_rx_active),
        .utmi_rx_error(phy2_rx_error),
        .utmi_line_state(phy2_line_state),
        .utmi_tx_data(phy2_tx_data),
        .utmi_tx_valid(phy2_tx_valid),
        .utmi_tx_ready(phy2_tx_ready),
        .utmi_tx_op_mode(phy2_tx_op_mode),
        .utmi_xcvr_select(phy2_speed_ctrl),
        .utmi_termselect(),
        .utmi_dppulldown(),
        .utmi_dmpulldown(),
        
        // Protocol Interface
        .packet_data(host_decoded_data),
        .packet_valid(host_decoded_valid),
        .packet_sop(host_decoded_sop),
        .packet_eop(host_decoded_eop),
        .pid(host_pid),
        .dev_addr(host_dev_addr),
        .endp(host_endp),
        .crc_valid(host_crc_valid),
        
        // Transmit Interface
        .tx_packet_data(host_tx_data),
        .tx_packet_valid(host_tx_valid),
        .tx_packet_sop(host_tx_sop),
        .tx_packet_eop(host_tx_eop),
        .tx_packet_ready(),
        .tx_pid(host_tx_pid),
        
        // Configuration
        .device_address(7'h00),  // Host doesn't need a device address
        .usb_speed(phy2_speed_ctrl),
        .conn_detect(host_conn_detect),
        .conn_speed(host_conn_speed),
        .reset_detect(),
        .suspend_detect(),
        .resume_detect()
    );
    
    // Clock domain crossing synchronizer for PID_ACK
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pid_ack_sync1 <= 1'b0;
            pid_ack_sync2 <= 1'b0;
        end else begin
            pid_ack_sync1 <= pid_ack_120;
            pid_ack_sync2 <= pid_ack_sync1;
        end
    end

    // Host TX valid mux for debug mode - consolidated parameters
    
    // Debug mode signals with CDC
    reg [1:0] debug_mode;                // Debug mode control
    wire host_tx_valid_mux;              // Host TX valid mux (declared but was missing)
    wire device_tx_valid_gated;          // Device TX valid gated (missing declaration)
    
    // CDC parameters
    localparam SYNC_STAGES = 2;
    
    // Debug mode configuration register
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debug_mode <= DEBUG_OFF;
        end
        else if (control_reg_write && control_reg_addr == 8'h40) begin
            debug_mode <= control_reg_data[1:0];
        end
    end

    // Reset synchronization for different clock domains
    
    // Reset synchronization registers for 120MHz domain
    reg [SYNC_STAGES-1:0] reset_sync_120mhz;
    always @(posedge clk_120mhz or negedge rst_n) begin
        if (!rst_n) begin
            reset_sync_120mhz <= {SYNC_STAGES{1'b0}};
        end else begin
            reset_sync_120mhz <= {reset_sync_120mhz[SYNC_STAGES-2:0], 1'b1};
        end
    end
    wire rst_120mhz_n = reset_sync_120mhz[SYNC_STAGES-1];

    // Reset synchronization registers for 60MHz domain
    reg [SYNC_STAGES-1:0] reset_sync_60mhz;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            reset_sync_60mhz <= {SYNC_STAGES{1'b0}};
        end else begin
            reset_sync_60mhz <= {reset_sync_60mhz[SYNC_STAGES-2:0], 1'b1};
        end
    end
    wire rst_60mhz_n = reset_sync_60mhz[SYNC_STAGES-1];
    
    // Active-high reset signals for convenience
    wire reset_120mhz = ~rst_120mhz_n;
    wire reset_60mhz = ~rst_60mhz_n;
    
    // Debug mode CDC with proper reset synchronization - 60MHz to 120MHz domain
    reg [1:0] debug_mode_cdc_sync1, debug_mode_cdc_sync2;
    always @(posedge clk_120mhz) begin
        if (!rst_120mhz_n) begin
            debug_mode_cdc_sync1 <= DEBUG_OFF; // Initialize to default
            debug_mode_cdc_sync2 <= DEBUG_OFF;
        end
        else begin
            debug_mode_cdc_sync1 <= debug_mode;
            debug_mode_cdc_sync2 <= debug_mode_cdc_sync1;
        end
    end

    // Registered output for debug mode in 120MHz domain
    wire [1:0] debug_mode_120mhz = debug_mode_cdc_sync2;

    // 60MHz domain synchronization - fixed array syntax
    reg [1:0] debug_mode_cdc_60mhz_0;
    reg [1:0] debug_mode_cdc_60mhz_1;
    always @(posedge clk) begin
        if (!rst_60mhz_n) begin
            debug_mode_cdc_60mhz_0 <= DEBUG_OFF;
            debug_mode_cdc_60mhz_1 <= DEBUG_OFF;
        end else begin
            debug_mode_cdc_60mhz_0 <= debug_mode;
            debug_mode_cdc_60mhz_1 <= debug_mode_cdc_60mhz_0;
        end
    end
    wire [1:0] debug_mode_sync = debug_mode_cdc_60mhz_1;  // Pass through the full 2-bit value

    // Cross-domain signals for TX valid signals
    // Signal declarations for signals crossing clock domains
    reg host_tx_valid_60mhz_sync1, host_tx_valid_60mhz_sync2;
    reg phy_tx_valid_60mhz_sync1, phy_tx_valid_60mhz_sync2;
    wire host_tx_valid_60mhz;
    wire phy_tx_valid_60mhz;
    
    // Synchronize host_tx_valid to 60MHz domain
    always @(posedge clk) begin
        if (!rst_60mhz_n) begin
            host_tx_valid_60mhz_sync1 <= 1'b0;
            host_tx_valid_60mhz_sync2 <= 1'b0;
        end else begin
            host_tx_valid_60mhz_sync1 <= host_tx_valid;
            host_tx_valid_60mhz_sync2 <= host_tx_valid_60mhz_sync1;
        end
    end
    assign host_tx_valid_60mhz = host_tx_valid_60mhz_sync2;
    
    // Synchronize phy_tx_valid to 60MHz domain
    always @(posedge clk) begin
        if (!rst_60mhz_n) begin
            phy_tx_valid_60mhz_sync1 <= 1'b0;
            phy_tx_valid_60mhz_sync2 <= 1'b0;
        end else begin
            phy_tx_valid_60mhz_sync1 <= device_tx_valid; // Assuming this is the intended signal
            phy_tx_valid_60mhz_sync2 <= phy_tx_valid_60mhz_sync1;
        end
    end
    assign phy_tx_valid_60mhz = phy_tx_valid_60mhz_sync2;
    
    // Verify synchronization latency and stability
    `ifdef FORMAL
    assert property (@(posedge clk_120mhz)
        $stable(debug_mode) |-> ##2 (debug_mode_sync == $past(debug_mode,2)));
    `endif

    // Device-to-host data mux based on debug_mode
    // Combinational logic for mux
    reg host_tx_valid_mux_reg;
    always @(*) begin
        host_tx_valid_mux_reg = 1'b0;  // Default assignment
        
        if (debug_mode_sync) begin
            host_tx_valid_mux_reg = phy_tx_valid_60mhz;
        end
        else begin
            host_tx_valid_mux_reg = host_tx_valid_60mhz;
        end
    end
    
    // Connect register to wire output
    assign host_tx_valid_mux = host_tx_valid_mux_reg;
    
    // Using PLL instance from line 201, removing redundant clk_wiz_0 instance
    
    // Create synchronized reset from PLL locked signal
    reg pll_locked_sync;
    always @(posedge clk_120mhz or negedge rst_n) begin
        if (!rst_n)
            pll_locked_sync <= 1'b0;
        else
            pll_locked_sync <= pll_locked;
    end

    // Proper connection for device_tx_valid_gated
    assign device_tx_valid_gated = device_tx_valid & debug_mode_sync;
    
    // USB Monitor instantiation with properly structured port list
    usb_monitor monitor (
        // Clock and Reset
        .clk(clk_60mhz),            // System clock (60MHz)
        .clk_120mhz(clk_120mhz),    // 120MHz clock for faster processing
        .rst_n(rst_120mhz_n),       // Active low reset
        .overflow_detected(),       // Buffer overflow indicator
        .pid_ack_120(pid_ack_120),  // PID ACK detection signal (120MHz domain)
        
        // Host Side USB Interface
        .host_rx_data(host_decoded_data),
        .host_rx_valid(host_decoded_valid),
        .host_rx_sop(host_decoded_sop),
        .host_rx_eop(host_decoded_eop),
        .host_rx_pid(host_pid),
        .host_rx_dev_addr(host_dev_addr),
        .host_rx_endp(host_endp),
        .host_rx_crc_valid(host_crc_valid),
        .host_tx_data(host_tx_data),
        .host_tx_valid(host_tx_valid),
        .host_tx_valid_mux(host_tx_valid_mux),
        .host_tx_sop(host_tx_sop),
        .host_tx_eop(host_tx_eop),
        .host_tx_pid(host_tx_pid),
        
        // Device Side Interface
        .device_rx_data(device_decoded_data),
        .device_rx_valid(device_decoded_valid),
        .device_rx_sop(device_decoded_sop),
        .device_rx_eop(device_decoded_eop),
        .device_rx_pid(device_pid),
        .device_rx_endp(4'b0000),   // Default value
        .device_rx_crc_valid(device_crc_valid),
        .device_tx_data(device_tx_data),
        // Separate signals for clarity
        .device_tx_valid(device_tx_valid_gated),
        .device_tx_sop(device_tx_sop),
        .device_tx_eop(device_tx_eop),
        .device_tx_pid(device_tx_pid),
        
        // Buffer Manager Interface
        .buffer_data(),
        .buffer_valid(),
        .buffer_timestamp(),
        .buffer_flags(),
        .buffer_ready(1'b1),
        
        // Timestamp Interface
        .timestamp(64'h0),
        
        // PHY State Monitor Interface
        .host_line_state(2'b00),
        .device_line_state(2'b00),
        .event_valid(1'b0),
        .event_type(8'h00),
        
        // Control Interface
        .control_reg_addr(control_reg_addr),
        .control_reg_data(control_reg_data),
        .control_reg_write(control_reg_write),
        .status_register(),
        .status_read_data(),
        
        // Configuration Registers
        .proxy_enable(1'b1),
        .packet_filter_en(1'b0),
        .packet_filter_mask(16'h0000),
        .modify_enable(1'b0),
        .addr_translate_en(8'h00),
        .addr_translate_from(7'h00),
        .addr_translate_to(7'h00)
    );
        
    // Additional port connections removed (they were duplicates)
    
    // Packet forwarding with inspection
    packet_proxy proxy (
        .clk(clk),
        .clk_120mhz(clk_120mhz),
        .rst_n(rst_n),
        
        // Host Controller Interface
        .host_rx_data(host_decoded_data),
        .host_rx_valid(host_decoded_valid),
        .host_rx_sop(host_decoded_sop),
        .host_rx_eop(host_decoded_eop),
        .host_tx_data(),
        .host_tx_valid(),
        .host_tx_sop(),
        .host_tx_eop(),
        
        // Device Controller Interface
        .device_rx_data(device_decoded_data),
        .device_rx_valid(device_decoded_valid),
        .device_rx_sop(device_decoded_sop),
        .device_rx_eop(device_decoded_eop),
        .device_tx_data(),
        .device_tx_valid(),
        .device_tx_sop(),
        .device_tx_eop(),
        
        // Buffer Manager Interface
        .buffer_data(packet_data),
        .buffer_valid(packet_valid),
        .buffer_timestamp(timestamp),
        .buffer_flags(),
        .buffer_ready(buffer_ready),
        
        // Timestamp Generator Interface
        .timestamp(timestamp),
        
        // Protocol Identification
        .packet_pid(packet_pid),
        .is_token_packet(is_token_packet),
        .is_data_packet(is_data_packet),
        .device_addr(),
        .endpoint_num(),
        
        // Control Interface
        .control_reg_addr(control_reg_addr),
        .control_reg_data(control_reg_data),
        .control_reg_write(control_reg_write),
        
        // Configuration
        .enable_proxy(proxy_enable),
        .enable_logging(1'b1),
        .enable_filtering(packet_filter_en),
        .packet_filter(packet_filter_mask),
        .enable_modify(modify_enable),
        .modify_flags(modify_flags)
    );
    
    // Ring buffer implementation
    buffer_manager buffer (
        .clk(clk),
        .rst_n(rst_n),
        
        // Write Interface
        .write_data(buffer_data),
        .write_valid(buffer_valid),
        .write_timestamp(buffer_timestamp),
        .write_flags(buffer_flags),
        .write_ready(buffer_ready),
        
        // Read Interface
        .read_data(read_data),
        .read_valid(read_valid),
        .read_req(read_req),
        .read_timestamp(),
        .read_flags(),
        .read_packet_start(),
        .read_packet_end(),
        
        // Control Interface
        .buffer_clear(1'b0),
        .high_watermark(16'h7000),  // 28KB high watermark
        .low_watermark(16'h1000),   // 4KB low watermark
        
        // Status Interface
        .buffer_used(buffer_used),
        .buffer_free(),
        .buffer_empty(),
        .buffer_full(),
        .buffer_overflow(buffer_overflow),
        .buffer_underflow(buffer_underflow),
        .packet_count(packet_count),
        
        // Configuration
        .enable_overflow_protection(1'b1),
        .buffer_mode(2'b01)  // Separate buffers for each direction
    );
    
    // Timestamp generator
    timestamp_generator timestamper (
        .clk(clk),
        .clk_high(clk_240mhz),
        .rst_n(rst_n),
        
        // Timestamp Outputs
        .timestamp(timestamp),
        .timestamp_ms(timestamp_ms),
        .sof_frame_num(sof_frame_num),
        
        // Synchronization
        .sync_enable(1'b0),
        .sync_pulse(1'b0),
        .sync_value(64'h0),
        
        // USB Frame Sync
        .sof_detected(host_pid == 4'b0101),  // SOF PID
        .sof_frame_num_in(11'h000),  // From SOF packet
        
        // Configuration
        .resolution_ctrl(resolution_ctrl),  // Configurable resolution
        .counter_enable(1'b1),
        .reset_counter(1'b0),
        
        // Status
        .timestamp_valid(timestamp_valid),
        .timestamp_rate()
    );
    
    // Initial configuration (would normally come from control interface)
    initial begin
        proxy_enable = 1'b1;
        packet_filter_mask = 16'h0000;  // No filtering initially
        packet_filter_en = 1'b0;
        modify_enable = 1'b0;
        modify_flags = 8'h00;
        resolution_ctrl = 4'h0;  // Full 60MHz resolution
    end
    
    // Debug interface module
    debug_interface debug_if (
        .clk(clk),
        .rst_n(system_rst_n),
        
        // Debug Control Interface
        .debug_cmd(debug_cmd),
        .debug_cmd_valid(debug_cmd_valid),
        .debug_resp(debug_resp),
        .debug_resp_valid(debug_resp_valid),
        
        // Status Inputs
        .proxy_active(proxy_enable),
        .host_connected(host_conn_detect),
        .device_connected(device_conn_detect),
        .host_speed(host_conn_speed),
        .device_speed(device_conn_speed),
        .buffer_overflow(buffer_overflow),
        .buffer_used(buffer_used),
        .packet_count(packet_count),
        .error_count(error_count),
        
        // Monitor Inputs
        .host_line_state(phy2_line_state),
        .device_line_state(phy1_line_state),
        .timestamp(timestamp),
        
        // Debug Outputs
        .debug_leds(debug_leds),
        .debug_probe(debug_probe),
        
        // Configuration Control
        .force_reset(force_reset),
        .debug_mode(debug_mode_sync),
        .trigger_config(),
        .loopback_enable()
    );
    
    // Status LEDs - use debug LED values when available, otherwise defaults
    assign led = debug_leds;
    
    // Debug outputs
    assign debug = debug_probe[3:0];  // Use the first 4 bits of debug probe
    
    // Connect debug interface to USB control path (simplified for clarity)
    // In a real implementation, this would involve proper USB endpoints
    assign debug_cmd = phy0_rx_data;
    assign debug_cmd_valid = phy0_rx_valid;
    assign phy0_tx_data = debug_resp;
    assign phy0_tx_valid = debug_resp_valid;
    
    // Error counter
    reg [15:0] internal_error_count;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            internal_error_count <= 16'd0;
        else if (host_decoded_valid && !host_crc_valid && host_decoded_eop)
            internal_error_count <= internal_error_count + 1'b1;
        else if (device_decoded_valid && !device_crc_valid && device_decoded_eop)
            internal_error_count <= internal_error_count + 1'b1;
    end
    assign error_count = internal_error_count;
    
endmodule

// PLL module definition - moved outside the top module to fix syntax error
module pll_60_to_240 (
    input  wire clkin,     // 60 MHz input clock
    output wire clkout0,   // 60 MHz output clock
    output wire clkout1,   // 120 MHz output clock
    output wire clkout2,   // 240 MHz output clock
    output wire locked     // PLL locked indicator
);
    // PLL would be implemented with platform-specific primitives
    // For ECP5, this would use the EHXPLLL primitive
    
    // Placeholder for simulation
    assign clkout0 = clkin;
    assign clkout1 = clkin;  // Would normally be 120 MHz
    assign clkout2 = clkin;  // Would normally be 240 MHz
    assign locked = 1'b1;    // Always locked for this placeholder
endmodule

// Clock Wizard IP core stub for simulation
// In a real implementation, this would be a platform-specific IP core
module clk_wiz_0 (
    input  wire clk_in1,   // Input clock
    output wire clk_out1,  // Output clock
    input  wire reset,     // Reset input
    output wire locked     // PLL lock indicator
);
    reg locked_reg = 0;
    
    // Simple pass-through for simulation
    assign clk_out1 = clk_in1;
    assign locked = ~reset && locked_reg;
    
    // Lock detection
    always @(posedge clk_in1 or posedge reset) begin
        if (reset)
            locked_reg <= 0;
        else
            locked_reg <= 1;
    end
endmodule