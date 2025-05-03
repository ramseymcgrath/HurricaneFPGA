///////////////////////////////////////////////////////////////////////////////
// File: usb_monitor.v
// Description: USB Monitor and Proxy Logic
//
// This module implements the main transparent proxy logic, coordinating between
// the host and device USB interfaces and enabling monitoring functionality.
//
// Target: Lattice ECP5 on Cynthion device
///////////////////////////////////////////////////////////////////////////////

module usb_monitor (
    // Clock and Reset
    input  wire        clk,               // System clock (60MHz)
    input  wire        clk_120mhz,        // 120MHz clock for faster processing
    input  wire        rst_n,             // Active low reset

    // Host Side USB Interface
    input  wire [7:0]  host_rx_data,      // Decoded data from host
    input  wire        host_rx_valid,     // Host data valid
    input  wire        host_rx_sop,       // Start of host packet
    input  wire        host_rx_eop,       // End of host packet
    input  wire [3:0]  host_rx_pid,       // USB PID from host
    input  wire [6:0]  host_rx_dev_addr,  // Device address from host packets
    input  wire [3:0]  host_rx_endp,      // Endpoint from host packets
    input  wire        host_rx_crc_valid, // CRC valid for host packets
    output wire [7:0]  host_tx_data,      // Data to transmit to host
    output wire        host_tx_valid,     // Data valid for host transmission
    output wire        host_tx_sop,       // Start of packet to host
    output wire        host_tx_eop,       // End of packet to host
    output wire [3:0]  host_tx_pid,       // PID to send to host
    
    // Device Side USB Interface
    input  wire [7:0]  device_rx_data,    // Decoded data from device
    input  wire        device_rx_valid,   // Device data valid
    input  wire        device_rx_sop,     // Start of device packet
    input  wire        device_rx_eop,     // End of device packet
    input  wire [3:0]  device_rx_pid,     // USB PID from device
    input  wire        device_rx_crc_valid, // CRC valid for device packets
    output wire [7:0]  device_tx_data,    // Data to transmit to device
    output wire        device_tx_valid,   // Data valid for device transmission
    output wire        device_tx_sop,     // Start of packet to device
    output wire        device_tx_eop,     // End of packet to device
    output wire [3:0]  device_tx_pid,     // PID to send to device
    
    // Buffer Manager Interface
    output reg  [7:0]  buffer_data,       // Data to store in buffer
    output reg         buffer_valid,      // Data valid flag
    output reg  [63:0] buffer_timestamp,  // Timestamp for data
    output reg  [7:0]  buffer_flags,      // Flags (direction, packet type, etc.)
    input  wire        buffer_ready,      // Buffer is ready to accept data
    
    // Timestamp Interface
    input  wire [63:0] timestamp,         // Current timestamp
    
    // PHY State Monitor Interface
    input  wire [1:0]  host_line_state,   // USB line state from host
    input  wire [1:0]  device_line_state, // USB line state from device
    input  wire        event_valid,       // PHY event valid
    input  wire [7:0]  event_type,        // PHY event type
    
    // Control Interface
    input  wire [7:0]  control_reg_addr,  // Control register address
    input  wire [7:0]  control_reg_data,  // Control register data
    input  wire        control_reg_write, // Control register write
    output reg  [7:0]  status_register,   // Status register
    
    // Configuration Registers
    input  wire        proxy_enable,      // Enable transparent proxy
    input  wire        packet_filter_en,  // Enable packet filtering
    input  wire [15:0] packet_filter_mask,// Packet type mask for filtering
    input  wire        modify_enable,     // Enable packet modification
    input  wire [7:0]  addr_translate_en, // Enable address translation
    input  wire [6:0]  addr_translate_from,// Source address for translation
    input  wire [6:0]  addr_translate_to  // Destination address for translation
);

    // Local Parameters
    localparam PID_OUT   = 4'b0001;
    localparam PID_IN    = 4'b1001;
    localparam PID_SETUP = 4'b1101;
    localparam PID_DATA0 = 4'b0011;
    localparam PID_DATA1 = 4'b1011;
    localparam PID_DATA2 = 4'b0111;
    localparam PID_MDATA = 4'b1111;
    localparam PID_ACK   = 4'b0010;
    localparam PID_NAK   = 4'b1010;
    localparam PID_STALL = 4'b1110;
    localparam PID_NYET  = 4'b0110;
    localparam PID_SOF   = 4'b0101;
    
    // Packet direction flags
    localparam DIR_HOST_TO_DEVICE = 1'b0;
    localparam DIR_DEVICE_TO_HOST = 1'b1;
    
    // Packet buffer and state
    reg [7:0]  packet_buffer [255:0]; // Buffer for packet modification
    reg [7:0]  packet_length;        // Current packet length
    reg [3:0]  packet_pid;           // Current packet PID
    reg        packet_dir;           // Current packet direction
    
    // FSM states
    localparam ST_IDLE               = 4'd0;
    localparam ST_HOST_TO_DEVICE     = 4'd1;
    localparam ST_WAIT_DEVICE_RESP   = 4'd2;
    localparam ST_DEVICE_TO_HOST     = 4'd3;
    localparam ST_WAIT_HOST_RESP     = 4'd4;
    localparam ST_FILTER_PACKET      = 4'd5;
    localparam ST_MODIFY_PACKET      = 4'd6;
    localparam ST_BUFFER_PACKET      = 4'd7;
    localparam ST_HANDLE_SOF         = 4'd8;
    
    reg [3:0]  state;                // Current state
    reg [7:0]  byte_counter;         // Track bytes in current packet
    reg        current_toggle;       // DATA0/1 toggle tracking
    reg [15:0] last_frame_num;       // Last SOF frame number
    
    // Per-endpoint toggle tracking (16 endpoints max, IN and OUT directions)
    reg [31:0] data_toggle;          // Toggle bits (2 bits per endpoint/direction)
    
    // Modification tracking
    reg        packet_modified;      // Flag if packet has been modified
    
    // Connection tracking
    reg        device_connected;     // Device connection status
    reg [1:0]  device_speed;         // Connected device speed
    
    // Statistics counters
    reg [31:0] host_packets;         // Host packet counter
    reg [31:0] device_packets;       // Device packet counter
    reg [15:0] error_count;          // Error counter
    
    // Host to device packet routing
    assign device_tx_data = (packet_filter_en && packet_modified) ? 
                           packet_buffer[byte_counter] : host_rx_data;
    assign device_tx_valid = (state == ST_HOST_TO_DEVICE) ? host_rx_valid : 1'b0;
    assign device_tx_sop = (state == ST_HOST_TO_DEVICE) ? host_rx_sop : 1'b0;
    assign device_tx_eop = (state == ST_HOST_TO_DEVICE) ? host_rx_eop : 1'b0;
    assign device_tx_pid = (packet_modified && addr_translate_en) ? 
                          ((host_rx_pid == PID_SETUP || host_rx_pid == PID_OUT || host_rx_pid == PID_IN) ?
                           host_rx_pid : packet_pid) : host_rx_pid;
    
    // Device to host packet routing
    assign host_tx_data = (packet_filter_en && packet_modified) ? 
                         packet_buffer[byte_counter] : device_rx_data;
    assign host_tx_valid = (state == ST_DEVICE_TO_HOST) ? device_rx_valid : 1'b0;
    assign host_tx_sop = (state == ST_DEVICE_TO_HOST) ? device_rx_sop : 1'b0;
    assign host_tx_eop = (state == ST_DEVICE_TO_HOST) ? device_rx_eop : 1'b0;
    assign host_tx_pid = (packet_modified) ? packet_pid : device_rx_pid;

    // Main proxy state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            packet_length <= 8'd0;
            packet_pid <= 4'd0;
            packet_dir <= DIR_HOST_TO_DEVICE;
            packet_modified <= 1'b0;
            byte_counter <= 8'd0;
            current_toggle <= 1'b0;
            last_frame_num <= 16'd0;
            data_toggle <= 32'd0;
            device_connected <= 1'b0;
            device_speed <= 2'b00;
            host_packets <= 32'd0;
            device_packets <= 32'd0;
            error_count <= 16'd0;
            buffer_valid <= 1'b0;
            status_register <= 8'h00;
            buffer_data <= 8'h00;
            buffer_flags <= 8'h00;
            buffer_timestamp <= 64'h0;
        end else begin
            // Default assignments
            buffer_valid <= 1'b0;
        
            case (state)
                ST_IDLE: begin
                    packet_modified <= 1'b0;
                    byte_counter <= 8'd0;
                    
                    // Check for host->device transaction
                    if (proxy_enable && host_rx_sop && host_rx_valid) begin
                        state <= ST_HOST_TO_DEVICE;
                        packet_pid <= host_rx_pid;
                        packet_dir <= DIR_HOST_TO_DEVICE;
                        host_packets <= host_packets + 1'b1;
                        
                        // Capture start timestamp
                        buffer_timestamp <= timestamp;
                        
                        // Check for SOF packets - special handling
                        if (host_rx_pid == PID_SOF) begin
                            state <= ST_HANDLE_SOF;
                        end
                        
                        // Check if we need to filter/modify this packet
                        else if (packet_filter_en && (packet_filter_mask[host_rx_pid[3:0]])) begin
                            state <= ST_FILTER_PACKET;
                        end
                    end
                    
                    // Check for device->host transaction
                    else if (proxy_enable && device_rx_sop && device_rx_valid) begin
                        state <= ST_DEVICE_TO_HOST;
                        packet_pid <= device_rx_pid;
                        packet_dir <= DIR_DEVICE_TO_HOST;
                        device_packets <= device_packets + 1'b1;
                        
                        // Capture start timestamp
                        buffer_timestamp <= timestamp;
                        
                        // Check if we need to filter/modify this packet
                        if (packet_filter_en && (packet_filter_mask[device_rx_pid[3:0]])) begin
                            state <= ST_FILTER_PACKET;
                        end
                    end
                    
                    // Check link status
                    if (device_line_state != 2'b00 && !device_connected) begin
                        device_connected <= 1'b1;
                        status_register[0] <= 1'b1; // Connected bit
                    end else if (device_line_state == 2'b00 && device_connected) begin
                        device_connected <= 1'b0;
                        status_register[0] <= 1'b0; // Disconnected
                    end
                end
                
                ST_HOST_TO_DEVICE: begin
                    // Forward host to device traffic
                    if (host_rx_valid) begin
                        // Buffer packet for logging
                        if (buffer_ready && !buffer_valid) begin
                            buffer_data <= host_rx_data;
                            buffer_valid <= 1'b1;
                            buffer_flags <= {3'b000, packet_pid, DIR_HOST_TO_DEVICE};
                        end
                        
                        byte_counter <= byte_counter + 1'b1;
                    end
                    
                    if (host_rx_eop) begin
                        state <= ST_WAIT_DEVICE_RESP;
                        byte_counter <= 8'd0;
                    end
                end
                
                ST_WAIT_DEVICE_RESP: begin
                    // Wait for device response
                    if (device_rx_sop && device_rx_valid) begin
                        state <= ST_DEVICE_TO_HOST;
                        packet_pid <= device_rx_pid;
                        device_packets <= device_packets + 1'b1;
                        
                        // Update data toggle if DATA packet
                        if (host_rx_pid == PID_DATA0 || host_rx_pid == PID_DATA1) begin
                            if (device_rx_pid == PID_ACK) begin
                                // On ACK, toggle the DATA toggle bit
                                if (packet_pid == PID_OUT || packet_pid == PID_SETUP) begin
                                    // Toggle for OUT/SETUP to this endpoint
                                    data_toggle[{1'b0, host_rx_endp}] <= ~data_toggle[{1'b0, host_rx_endp}];
                                end
                            end
                        end
                    end
                    
                    // Timeout or other condition to go back to IDLE
                    if (!host_rx_valid && !device_rx_valid) begin
                        state <= ST_IDLE;
                    end
                end
                
                ST_DEVICE_TO_HOST: begin
                    // Forward device to host traffic
                    if (device_rx_valid) begin
                        // Buffer packet for logging
                        if (buffer_ready && !buffer_valid) begin
                            buffer_data <= device_rx_data;
                            buffer_valid <= 1'b1;
                            buffer_flags <= {3'b000, packet_pid, DIR_DEVICE_TO_HOST};
                        end
                        
                        byte_counter <= byte_counter + 1'b1;
                    end
                    
                    if (device_rx_eop) begin
                        state <= ST_WAIT_HOST_RESP;
                        byte_counter <= 8'd0;
                        
                        // Update data toggle if DATA packet
                        if (device_rx_pid == PID_DATA0 || device_rx_pid == PID_DATA1) begin
                            // Toggle for IN to this endpoint on successful transmission
                            if (host_rx_pid == PID_IN) begin
                                data_toggle[{1'b1, host_rx_endp}] <= ~data_toggle[{1'b1, host_rx_endp}];
                            end
                        end
                    end
                end
                
                ST_WAIT_HOST_RESP: begin
                    // Wait for host response or go back to idle
                    if (host_rx_sop || (byte_counter > 8'd200)) begin
                        state <= ST_IDLE;
                    end else begin
                        byte_counter <= byte_counter + 1'b1;
                    end
                end
                
                ST_FILTER_PACKET: begin
                    // Packet filtering logic
                    if (packet_dir == DIR_HOST_TO_DEVICE) begin
                        if (host_rx_valid) begin
                            // Store packet in buffer
                            packet_buffer[byte_counter] <= host_rx_data;
                            byte_counter <= byte_counter + 1'b1;
                        end
                        
                        if (host_rx_eop) begin
                            packet_length <= byte_counter + 1'b1;
                            byte_counter <= 8'd0;
                            
                            // Decide if we need to modify the packet
                            if (modify_enable) begin
                                state <= ST_MODIFY_PACKET;
                            end else begin
                                state <= ST_HOST_TO_DEVICE;
                            end
                        end
                    end else begin
                        // Device to host filtering
                        if (device_rx_valid) begin
                            // Store packet in buffer
                            packet_buffer[byte_counter] <= device_rx_data;
                            byte_counter <= byte_counter + 1'b1;
                        end
                        
                        if (device_rx_eop) begin
                            packet_length <= byte_counter + 1'b1;
                            byte_counter <= 8'd0;
                            
                            // Decide if we need to modify the packet
                            if (modify_enable) begin
                                state <= ST_MODIFY_PACKET;
                            end else begin
                                state <= ST_DEVICE_TO_HOST;
                            end
                        end
                    end
                end
                
                ST_MODIFY_PACKET: begin
                    // Packet modification logic
                    packet_modified <= 1'b1;
                    
                    // Address translation if enabled
                    if (addr_translate_en && 
                        (packet_pid == PID_IN || packet_pid == PID_OUT || packet_pid == PID_SETUP)) begin
                        if (packet_buffer[1][6:0] == addr_translate_from) begin
                            packet_buffer[1][6:0] <= addr_translate_to;
                        end
                    end
                    
                    // Route packet after modification
                    if (packet_dir == DIR_HOST_TO_DEVICE) begin
                        state <= ST_HOST_TO_DEVICE;
                    end else begin
                        state <= ST_DEVICE_TO_HOST;
                    end
                    
                    // Log modified packet
                    state <= ST_BUFFER_PACKET;
                end
                
                ST_BUFFER_PACKET: begin
                    // Store packet in buffer with modification flag
                    if (buffer_ready) begin
                        buffer_valid <= 1'b1;
                        buffer_data <= packet_buffer[byte_counter];
                        buffer_flags <= {2'b00, 1'b1, packet_pid, packet_dir}; // Modified flag set
                        
                        byte_counter <= byte_counter + 1'b1;
                        if (byte_counter >= packet_length - 1) begin
                            if (packet_dir == DIR_HOST_TO_DEVICE) begin
                                state <= ST_HOST_TO_DEVICE;
                            end else begin
                                state <= ST_DEVICE_TO_HOST;
                            end
                            byte_counter <= 8'd0;
                        end
                    end
                end
                
                ST_HANDLE_SOF: begin
                    // Process SOF packet
                    if (host_rx_valid) begin
                        // Extract frame number (typically in byte 1-2)
                        if (byte_counter == 8'd1) begin
                            last_frame_num[7:0] <= host_rx_data;
                        end else if (byte_counter == 8'd2) begin
                            last_frame_num[15:8] <= host_rx_data;
                        end
                        
                        byte_counter <= byte_counter + 1'b1;
                    end
                    
                    if (host_rx_eop) begin
                        state <= ST_IDLE;
                    end
                end
                
                default: state <= ST_IDLE;
            endcase
            
            // Error detection
            if ((host_rx_valid && !host_rx_crc_valid && host_rx_eop) || 
                (device_rx_valid && !device_rx_crc_valid && device_rx_eop)) begin
                error_count <= error_count + 1'b1;
                status_register[1] <= 1'b1; // Error bit
            end
            
            // Update status register
            status_register[2] <= proxy_enable;
            status_register[3] <= packet_filter_en;
            status_register[4] <= modify_enable;
            status_register[5] <= addr_translate_en;
            status_register[7:6] <= device_speed;
        end
    end
    
    // Connection speed detection
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            device_speed <= 2'b00; // Unknown speed
        end else if (event_valid) begin
            // Speed detection mode - logic based on chirp sequences/reset timing
            case (event_type)
                8'h01: device_speed <= 2'b01; // Full-speed event detected
                8'h02: device_speed <= 2'b10; // High-speed event detected
                8'h03: device_speed <= 2'b00; // Low-speed event detected
                default: ; // No change
            endcase
        end
    end

endmodule