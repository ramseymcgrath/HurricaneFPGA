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
    output reg         overflow_detected, // Buffer overflow indicator

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
    output reg  [7:0]  status_read_data,  // Readback data

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
    
    // Status register address
    localparam STATUS_REG_ADDR = 8'h00;
    
    // Packet direction flags
    localparam DIR_HOST_TO_DEVICE = 1'b0;
    localparam DIR_DEVICE_TO_HOST = 1'b1;
    
    // Packet buffer and state
    (* ram_style = "distributed", mem_init = "0" *) reg [7:0] monitor_packet_buffer [255:0]; // Buffer for packet modification
    reg [7:0]  packet_length;        // Current packet length
    reg [3:0]  last_host_pid;        // Last host PID (4-bit PID)
    reg [3:0]  last_device_pid;      // Last device PID (4-bit PID)
    reg [3:0]  latched_host_pid;     // Host PID saved for state transitions
    reg [3:0]  latched_device_pid;   // Device PID saved for state transitions
    reg        pending_dir;          // Direction flag for deferred routing
    reg        buffer_write_en;      // Buffer write enable
    reg        packet_dir;           // Current packet direction
    reg [3:0]  modify_pid;           // PID for modification
    reg        fifo_full;            // FIFO full flag

    // Endpoint index combines direction and endpoint number (5-bit: 1b dir + 4b endp)
    wire [4:0] endpoint_index;
    wire       index_valid;
    wire       index_overflow;
    
    assign endpoint_index = {packet_dir, host_rx_endp};
    assign index_valid = (host_rx_endp <= 4'hF); // USB spec allows max 15 endpoints
    assign index_overflow = (endpoint_index >= 5'd16); // Protect 32-entry data_toggle array
    
    // FSM states
    localparam ST_IDLE               = 4'd0;
    localparam ST_HOST_TO_DEVICE     = 4'd1;
    localparam ST_WAIT_DEVICE_RESP   = 4'd2;
    localparam ST_DEVICE_TO_HOST     = 4'd3;
    localparam ST_WAIT_HOST_RESP     = 4'd4;
    localparam ST_FILTER_PACKET      = 4'd5;
    localparam ST_MODIFY_PACKET      = 4'd6;
    localparam ST_ROUTE_PACKET       = 4'd9; // Post-modification routing
    localparam ST_BUFFER_VALIDATE    = 4'd10; // Buffer validation state
    localparam ST_BUFFER_PACKET      = 4'd7;
    localparam ST_HANDLE_SOF         = 4'd8;
    
    reg [3:0]  state;                // Current state
    reg [7:0]  byte_counter;         // Track bytes in current packet
    reg        current_toggle;       // DATA0/1 toggle tracking
    reg [15:0] last_frame_num;       // Last SOF frame number
    
    // Per-endpoint toggle tracking (16 endpoints max, IN and OUT directions)
    reg [31:0] data_toggle;          // Toggle bits (1 bit per endpoint/direction)
    
    // Helper function for address byte index
    function automatic [7:0] get_address_byte_index;
        input [3:0] pid;
        begin
            case (pid)
                PID_SETUP, PID_OUT, PID_IN: get_address_byte_index = 8'd1;  // Address at byte 1
                default: get_address_byte_index = 8'd0;                     // Default to byte 0
            endcase
        end
    endfunction

    // Modification tracking
    reg        packet_modified;      // Flag if packet has been modified
    
    // Connection tracking
    reg        device_connected;     // Device connection status
    reg [1:0]  device_speed;         // Connected device speed
    
    // Statistics counters
    reg [31:0] host_packets;         // Host packet counter
    reg [31:0] device_packets;       // Device packet counter
    reg [15:0] error_count;          // Error counter
    reg        overflow_detected;    // Buffer overflow indicator
    
    // Host to device packet routing
    assign device_tx_data = (packet_filter_en && packet_modified) ?
                           monitor_packet_buffer[byte_counter] : host_rx_data;
    assign device_tx_valid = (state == ST_HOST_TO_DEVICE) ? host_rx_valid : 1'b0;
    assign device_tx_sop = (state == ST_HOST_TO_DEVICE) ? host_rx_sop : 1'b0;
    assign device_tx_eop = (state == ST_HOST_TO_DEVICE) ? host_rx_eop : 1'b0;
    assign device_tx_pid = (packet_modified && addr_translate_en) ?
                          ((host_rx_pid == PID_SETUP || host_rx_pid == PID_OUT || host_rx_pid == PID_IN) ?
                           host_rx_pid : last_host_pid) : host_rx_pid;
    
    // Device to host packet routing
    assign host_tx_data = (packet_filter_en && packet_modified) ?
                         monitor_packet_buffer[byte_counter] : device_rx_data;
    assign host_tx_valid = (state == ST_DEVICE_TO_HOST) ? device_rx_valid : 1'b0;
    assign host_tx_sop = (state == ST_DEVICE_TO_HOST) ? device_rx_sop : 1'b0;
    assign host_tx_eop = (state == ST_DEVICE_TO_HOST) ? device_rx_eop : 1'b0;
    assign host_tx_pid = (packet_modified) ? last_device_pid : device_rx_pid;

    // Main proxy state machine
    always @(posedge clk or negedge rst_n) begin : main_state_machine
        if (!rst_n) begin
            state <= ST_IDLE;
            packet_length <= 8'd0;
            packet_dir <= DIR_HOST_TO_DEVICE;
            packet_modified <= 1'b0;
            byte_counter <= 8'd0;
            current_toggle <= 1'b0;
            last_frame_num <= 16'd0;
            data_toggle <= 32'd0;
            device_connected <= 1'b0;
            // Reset newly added signals
            pending_dir <= 1'b0;
            device_speed <= 2'b00;
            host_packets <= 32'd0;
            device_packets <= 32'd0;
            error_count <= 16'd0;
            buffer_valid <= 1'b0;
            status_register <= 8'h00;
            buffer_data <= 8'h00;
            buffer_flags <= 8'h00;
            buffer_timestamp <= 64'h0;
            fifo_full <= 1'b0;
            modify_pid <= 4'h0;
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
                        last_host_pid <= host_rx_pid;
                        latched_host_pid <= host_rx_pid;  // Capture for state transitions
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
                        last_device_pid <= device_rx_pid;
                        latched_device_pid <= device_rx_pid;  // Capture for state transitions
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
                            buffer_flags <= {3'b000, last_host_pid, DIR_HOST_TO_DEVICE};
                        end
                        
                        buffer_write_en <= (state == ST_BUFFER_PACKET) && (byte_counter < 8'hFF);
                        byte_counter <= (byte_counter == 8'hFF) ? 8'hFF : byte_counter + 1'b1;
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
                        device_packets <= device_packets + 1'b1;
                        
                        // Update data toggle if DATA packet using latched host PID
                        if (latched_host_pid == PID_DATA0 || latched_host_pid == PID_DATA1) begin
                            if (device_rx_pid == PID_ACK) begin
                                // On ACK, toggle using original host PID
                                if (latched_host_pid == PID_OUT || latched_host_pid == PID_SETUP) begin
                                    // Toggle for OUT/SETUP to this endpoint
                                    if (index_valid && !index_overflow) begin
                                        data_toggle[endpoint_index] <= ~data_toggle[endpoint_index];
                                    end else if (index_overflow) begin
                                        status_register[5] <= 1'b1; // Set index error flag
                                    end
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
                            buffer_flags <= {3'b000, last_device_pid, DIR_DEVICE_TO_HOST};
                            last_device_pid <= device_rx_pid;  // Track device-side PID
                            buffer_write_en <= 1'b1;  // Enable buffer write
                        end
                        
                        if (byte_counter < 8'hFF) begin
                            byte_counter <= byte_counter + 1'b1;
                        end else begin
                            byte_counter <= 8'hFF;
                            status_register[7] <= 1'b1; // Set overflow flag
                        end
                    end
                    
                    if (device_rx_eop) begin
                        state <= ST_WAIT_HOST_RESP;
                        byte_counter <= 8'd0;
                        
                        // Update data toggle if DATA packet
                        if (device_rx_pid == PID_DATA0 || device_rx_pid == PID_DATA1) begin
                            // Use direction-aware endpoint index
                            if (last_device_pid == PID_IN && !index_overflow) begin
                                data_toggle[endpoint_index] <= ~data_toggle[endpoint_index];
                            end else if (index_overflow) begin
                                status_register[5] <= 1'b1; // Set index error flag
                            end
                        end
                    end
                end
                
                ST_WAIT_HOST_RESP: begin
                    // Wait for host response or go back to idle
                    if (host_rx_sop || (byte_counter > 8'd200)) begin
                        state <= ST_IDLE;
                        // Store host PID before transition
                        last_host_pid <= host_rx_pid;
                    end else begin
                        byte_counter <= byte_counter + 1'b1;
                    end
                end
                
                ST_FILTER_PACKET: begin
                    // Packet filtering logic
                    if (packet_dir == DIR_HOST_TO_DEVICE) begin
                        // Host to device filtering
                        // Generate write enable only for valid data cycles
                        buffer_write_en <= host_rx_valid && buffer_ready && !fifo_full;
                        
                        if (buffer_write_en) begin
                            // Store packet in buffer only when ready
                            if (byte_counter < 8'hFF) begin
                                // Special handling for address byte
                                if (byte_counter == 1) begin // Address byte index
                                    monitor_packet_buffer[1][6:0] <=
                                        (host_rx_data[6:0] == addr_translate_from) ?
                                        addr_translate_to : host_rx_data[6:0];
                                end else begin
                                    monitor_packet_buffer[byte_counter] <= host_rx_data;
                                end
                                // Clamp counter at max value
                                byte_counter <= (byte_counter == 8'hFF) ? 8'hFF : byte_counter + 1'b1;
                            end
                        end
                        
                        if (host_rx_eop) begin
                            packet_length <= (byte_counter == 8'hFF) ? 8'hFF : byte_counter + 1'b1;
                            byte_counter <= 8'd0;
                            
                            // Buffer before routing
                            state <= ST_BUFFER_PACKET;
                        end
                    end else begin
                        // Device to host filtering
                        if (device_rx_valid && buffer_write_en && buffer_ready) begin
                            if (byte_counter < 8'hFF) begin
                               monitor_packet_buffer[byte_counter] <= device_rx_data;
                               buffer_write_en <= 1'b0;  // Clear after write
                               byte_counter <= byte_counter + 1'b1;
                            end
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
                    // Packet modification logic using direction-specific PIDs
                    packet_modified <= 1'b1;

                    // Buffer before routing
                    state <= ST_BUFFER_PACKET;
                    pending_dir <= packet_dir;

                    // Gate buffer writes with valid flag
                    buffer_valid <= (byte_counter < packet_length);

                    // Address translation if enabled
                    if (addr_translate_en) begin
                        modify_pid <= (packet_dir == DIR_HOST_TO_DEVICE) ? 
                                      latched_host_pid : latched_device_pid;
                                      
                        // Check if the PID is one we should modify address for
                        if ((modify_pid == PID_IN) || (modify_pid == PID_OUT) || (modify_pid == PID_SETUP)) begin
                            // Get address byte index using PID type
                            if (get_address_byte_index(modify_pid) > 0 && 
                                get_address_byte_index(modify_pid) <= packet_length) begin
                                
                                // Perform the address translation if address matches
                                if (monitor_packet_buffer[1][6:0] == addr_translate_from) begin
                                    monitor_packet_buffer[1][6:0] <= addr_translate_to;
                                end
                            end
                        end
                    end

                    // State transition logic
                    if (byte_counter == packet_length) begin
                        state <= ST_BUFFER_VALIDATE;
                        pending_dir <= packet_dir;
                    end else if (byte_counter > 8'hFF) begin
                        status_register[7] <= 1'b1; // Set buffer overflow flag
                        status_register[6] <= (byte_counter == 8'hFF) ? 1'b1 : 1'b0; // Set max length flag
                        state <= ST_IDLE;
                    end
                    // Reset byte counter for buffer write
                    byte_counter <= 8'd0;
                end
                
                ST_BUFFER_PACKET: begin
                    // Store packet in buffer with modification flag
                    if (buffer_ready) begin
                        buffer_valid <= 1'b1;
                        buffer_data <= monitor_packet_buffer[byte_counter];
                        buffer_flags <= {2'b00, 1'b1, modify_pid, packet_dir};
                        
                        byte_counter <= byte_counter + 1'b1;
                        if (byte_counter >= packet_length - 1) begin
                            if (pending_dir == DIR_HOST_TO_DEVICE) begin
                                state <= ST_HOST_TO_DEVICE;
                            end else begin
                                state <= ST_DEVICE_TO_HOST;
                            end
                            byte_counter <= 8'd0;
                        end
                    end
                end
                
                ST_BUFFER_VALIDATE: begin
                    // Validate buffer contents
                    state <= ST_ROUTE_PACKET;
                end
                
                ST_ROUTE_PACKET: begin
                    // Route the modified packet
                    if (pending_dir == DIR_HOST_TO_DEVICE) begin
                        state <= ST_HOST_TO_DEVICE;
                    end else begin
                        state <= ST_DEVICE_TO_HOST;
                    end
                    byte_counter <= 8'd0;
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
            status_register[5] <= overflow_detected;  // Add overflow bit
            status_register[6] <= addr_translate_en[0];
            status_register[7:6] <= device_speed;
        end
    end

    // Status register readback - consolidated into a single case statement
    always @(*) begin
        case (control_reg_addr)
            // Status registers
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
            
            // Debug registers
            8'h10: status_read_data = {4'h0, last_host_pid};
            8'h11: status_read_data = {4'h0, last_device_pid};
            8'h12: status_read_data = byte_counter;
            8'h13: status_read_data = {6'b0, device_speed};
            8'h14: status_read_data = {7'b0, device_connected};
            
            // Control register mirroring
            8'hF0: status_read_data = {addr_translate_en[7:1], overflow_detected};
            8'hF1: status_read_data = addr_translate_from;
            8'hF2: status_read_data = addr_translate_to;
            8'hF3: status_read_data = {proxy_enable, packet_filter_en, modify_enable, 5'b0};
            
            default: status_read_data = 8'h00;
        endcase
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

    // Byte counter overflow detection
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            overflow_detected <= 1'b0;
            status_register[5] <= 1'b0;
        end else if (byte_counter == 8'hFF) begin
            overflow_detected <= 1'b1;
            status_register[5] <= 1'b1; // Overflow warning flag
        end
    end

endmodule