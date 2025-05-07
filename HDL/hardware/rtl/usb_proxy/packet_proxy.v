///////////////////////////////////////////////////////////////////////////////
// File: packet_proxy.v
// Description: Packet Forwarding with Inspection Module
//
// This module handles the forwarding of USB packets between host and device
// controllers while enabling inspection, filtering, and modification.
//
// Target: Lattice ECP5 on Cynthion device
///////////////////////////////////////////////////////////////////////////////

module packet_proxy (
    // Clock and Reset
    input  wire        clk,             // System clock (60MHz)
    input  wire        clk_120mhz,      // 120MHz clock for reduced latency
    input  wire        rst_n,           // Active low reset

    // Host Controller Interface
    input  wire [7:0]  host_rx_data,    // Received data from host
    input  wire        host_rx_valid,   // Valid data from host
    input  wire        host_rx_sop,     // Start of packet from host
    input  wire        host_rx_eop,     // End of packet from host
    output reg  [7:0]  host_tx_data,    // Data to transmit to host
    output reg         host_tx_valid,   // Valid data to host
    output reg         host_tx_sop,     // Start of packet to host
    output reg         host_tx_eop,     // End of packet to host
    
    // Device Controller Interface
    input  wire [7:0]  device_rx_data,  // Received data from device
    input  wire        device_rx_valid, // Valid data from device
    input  wire        device_rx_sop,   // Start of packet from device
    input  wire        device_rx_eop,   // End of packet from device
    output reg  [7:0]  device_tx_data,  // Data to transmit to device
    output reg         device_tx_valid, // Valid data to device
    output reg         device_tx_sop,   // Start of packet to device
    output reg         device_tx_eop,   // End of packet to device
    
    // Buffer Manager Interface
    output reg  [7:0]  buffer_data,     // Data to buffer
    output reg         buffer_valid,    // Valid data for buffer
    output reg  [63:0] buffer_timestamp,// Timestamp for packet
    output reg  [7:0]  buffer_flags,    // Buffer flags (direction, type, etc)
    input  wire        buffer_ready,    // Buffer is ready for data
    
    // Timestamp Generator Interface
    input  wire [63:0] timestamp,       // Current timestamp value
    
    // Protocol Identification
    output reg  [3:0]  packet_pid,      // Current packet PID
    output reg         is_token_packet, // Packet is a token
    output reg         is_data_packet,  // Packet is data
    output reg         is_handshake,    // Packet is handshake
    output reg  [6:0]  device_addr,     // Device address from packet
    output reg  [3:0]  endpoint_num,    // Endpoint number from packet
    
    // Control Interface
    input  wire [7:0]  control_reg_addr,// Control register address
    input  wire [7:0]  control_reg_data,// Control register data
    input  wire        control_reg_write,// Control register write enable
    
    // Configuration
    input  wire        enable_proxy,    // Enable proxy functionality
    input  wire        enable_logging,  // Enable packet logging
    input  wire        enable_filtering,// Enable packet filtering
    input  wire [15:0] packet_filter,   // Packet type filter (by PID)
    input  wire        enable_modify,   // Enable packet modification
    input  wire [7:0]  modify_flags     // Modification control flags
);

    // Local Parameters
    localparam PID_OUT    = 4'b0001;
    localparam PID_IN     = 4'b1001;
    localparam PID_SETUP  = 4'b1101;
    localparam PID_DATA0  = 4'b0011;
    localparam PID_DATA1  = 4'b1011;
    localparam PID_DATA2  = 4'b0111;
    localparam PID_MDATA  = 4'b1111;
    localparam PID_ACK    = 4'b0010;
    localparam PID_NAK    = 4'b1010;
    localparam PID_STALL  = 4'b1110;
    localparam PID_NYET   = 4'b0110;
    localparam PID_SOF    = 4'b0101;
    
    // Buffer flags
    localparam FLAG_DIR_H2D     = 8'h00;  // Host to device
    localparam FLAG_DIR_D2H     = 8'h01;  // Device to host
    localparam FLAG_MODIFIED    = 8'h02;  // Packet was modified
    localparam FLAG_CRC_ERROR   = 8'h04;  // CRC error detected
    localparam FLAG_FILTERED    = 8'h08;  // Packet was filtered
    
    // FSM States
    localparam ST_IDLE            = 4'd0;
    localparam ST_H2D_FORWARD     = 4'd1;  // Host to Device forwarding
    localparam ST_D2H_FORWARD     = 4'd2;  // Device to Host forwarding
    localparam ST_PACKET_CAPTURE  = 4'd3;  // Capture packet for inspection
    localparam ST_PACKET_INSPECT  = 4'd4;  // Inspect packet content
    localparam ST_PACKET_MODIFY   = 4'd5;  // Modify packet content
    localparam ST_PACKET_FILTER   = 4'd6;  // Filter packet decision
    localparam ST_BUFFER_WRITE    = 4'd7;  // Write packet to buffer
    localparam ST_FORWARD_MODIFIED = 4'd8; // Forward modified packet
    
    // Internal Registers
    reg [3:0]  state;                      // Current FSM state
    reg [3:0]  next_state;                 // Next state for return after inspect
    (* ram_style = "block", mem_init = "0" *) reg [7:0] packet_buffer [255:0];  // Block RAM for packet buffer
    reg [7:0] buffer_write_addr;                // Registered write address
    reg buffer_we;                              // Write enable register
    reg [7:0]  packet_length;              // Current packet length
    reg [7:0]  buffer_index;               // Current index in buffer
    reg        packet_direction;           // 0=Host to Device, 1=Device to Host
    reg        packet_complete;            // Packet is complete
    reg        packet_processed;           // Packet has been processed
    reg        packet_modified;            // Packet has been modified
    reg        packet_filtered;            // Packet should be filtered (dropped)
    reg        token_received;             // Token packet received
    reg        data_phase;                 // In data phase of transfer
    reg [15:0] last_token;                 // Last token packet
    reg [15:0] frame_number;               // SOF frame number
    
    // CRC check registers
    reg [15:0] token_crc5;                 // CRC5 for token packets
    reg [15:0] data_crc16;                 // CRC16 for data packets
    reg        crc_valid;                  // CRC is valid
    
    // Performance optimization
    reg        fast_path_enabled;          // Enable low-latency path
    
    // Fast-path direct connections for minimum latency when inspection not needed
    wire       use_fast_path;              // Use the fast path
    
    // Determine if packet should be inspected or fast-pathed
    assign use_fast_path = fast_path_enabled && enable_proxy && 
                           !enable_filtering && !enable_modify;

    // Packet PID detection
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            packet_pid <= 4'h0;
            is_token_packet <= 1'b0;
            is_data_packet <= 1'b0;
            is_handshake <= 1'b0;
            device_addr <= 7'h0;
            endpoint_num <= 4'h0;
        end else begin
            // Detect packet type from first byte (PID)
            if (host_rx_sop && host_rx_valid) begin
                packet_pid <= host_rx_data[3:0]; // PID is in low nibble
                
                // Classify packet type
                case (host_rx_data[3:0])
                    PID_OUT, PID_IN, PID_SETUP, PID_SOF: begin
                        is_token_packet <= 1'b1;
                        is_data_packet <= 1'b0;
                        is_handshake <= 1'b0;
                    end
                    
                    PID_DATA0, PID_DATA1, PID_DATA2, PID_MDATA: begin
                        is_token_packet <= 1'b0;
                        is_data_packet <= 1'b1;
                        is_handshake <= 1'b0;
                    end
                    
                    PID_ACK, PID_NAK, PID_STALL, PID_NYET: begin
                        is_token_packet <= 1'b0;
                        is_data_packet <= 1'b0;
                        is_handshake <= 1'b1;
                    end
                    
                    default: begin
                        is_token_packet <= 1'b0;
                        is_data_packet <= 1'b0;
                        is_handshake <= 1'b0;
                    end
                endcase
            end else if (device_rx_sop && device_rx_valid) begin
                packet_pid <= device_rx_data[3:0]; // PID is in low nibble
                
                // Similar packet classification for device packets
                case (device_rx_data[3:0])
                    PID_DATA0, PID_DATA1, PID_DATA2, PID_MDATA: begin
                        is_token_packet <= 1'b0;
                        is_data_packet <= 1'b1;
                        is_handshake <= 1'b0;
                    end
                    
                    PID_ACK, PID_NAK, PID_STALL, PID_NYET: begin
                        is_token_packet <= 1'b0;
                        is_data_packet <= 1'b0;
                        is_handshake <= 1'b1;
                    end
                    
                    default: begin
                        is_token_packet <= 1'b0;
                        is_data_packet <= 1'b0;
                        is_handshake <= 1'b0;
                    end
                endcase
            end
            
            // Extract device address and endpoint from token packets
            if (is_token_packet && host_rx_valid) begin
                if (buffer_index == 8'd1) begin
                    device_addr <= host_rx_data[6:0];
                    endpoint_num[0] <= host_rx_data[7];
                end else if (buffer_index == 8'd2) begin
                    endpoint_num[3:1] <= host_rx_data[2:0];
                end
            end
        end
    end
    
    // Main proxy state machine - handles packet forwarding, inspection, and modification
    always @(posedge clk_120mhz or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            next_state <= ST_IDLE;
            packet_length <= 8'd0;
            buffer_index <= 8'd0;
            packet_direction <= 1'b0;
            packet_complete <= 1'b0;
            packet_processed <= 1'b0;
            packet_modified <= 1'b0;
            packet_filtered <= 1'b0;
            token_received <= 1'b0;
            data_phase <= 1'b0;
            last_token <= 16'h0000;
            frame_number <= 16'h0000;
            fast_path_enabled <= 1'b1;
            crc_valid <= 1'b1;
            
            // Output registers
            host_tx_data <= 8'h00;
            host_tx_valid <= 1'b0;
            host_tx_sop <= 1'b0;
            host_tx_eop <= 1'b0;
            device_tx_data <= 8'h00;
            device_tx_valid <= 1'b0;
            device_tx_sop <= 1'b0;
            device_tx_eop <= 1'b0;
            buffer_data <= 8'h00;
            buffer_valid <= 1'b0;
            buffer_timestamp <= 64'h0000000000000000;
            buffer_flags <= 8'h00;
        end else begin
            // Default assignments
            host_tx_valid <= 1'b0;
            host_tx_sop <= 1'b0;
            host_tx_eop <= 1'b0;
            device_tx_valid <= 1'b0;
            device_tx_sop <= 1'b0;
            device_tx_eop <= 1'b0;
            buffer_valid <= 1'b0;
            
            case (state)
                ST_IDLE: begin
                    // Reset packet processing flags
                    packet_complete <= 1'b0;
                    packet_processed <= 1'b0;
                    packet_modified <= 1'b0;
                    packet_filtered <= 1'b0;
                    buffer_index <= 8'd0;
                    
                    // Check for incoming packets
                    if (host_rx_valid && host_rx_sop) begin
                        // Host to device packet detected
                        packet_direction <= 1'b0;
                        buffer_timestamp <= timestamp; // Timestamp the packet
                        
                        if (use_fast_path) begin
                            // Fast path - direct forwarding with minimal latency
                            state <= ST_H2D_FORWARD;
                            device_tx_data <= host_rx_data;
                            device_tx_valid <= 1'b1;
                            device_tx_sop <= 1'b1;
                        end else begin
                            // Inspection path - capture packet for analysis
                            state <= ST_PACKET_CAPTURE;
                            packet_buffer[0] <= host_rx_data;
                            buffer_index <= 8'd1;
                            
                            // Identify token packets
                            if (is_token_packet) begin
                                token_received <= 1'b1;
                                // Save the SOF frame number
                                if (packet_pid == PID_SOF) begin
                                    data_phase <= 1'b0;
                                end else begin
                                    data_phase <= 1'b0; // Will become 1 after token
                                end
                            end
                        end
                    end else if (device_rx_valid && device_rx_sop) begin
                        // Device to host packet
                        packet_direction <= 1'b1;
                        buffer_timestamp <= timestamp; // Timestamp the packet
                        
                        if (use_fast_path) begin
                            // Fast path
                            state <= ST_D2H_FORWARD;
                            host_tx_data <= device_rx_data;
                            host_tx_valid <= 1'b1;
                            host_tx_sop <= 1'b1;
                        end else begin
                            // Inspection path
                            state <= ST_PACKET_CAPTURE;
                            packet_buffer[0] <= device_rx_data;
                            buffer_index <= 8'd1;
                            
                            // Update data phase flag
                            if (is_data_packet) begin
                                data_phase <= 1'b1;
                            end else if (is_handshake) begin
                                data_phase <= 1'b0;
                            end
                        end
                    end
                end
                
                ST_H2D_FORWARD: begin
                    // Direct forwarding from host to device (fast path)
                    if (host_rx_valid) begin
                        device_tx_data <= host_rx_data;
                        device_tx_valid <= 1'b1;
                        
                        // Log packet if enabled
                        if (enable_logging && buffer_ready) begin
                            buffer_data <= host_rx_data;
                            buffer_valid <= 1'b1;
                            buffer_flags <= FLAG_DIR_H2D;
                        end
                    end
                    
                    if (host_rx_eop) begin
                        device_tx_eop <= 1'b1;
                        state <= ST_IDLE;
                    end
                end
                
                ST_D2H_FORWARD: begin
                    // Direct forwarding from device to host (fast path)
                    if (device_rx_valid) begin
                        host_tx_data <= device_rx_data;
                        host_tx_valid <= 1'b1;
                        
                        // Log packet if enabled
                        if (enable_logging && buffer_ready) begin
                            buffer_data <= device_rx_data;
                            buffer_valid <= 1'b1;
                            buffer_flags <= FLAG_DIR_D2H;
                        end
                    end
                    
                    if (device_rx_eop) begin
                        host_tx_eop <= 1'b1;
                        state <= ST_IDLE;
                    end
                end
                
                ST_PACKET_CAPTURE: begin
                    // Capture complete packet for inspection
                    if (packet_direction == 1'b0) begin
                        // Host to device capture
                        if (host_rx_valid) begin
                            packet_buffer[buffer_index] <= host_rx_data;
                            buffer_index <= buffer_index + 8'd1;
                        end
                        
                        if (host_rx_eop) begin
                            packet_length <= buffer_index;
                            packet_complete <= 1'b1;
                            state <= ST_PACKET_INSPECT;
                        end
                    end else begin
                        // Device to host capture
                        if (device_rx_valid) begin
                            packet_buffer[buffer_index] <= device_rx_data;
                            buffer_index <= buffer_index + 8'd1;
                        end
                        
                        if (device_rx_eop) begin
                            packet_length <= buffer_index;
                            packet_complete <= 1'b1;
                            state <= ST_PACKET_INSPECT;
                        end
                    end
                end
                
                ST_PACKET_INSPECT: begin
                    // Basic packet inspection and decision making
                    if (enable_filtering && packet_filter[packet_pid]) begin
                        // Packet matches filter criteria - check if we should filter it
                        state <= ST_PACKET_FILTER;
                    end else if (enable_modify && modify_flags[0]) begin
                        // Packet modification requested
                        state <= ST_PACKET_MODIFY;
                    end else begin
                        // No filtering or modification needed - proceed to buffering if enabled
                        if (enable_logging) begin
                            buffer_index <= 8'd0;
                            next_state <= packet_direction ? ST_D2H_FORWARD : ST_H2D_FORWARD;
                            state <= ST_BUFFER_WRITE;
                        end else begin
                            // Forward packet
                            buffer_index <= 8'd0;
                            state <= packet_direction ? ST_D2H_FORWARD : ST_H2D_FORWARD;
                        end
                    end
                end
                
                ST_PACKET_FILTER: begin
                    // Implement filtering logic
                    // Here we decide if packet should be dropped or forwarded
                    
                    // Example: Filter out SOF packets (don't forward)
                    if (packet_pid == PID_SOF) begin
                        packet_filtered <= 1'b1;
                        if (enable_logging) begin
                            buffer_index <= 8'd0;
                            next_state <= ST_IDLE; // Drop packet
                            state <= ST_BUFFER_WRITE;
                            buffer_flags <= packet_direction ? 
                                          (FLAG_DIR_D2H | FLAG_FILTERED) : 
                                          (FLAG_DIR_H2D | FLAG_FILTERED);
                        end else begin
                            state <= ST_IDLE; // Drop packet
                        end
                    // Example: Filter specific device address
                    end else if (is_token_packet && device_addr == 7'h7F) begin
                        packet_filtered <= 1'b1;
                        if (enable_logging) begin
                            buffer_index <= 8'd0;
                            next_state <= ST_IDLE; // Drop packet
                            state <= ST_BUFFER_WRITE;
                            buffer_flags <= packet_direction ? 
                                          (FLAG_DIR_D2H | FLAG_FILTERED) : 
                                          (FLAG_DIR_H2D | FLAG_FILTERED);
                        end else begin
                            state <= ST_IDLE; // Drop packet
                        end
                    end else begin
                        // No filtering needed, proceed to modification or forwarding
                        if (enable_modify && modify_flags[0]) begin
                            state <= ST_PACKET_MODIFY;
                        end else begin
                            // Forward packet
                            if (enable_logging) begin
                                buffer_index <= 8'd0;
                                next_state <= packet_direction ? ST_D2H_FORWARD : ST_H2D_FORWARD;
                                state <= ST_BUFFER_WRITE;
                            end else begin
                                buffer_index <= 8'd0;
                                state <= packet_direction ? ST_D2H_FORWARD : ST_H2D_FORWARD;
                            end
                        end
                    end
                end
                
                ST_PACKET_MODIFY: begin
                    // Packet modification logic
                    packet_modified <= 1'b1;
                    
                    // Example: Modify descriptor packets to change device capabilities
                    // In practice, this would be implemented based on specific requirements
                    
                    // Once modified, proceed to forwarding
                    buffer_index <= 8'd0;
                    state <= ST_FORWARD_MODIFIED;
                    buffer_flags <= packet_direction ? 
                                  (FLAG_DIR_D2H | FLAG_MODIFIED) : 
                                  (FLAG_DIR_H2D | FLAG_MODIFIED);
                end
                
                ST_FORWARD_MODIFIED: begin
                    // Forward modified packet
                    if (buffer_index < packet_length) begin
                        if (packet_direction == 1'b0) begin
                            // Host to device forwarding
                            device_tx_data <= packet_buffer[buffer_index];
                            device_tx_valid <= 1'b1;
                            if (buffer_index == 8'd0) device_tx_sop <= 1'b1;
                            if (buffer_index == packet_length - 1) device_tx_eop <= 1'b1;
                        end else begin
                            // Device to host forwarding
                            host_tx_data <= packet_buffer[buffer_index];
                            host_tx_valid <= 1'b1;
                            if (buffer_index == 8'd0) host_tx_sop <= 1'b1;
                            if (buffer_index == packet_length - 1) host_tx_eop <= 1'b1;
                        end
                        
                        // Log packet if enabled
                        if (enable_logging && buffer_ready) begin
                            buffer_data <= packet_buffer[buffer_index];
                            buffer_valid <= 1'b1;
                        end
                        
                        buffer_index <= buffer_index + 8'd1;
                    end else begin
                        // Packet forwarding complete
                        state <= ST_IDLE;
                    end
                end
                
                ST_BUFFER_WRITE: begin
                    // Write packet to buffer manager
                    if (buffer_ready) begin
                        if (buffer_index < packet_length) begin
                            buffer_data <= packet_buffer[buffer_index];
                            buffer_valid <= 1'b1;
                            buffer_index <= buffer_index + 8'd1;
                        end else begin
                            // Buffer writing complete, go to next state
                            state <= next_state;
                            buffer_index <= 8'd0;
                        end
                    end else begin
                        // Buffer not ready, skip logging
                        state <= next_state;
                        buffer_index <= 8'd0;
                    end
                end
                
                default: state <= ST_IDLE;
            endcase
        end
    end
    
    // Configuration register write handling
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset configuration
            fast_path_enabled <= 1'b1;
        end else if (control_reg_write) begin
            case (control_reg_addr)
                8'h01: fast_path_enabled <= control_reg_data[0]; // Latency optimization control
            endcase
        end
    end

endmodule