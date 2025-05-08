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
    output reg         pid_ack_120,       // PID ACK detection signal (120MHz domain)

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
    output wire        host_tx_valid_mux, // Muxed valid signal for debug mode
    output wire        host_tx_sop,       // Start of packet to host
    output wire        host_tx_eop,       // End of packet to host
    output wire [3:0]  host_tx_pid,       // PID to send to host
    
    // Device Side USB Interface
    input  wire [7:0]  device_rx_data,    // Decoded data from device
    input  wire        device_rx_valid,   // Device data valid
    input  wire        device_rx_sop,     // Start of device packet
    input  wire        device_rx_eop,     // End of device packet
    input  wire [3:0]  device_rx_pid,     // USB PID from device
    input  wire [3:0]  device_rx_endp,    // Endpoint from device packets
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
    localparam PID_ACK   = 4'b0010;
    localparam PID_DATA0 = 4'b0011;
    localparam PID_DATA1 = 4'b1011;
    localparam PID_SOF   = 4'b0101;
    
    // Status register bits
    localparam STAT_CONNECTED  = 0;
    localparam STAT_ERROR      = 1;
    localparam STAT_PROXY_EN   = 2;
    localparam STAT_FILTER_EN  = 3;
    localparam STAT_MODIFY_EN  = 4;
    localparam STAT_OVERFLOW   = 5;
    localparam STAT_SPEED_HIGH = 6;
    localparam STAT_SPEED_FULL = 7;

    // Packet direction flags
    localparam DIR_HOST_TO_DEVICE = 1'b0;
    localparam DIR_DEVICE_TO_HOST = 1'b1;
    
    // FSM states
    localparam ST_IDLE               = 4'd0;
    localparam ST_HOST_TO_DEVICE     = 4'd1;
    localparam ST_WAIT_DEVICE_RESP   = 4'd2;
    localparam ST_DEVICE_TO_HOST     = 4'd3;
    localparam ST_WAIT_HOST_RESP     = 4'd4;
    localparam ST_FILTER_PACKET      = 4'd5;
    localparam ST_MODIFY_PACKET      = 4'd6;
    localparam ST_ROUTE_PACKET       = 4'd9;
    localparam ST_BUFFER_VALIDATE    = 4'd10;
    localparam ST_BUFFER_PACKET      = 4'd7;
    localparam ST_HANDLE_SOF         = 4'd8;

    // Internal registers
    (* ram_style = "distributed" *) reg [7:0] monitor_packet_buffer [0:255];
    reg [7:0]  packet_length;
    reg [3:0]  state, next_state;
    reg [7:0]  byte_counter;
    reg        packet_dir;
    reg        packet_modified;
    reg [3:0]  latched_host_pid;
    reg [3:0]  latched_device_pid;
    reg [31:0] data_toggle;
    reg        current_toggle;
    reg [15:0] last_frame_num;
    reg        device_connected;
    reg [1:0]  device_speed;
    reg [31:0] host_packets;
    reg [31:0] device_packets;
    reg [15:0] error_count;
    reg        buffer_write_en;
    reg  [1:0] pid_ack_counter;  // Counter for PID ACK pulse generation

    // Endpoint index calculation
    wire [4:0] endpoint_index = packet_dir ? 
                               {1'b1, device_rx_endp} : 
                               {1'b0, host_rx_endp};
    wire       index_valid = (endpoint_index < 5'd16);

    // Output assignments
    assign device_tx_data = packet_modified ? 
                           monitor_packet_buffer[byte_counter] : host_rx_data;
    assign device_tx_valid = (state == ST_HOST_TO_DEVICE) ? host_rx_valid : 1'b0;
    assign device_tx_sop = (state == ST_HOST_TO_DEVICE) ? host_rx_sop : 1'b0;
    assign device_tx_eop = (state == ST_HOST_TO_DEVICE) ? host_rx_eop : 1'b0;
    assign device_tx_pid = packet_modified ? latched_host_pid : host_rx_pid;

    assign host_tx_data = packet_modified ? 
                        monitor_packet_buffer[byte_counter] : device_rx_data;
    assign host_tx_valid = (state == ST_DEVICE_TO_HOST) ? device_rx_valid : 1'b0;
    assign host_tx_sop = (state == ST_DEVICE_TO_HOST) ? device_rx_sop : 1'b0;
    assign host_tx_eop = (state == ST_DEVICE_TO_HOST) ? device_rx_eop : 1'b0;
    assign host_tx_pid = packet_modified ? latched_device_pid : device_rx_pid;

    // Main state machine
    always @(posedge clk_120mhz or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            packet_length <= 0;
            packet_dir <= DIR_HOST_TO_DEVICE;
            packet_modified <= 0;
            byte_counter <= 0;
            data_toggle <= 0;
            device_connected <= 0;
            host_packets <= 0;
            device_packets <= 0;
            error_count <= 0;
            status_register <= 0;
            buffer_valid <= 0;
            buffer_data <= 0;
            buffer_flags <= 0;
            buffer_timestamp <= 0;
            buffer_write_en <= 0;
            last_frame_num <= 0;
            current_toggle <= 0;
            pid_ack_120 <= 1'b0;
            pid_ack_counter <= 2'b00;
            
        end else begin
            // Default assignments
            buffer_valid <= 0;
            status_register[STAT_ERROR] <= 0;
            status_register[STAT_OVERFLOW] <= overflow_detected;

            case (state)
                ST_IDLE: begin
                    byte_counter <= 0;
                    packet_modified <= 0;
                    if (proxy_enable) begin
                        if (host_rx_sop && host_rx_valid) begin
                            state <= ST_HOST_TO_DEVICE;
                            latched_host_pid <= host_rx_pid;
                            packet_dir <= DIR_HOST_TO_DEVICE;
                            host_packets <= host_packets + 1;
                            buffer_timestamp <= timestamp;
                            if (host_rx_pid == PID_SOF) begin
                                state <= ST_HANDLE_SOF;
                            end else if (packet_filter_en && packet_filter_mask[host_rx_pid]) begin
                                state <= ST_FILTER_PACKET;
                            end
                        end
                        else if (device_rx_sop && device_rx_valid) begin
                            state <= ST_DEVICE_TO_HOST;
                            latched_device_pid <= device_rx_pid;
                            packet_dir <= DIR_DEVICE_TO_HOST;
                            device_packets <= device_packets + 1;
                            buffer_timestamp <= timestamp;
                            if (packet_filter_en && packet_filter_mask[device_rx_pid]) begin
                                state <= ST_FILTER_PACKET;
                            end
                        end
                    end
                    
                    // Connection status tracking
                    device_connected <= (device_line_state != 2'b00);
                end

                ST_HOST_TO_DEVICE: begin
                    if (host_rx_valid) begin
                        if (buffer_ready && !buffer_valid) begin
                            buffer_data <= host_rx_data;
                            buffer_valid <= 1'b1;
                            buffer_flags <= {latched_host_pid, 3'b0, DIR_HOST_TO_DEVICE};
                        end
                        byte_counter <= (byte_counter < 255) ? byte_counter + 1 : 255;
                    end
                    
                    if (host_rx_eop) begin
                        state <= ST_WAIT_DEVICE_RESP;
                        byte_counter <= 0;
                    end
                end

                ST_WAIT_DEVICE_RESP: begin
                    if (device_rx_sop && device_rx_valid) begin
                        state <= ST_DEVICE_TO_HOST;
                        device_packets <= device_packets + 1;
                        // Update data toggle if needed
                        if ((latched_host_pid == PID_OUT || latched_host_pid == PID_SETUP) && 
                            device_rx_pid == PID_ACK && index_valid) begin
                            // Initiate 2-cycle pulse for clock domain crossing
                            pid_ack_120 <= 1'b1;
                            pid_ack_counter <= 2'b11;
                        
                            data_toggle[endpoint_index] <= ~data_toggle[endpoint_index];
                        end
                    end else begin
                        // Decrement counter and clear pulse
                        if (pid_ack_counter != 2'b00) begin
                            pid_ack_counter <= pid_ack_counter - 1'b1;
                            pid_ack_120 <= (pid_ack_counter != 2'b01);
                        end
                        else begin
                            pid_ack_counter <= 2'b00;
                            pid_ack_120 <= 1'b0;
                        end
                        if (!host_rx_valid && !device_rx_valid) begin
                            state <= ST_IDLE;
                        end
                    end
                end

                ST_DEVICE_TO_HOST: begin
                    if (device_rx_valid) begin
                        if (buffer_ready && !buffer_valid) begin
                            buffer_data <= device_rx_data;
                            buffer_valid <= 1'b1;
                            buffer_flags <= {latched_device_pid, 3'b0, DIR_DEVICE_TO_HOST};
                        end
                        byte_counter <= (byte_counter < 255) ? byte_counter + 1 : 255;
                    end
                    
                    if (device_rx_eop) begin
                        state <= ST_WAIT_HOST_RESP;
                        byte_counter <= 0;
                        // Update data toggle if needed
                        if ((latched_device_pid == PID_IN) && index_valid) begin
                            data_toggle[endpoint_index] <= ~data_toggle[endpoint_index];
                        end
                    end
                end

                ST_WAIT_HOST_RESP: begin
                    if (host_rx_sop || (byte_counter > 200)) begin
                        state <= ST_IDLE;
                    end else begin
                        byte_counter <= byte_counter + 1;
                    end
                end

                ST_FILTER_PACKET: begin
                    if (packet_dir == DIR_HOST_TO_DEVICE) begin
                        if (host_rx_valid && buffer_ready) begin
                            monitor_packet_buffer[byte_counter] <= 
                                (byte_counter == 1 && addr_translate_en) ? 
                                {host_rx_data[7], (host_rx_data[6:0] == addr_translate_from) ? 
                                 addr_translate_to : host_rx_data[6:0]} : 
                                host_rx_data;
                            byte_counter <= (byte_counter < 255) ? byte_counter + 1 : 255;
                        end
                        if (host_rx_eop) begin
                            packet_length <= byte_counter + 1;
                            byte_counter <= 0;
                            state <= modify_enable ? ST_MODIFY_PACKET : ST_BUFFER_PACKET;
                        end
                    end else begin
                        if (device_rx_valid && buffer_ready) begin
                            monitor_packet_buffer[byte_counter] <= device_rx_data;
                            byte_counter <= (byte_counter < 255) ? byte_counter + 1 : 255;
                        end
                        if (device_rx_eop) begin
                            packet_length <= byte_counter + 1;
                            byte_counter <= 0;
                            state <= modify_enable ? ST_MODIFY_PACKET : ST_BUFFER_PACKET;
                        end
                    end
                end

                ST_MODIFY_PACKET: begin
                    packet_modified <= 1'b1;
                    state <= ST_BUFFER_PACKET;
                end

                ST_BUFFER_PACKET: begin
                    if (buffer_ready) begin
                        buffer_data <= monitor_packet_buffer[byte_counter];
                        buffer_valid <= 1'b1;
                        buffer_flags <= {latched_host_pid, 3'b0, packet_dir};
                        byte_counter <= byte_counter + 1;
                        if (byte_counter == packet_length - 1) begin
                            state <= (packet_dir == DIR_HOST_TO_DEVICE) ? 
                                   ST_HOST_TO_DEVICE : ST_DEVICE_TO_HOST;
                            byte_counter <= 0;
                        end
                    end
                end

                ST_HANDLE_SOF: begin
                    if (host_rx_valid) begin
                        case (byte_counter)
                            1: last_frame_num[7:0] <= host_rx_data;
                            2: last_frame_num[15:8] <= host_rx_data;
                        endcase
                        byte_counter <= byte_counter + 1;
                    end
                    if (host_rx_eop) state <= ST_IDLE;
                end

                default: state <= ST_IDLE;
            endcase

            // Error detection
            if ((host_rx_valid && !host_rx_crc_valid && host_rx_eop) ||
                (device_rx_valid && !device_rx_crc_valid && device_rx_eop)) begin
                error_count <= error_count + 1;
                status_register[STAT_ERROR] <= 1'b1;
            end
        end
    end

    // Status register management
    always @(posedge clk) begin
        status_register[STAT_CONNECTED] <= device_connected;
        status_register[STAT_PROXY_EN] <= proxy_enable;
        status_register[STAT_FILTER_EN] <= packet_filter_en;
        status_register[STAT_MODIFY_EN] <= modify_enable;
        {status_register[STAT_SPEED_HIGH], status_register[STAT_SPEED_FULL]} <= device_speed;
    end

    // Speed detection
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            device_speed <= 2'b00;
        end else if (event_valid) begin
            case (event_type)
                8'h01: device_speed <= 2'b01;  // Full-speed
                8'h02: device_speed <= 2'b10;  // High-speed
                8'h03: device_speed <= 2'b00;  // Low-speed
                default: ; // No change
            endcase
        end
    end

    // Overflow detection
    always @(posedge clk) begin
        overflow_detected <= (byte_counter == 255) && (state != ST_IDLE);
    end

    // Status readback
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
