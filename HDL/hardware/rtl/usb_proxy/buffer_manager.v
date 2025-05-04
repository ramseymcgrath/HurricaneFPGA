///////////////////////////////////////////////////////////////////////////////
// File: buffer_manager.v
// Description: Ring Buffer Implementation (32KB BRAM)
//
// This module implements a ring buffer in BRAM for storing USB packets with
// timestamps. It provides 32KB total (16KB per direction) and handles flow control.
//
// Target: Lattice ECP5 on Cynthion device
///////////////////////////////////////////////////////////////////////////////

module buffer_manager (
    // Clock and Reset
    input  wire        clk,             // System clock
    input  wire        rst_n,           // Active low reset
    
    // Write Interface
    input  wire [7:0]  write_data,      // Data to write
    input  wire        write_valid,     // Write data valid
    input  wire [63:0] write_timestamp, // Timestamp for current packet
    input  wire [7:0]  write_flags,     // Packet flags (direction, type, etc)
    output wire        write_ready,     // Ready to accept data
    
    // Read Interface
    output reg  [7:0]  read_data,       // Read data
    output reg         read_valid,      // Read data valid
    input  wire        read_req,        // Request to read data
    output reg  [63:0] read_timestamp,  // Timestamp for current packet
    output reg  [7:0]  read_flags,      // Packet flags
    output wire        read_packet_start,// Indicates start of a packet
    output wire        read_packet_end, // Indicates end of a packet
    
    // Control Interface
    input  wire        buffer_clear,    // Clear buffer command
    input  wire [15:0] high_watermark,  // High watermark level
    input  wire [15:0] low_watermark,   // Low watermark level
    
    // Status Interface
    output reg  [15:0] buffer_used,     // Used buffer space
    output reg  [15:0] buffer_free,     // Free buffer space
    output reg         buffer_empty,    // Buffer empty flag
    output reg         buffer_full,     // Buffer full flag
    output reg         buffer_overflow, // Buffer overflow occurred
    output reg         buffer_underflow,// Buffer underflow occurred
    output reg  [31:0] packet_count,    // Number of complete packets in buffer
    
    // Configuration
    input  wire        enable_overflow_protection, // Prevent overflow by flow control
    input  wire [1:0]  buffer_mode      // 00=single, 01=dual direction, 10=priority
);

    // Constants
    localparam BUFFER_SIZE = 32768;     // 32KB total buffer size
    localparam BUFFER_SIZE_PER_DIR = 16384; // 16KB per direction
    localparam ADDR_WIDTH = 15;         // Address width (32K = 2^15)
    
    // Packet state constants
    localparam PKT_IDLE       = 2'b00;
    localparam PKT_HEADER     = 2'b01;
    localparam PKT_DATA       = 2'b10;
    localparam PKT_COMPLETE   = 2'b11;
    
    // Header structure in buffer:
    // [0-7]   - Magic byte (0xA5)
    // [8-15]  - Flags
    // [16-31] - Packet length (16-bit)
    // [32-95] - Timestamp (64-bit)
    // [96+]   - Packet data
    localparam HEADER_SIZE = 12;        // 12 bytes header (magic, flags, length, timestamp)
    localparam MAGIC_BYTE = 8'hA5;      // Magic byte value
    
    // Buffer memory (implemented as dual-port BRAM)
    // Split into two separate memories with explicit RAM synthesis attributes
    (* ram_style = "block" *) reg [7:0] buffer_mem_host [0:BUFFER_SIZE_PER_DIR-1];
    (* ram_style = "block" *) reg [7:0] buffer_mem_device [0:BUFFER_SIZE_PER_DIR-1];
    
    // Write control
    reg [ADDR_WIDTH-1:0] write_ptr;     // Write pointer
    reg [ADDR_WIDTH-1:0] write_ptr_host;// Write pointer for host direction
    reg [ADDR_WIDTH-1:0] write_ptr_dev; // Write pointer for device direction
    reg [15:0] write_length;            // Current packet write length
    reg [1:0]  write_state;             // Packet write state
    reg [3:0]  write_header_idx;        // Header byte index for writing
    
    // Read control
    reg [ADDR_WIDTH-1:0] read_ptr;      // Read pointer
    reg [ADDR_WIDTH-1:0] read_ptr_host; // Read pointer for host direction
    reg [ADDR_WIDTH-1:0] read_ptr_dev;  // Read pointer for device direction
    reg [15:0] read_length;             // Current packet read length
    reg [15:0] read_remaining;          // Remaining bytes for current packet
    reg [1:0]  read_state;              // Packet read state
    reg [3:0]  read_header_idx;         // Header byte index for reading
    reg        read_direction;          // Current read direction
    reg [15:0] packet_length;           // Length of current packet being read
    
    // Status tracking
    reg [15:0] buffer_used_host;        // Used space in host buffer
    reg [15:0] buffer_used_dev;         // Used space in device buffer
    reg [31:0] packet_count_host;       // Packets in host buffer
    reg [31:0] packet_count_dev;        // Packets in device buffer
    reg        buffer_empty_host;       // Host buffer empty
    reg        buffer_empty_dev;        // Device buffer empty
    reg        buffer_full_host;        // Host buffer full
    reg        buffer_full_dev;         // Device buffer full
    
    // Error and flow control
    reg        flow_control_active;     // Flow control is currently active
    
    // Start/end indicators
    reg        packet_start;            // Current byte is start of packet
    reg        packet_end;              // Current byte is end of packet
    
    // Assign output signals
    assign write_ready = !buffer_full && !flow_control_active;
    assign read_packet_start = packet_start;
    assign read_packet_end = packet_end;
    
    // Determine current buffer status based on mode
    always @(*) begin
        case (buffer_mode)
            2'b00: begin // Single buffer mode
                buffer_used = buffer_used_host + buffer_used_dev;
                buffer_free = BUFFER_SIZE - buffer_used;
                buffer_empty = (buffer_used == 0);
                buffer_full = (buffer_used >= (BUFFER_SIZE - HEADER_SIZE - 256)); // Leave space for headers + max packet
                packet_count = packet_count_host + packet_count_dev;
            end
            
            2'b01: begin // Dual direction mode
                // Select based on read_direction
                if (read_direction == 0) begin // Host direction
                    buffer_used = buffer_used_host;
                    buffer_free = BUFFER_SIZE_PER_DIR - buffer_used_host;
                    buffer_empty = buffer_empty_host;
                    buffer_full = buffer_full_host;
                    packet_count = packet_count_host;
                end else begin // Device direction
                    buffer_used = buffer_used_dev;
                    buffer_free = BUFFER_SIZE_PER_DIR - buffer_used_dev;
                    buffer_empty = buffer_empty_dev;
                    buffer_full = buffer_full_dev;
                    packet_count = packet_count_dev;
                end
            end
            
            default: begin // Priority mode - same as single buffer for status
                buffer_used = buffer_used_host + buffer_used_dev;
                buffer_free = BUFFER_SIZE - buffer_used;
                buffer_empty = (buffer_used == 0);
                buffer_full = (buffer_used >= (BUFFER_SIZE - HEADER_SIZE - 256));
                packet_count = packet_count_host + packet_count_dev;
            end
        endcase
    end

    // Write logic - handles storing packets with headers and timestamps
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            write_ptr <= {ADDR_WIDTH{1'b0}};
            write_ptr_host <= {ADDR_WIDTH{1'b0}};
            write_ptr_dev <= BUFFER_SIZE_PER_DIR; // Second half of buffer
            write_length <= 16'd0;
            write_state <= PKT_IDLE;
            write_header_idx <= 4'd0;
            buffer_used_host <= 16'd0;
            buffer_used_dev <= 16'd0;
            packet_count_host <= 32'd0;
            packet_count_dev <= 32'd0;
            buffer_empty_host <= 1'b1;
            buffer_empty_dev <= 1'b1;
            buffer_full_host <= 1'b0;
            buffer_full_dev <= 1'b0;
            buffer_overflow <= 1'b0;
            flow_control_active <= 1'b0;
        end else begin
            // Clear buffer if requested
            if (buffer_clear) begin
                write_ptr <= {ADDR_WIDTH{1'b0}};
                write_ptr_host <= {ADDR_WIDTH{1'b0}};
                write_ptr_dev <= BUFFER_SIZE_PER_DIR; // Second half of buffer
                write_length <= 16'd0;
                write_state <= PKT_IDLE;
                write_header_idx <= 4'd0;
                buffer_used_host <= 16'd0;
                buffer_used_dev <= 16'd0;
                packet_count_host <= 32'd0;
                packet_count_dev <= 32'd0;
                buffer_empty_host <= 1'b1;
                buffer_empty_dev <= 1'b1;
                buffer_full_host <= 1'b0;
                buffer_full_dev <= 1'b0;
                buffer_overflow <= 1'b0;
                flow_control_active <= 1'b0;
            end else begin
                // Flow control management
                if (enable_overflow_protection) begin
                    // Activate flow control at high watermark
                    if (buffer_used >= high_watermark && !flow_control_active) begin
                        flow_control_active <= 1'b1;
                    end
                    // Deactivate flow control at low watermark
                    else if (buffer_used <= low_watermark && flow_control_active) begin
                        flow_control_active <= 1'b0;
                    end
                end
                
                // Write processing
                case (write_state)
                    PKT_IDLE: begin
                        if (write_valid && write_ready) begin
                            // New packet - write header first
                            write_state <= PKT_HEADER;
                            write_header_idx <= 4'd0;
                            write_length <= 16'd0;
                            
                            // Choose buffer based on direction flag
                            case (buffer_mode)
                                2'b00: begin // Single buffer mode
                                    write_ptr <= write_ptr;
                                end
                                
                                2'b01: begin // Dual direction mode
                                    if (write_flags[0] == 0) begin // Host direction
                                        write_ptr <= write_ptr_host;
                                    end else begin // Device direction
                                        write_ptr <= write_ptr_dev;
                                    end
                                end
                                
                                default: begin // Priority mode - same as single buffer
                                    write_ptr <= write_ptr;
                                end
                            endcase
                        end
                    end
                    
                    PKT_HEADER: begin
                        if (write_header_idx < HEADER_SIZE) begin
                            // Write header bytes
                            case (write_header_idx)
                                4'd0: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= MAGIC_BYTE;
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= MAGIC_BYTE;
                                end
                                4'd1: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_flags;
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_flags;
                                end
                                4'd2: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_length[7:0];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_length[7:0];
                                end
                                4'd3: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_length[15:8];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_length[15:8];
                                end
                                4'd4: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[7:0];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[7:0];
                                end
                                4'd5: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[15:8];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[15:8];
                                end
                                4'd6: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[23:16];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[23:16];
                                end
                                4'd7: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[31:24];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[31:24];
                                end
                                4'd8: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[39:32];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[39:32];
                                end
                                4'd9: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[47:40];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[47:40];
                                end
                                4'd10: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[55:48];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[55:48];
                                end
                                4'd11: begin
                                    if (write_flags[0] == 0) // Host direction
                                        buffer_mem_host[write_ptr] <= write_timestamp[63:56];
                                    else // Device direction
                                        buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_timestamp[63:56];
                                end
                            endcase
                            
                            // Update pointers and counters
                            write_ptr <= write_ptr + 1'b1;
                            write_header_idx <= write_header_idx + 1'b1;
                            
                            // Update used space based on direction
                            if (buffer_mode == 2'b01) begin
                                if (write_flags[0] == 0) begin // Host direction
                                    buffer_used_host <= buffer_used_host + 1'b1;
                                    buffer_empty_host <= 1'b0;
                                    buffer_full_host <= (buffer_used_host >= (BUFFER_SIZE_PER_DIR - HEADER_SIZE - 256));
                                end else begin // Device direction
                                    buffer_used_dev <= buffer_used_dev + 1'b1;
                                    buffer_empty_dev <= 1'b0;
                                    buffer_full_dev <= (buffer_used_dev >= (BUFFER_SIZE_PER_DIR - HEADER_SIZE - 256));
                                end
                            end
                        end else begin
                            // Header complete, move to data state
                            write_state <= PKT_DATA;
                            
                            // Write first data byte if available
                            if (write_valid) begin
                                // Select appropriate buffer based on direction
                                if (write_flags[0] == 0) // Host direction
                                    buffer_mem_host[write_ptr] <= write_data;
                                else // Device direction
                                    buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_data;
                                write_ptr <= write_ptr + 1'b1;
                                write_length <= write_length + 1'b1;
                                
                                // Update used space based on direction
                                if (buffer_mode == 2'b01) begin
                                    if (write_flags[0] == 0) begin // Host direction
                                        buffer_used_host <= buffer_used_host + 1'b1;
                                    end else begin // Device direction
                                        buffer_used_dev <= buffer_used_dev + 1'b1;
                                    end
                                end
                            end
                        end
                    end
                    
                    PKT_DATA: begin
                        if (write_valid) begin
                            // Check for buffer space
                            if (write_ready) begin
                                // Select appropriate buffer based on direction
                                if (write_flags[0] == 0) // Host direction
                                    buffer_mem_host[write_ptr] <= write_data;
                                else // Device direction
                                    buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR] <= write_data;
                                write_ptr <= write_ptr + 1'b1;
                                write_length <= write_length + 1'b1;
                                
                                // Update used space based on direction
                                if (buffer_mode == 2'b01) begin
                                    if (write_flags[0] == 0) begin // Host direction
                                        buffer_used_host <= buffer_used_host + 1'b1;
                                    end else begin // Device direction
                                        buffer_used_dev <= buffer_used_dev + 1'b1;
                                    end
                                end
                            end else begin
                                // Buffer full condition
                                buffer_overflow <= 1'b1;
                                write_state <= PKT_IDLE;
                            end
                        end else begin
                            // No more data - complete packet
                            // Update the length field in the header
                            // Update length in the appropriate buffer
                            if (write_flags[0] == 0) begin // Host direction
                                buffer_mem_host[write_ptr - write_length - HEADER_SIZE + 2] <= write_length[7:0];
                                buffer_mem_host[write_ptr - write_length - HEADER_SIZE + 3] <= write_length[15:8];
                            end else begin // Device direction
                                buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR - write_length - HEADER_SIZE + 2] <= write_length[7:0];
                                buffer_mem_device[write_ptr - BUFFER_SIZE_PER_DIR - write_length - HEADER_SIZE + 3] <= write_length[15:8];
                            end
                            
                            // Update packet counters
                            if (buffer_mode == 2'b01) begin
                                if (write_flags[0] == 0) begin // Host direction
                                    packet_count_host <= packet_count_host + 1'b1;
                                    write_ptr_host <= write_ptr;
                                end else begin // Device direction
                                    packet_count_dev <= packet_count_dev + 1'b1;
                                    write_ptr_dev <= write_ptr;
                                end
                            end else begin
                                packet_count_host <= packet_count_host + ((write_flags[0] == 0) ? 1'b1 : 1'b0);
                                packet_count_dev <= packet_count_dev + ((write_flags[0] != 0) ? 1'b1 : 1'b0);
                            end
                            
                            write_state <= PKT_IDLE;
                        end
                    end
                    
                    default: write_state <= PKT_IDLE;
                endcase
            end
        end
    end
    
    // Read logic - handles retrieving packets with headers and timestamps
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            read_ptr <= {ADDR_WIDTH{1'b0}};
            read_ptr_host <= {ADDR_WIDTH{1'b0}};
            read_ptr_dev <= BUFFER_SIZE_PER_DIR; // Second half of buffer
            read_length <= 16'd0;
            read_remaining <= 16'd0;
            read_state <= PKT_IDLE;
            read_header_idx <= 4'd0;
            read_data <= 8'd0;
            read_valid <= 1'b0;
            read_timestamp <= 64'd0;
            read_flags <= 8'd0;
            read_direction <= 1'b0; // Start with host direction
            packet_length <= 16'd0;
            packet_start <= 1'b0;
            packet_end <= 1'b0;
            buffer_underflow <= 1'b0;
        end else begin
            // Default values
            read_valid <= 1'b0;
            packet_start <= 1'b0;
            packet_end <= 1'b0;
            
            // Clear buffer if requested
            if (buffer_clear) begin
                read_ptr <= {ADDR_WIDTH{1'b0}};
                read_ptr_host <= {ADDR_WIDTH{1'b0}};
                read_ptr_dev <= BUFFER_SIZE_PER_DIR; // Second half of buffer
                read_length <= 16'd0;
                read_remaining <= 16'd0;
                read_state <= PKT_IDLE;
                read_header_idx <= 4'd0;
                read_direction <= 1'b0;
                packet_length <= 16'd0;
            end else begin
                case (read_state)
                    PKT_IDLE: begin
                        if (read_req && !buffer_empty) begin
                            // Start reading a packet
                            read_state <= PKT_HEADER;
                            read_header_idx <= 4'd0;
                            
                            // Choose buffer based on mode and availability
                            case (buffer_mode)
                                2'b00: begin // Single buffer mode
                                    read_ptr <= read_ptr;
                                end
                                
                                2'b01: begin // Dual direction mode
                                    // Alternate between host and device packets
                                    if (read_direction == 0) begin
                                        if (!buffer_empty_host) begin
                                            read_ptr <= read_ptr_host;
                                        end else if (!buffer_empty_dev) begin
                                            read_ptr <= read_ptr_dev;
                                            read_direction <= 1'b1;
                                        end
                                    end else begin
                                        if (!buffer_empty_dev) begin
                                            read_ptr <= read_ptr_dev;
                                        end else if (!buffer_empty_host) begin
                                            read_ptr <= read_ptr_host;
                                            read_direction <= 1'b0;
                                        end
                                    end
                                end
                                
                                2'b10: begin // Priority mode - prefer host direction
                                    if (!buffer_empty_host) begin
                                        read_ptr <= read_ptr_host;
                                        read_direction <= 1'b0;
                                    end else if (!buffer_empty_dev) begin
                                        read_ptr <= read_ptr_dev;
                                        read_direction <= 1'b1;
                                    end
                                end
                                
                                default: begin // Default to single buffer
                                    read_ptr <= read_ptr;
                                end
                            endcase
                        end
                    end
                    
                    PKT_HEADER: begin
                        // Read header bytes
                        if (read_header_idx < HEADER_SIZE) begin
                            case (read_header_idx)
                                4'd0: begin // Magic byte
                                    // Check magic byte in appropriate buffer
                                    if ((read_direction == 0 && buffer_mem_host[read_ptr] == MAGIC_BYTE) ||
                                        (read_direction == 1 && buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR] == MAGIC_BYTE)) begin
                                        read_ptr <= read_ptr + 1'b1;
                                        read_header_idx <= read_header_idx + 1'b1;
                                    end else begin
                                        // Invalid magic - buffer corruption
                                        read_state <= PKT_IDLE;
                                        buffer_underflow <= 1'b1;
                                    end
                                end
                                
                                4'd1: begin // Flags
                                    // Read flags from appropriate buffer
                                    if (read_direction == 0)
                                        read_flags <= buffer_mem_host[read_ptr];
                                    else
                                        read_flags <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd2: begin // Length low byte
                                    // Read length low byte from appropriate buffer
                                    if (read_direction == 0)
                                        packet_length[7:0] <= buffer_mem_host[read_ptr];
                                    else
                                        packet_length[7:0] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd3: begin // Length high byte
                                    // Read length high byte from appropriate buffer
                                    if (read_direction == 0) begin
                                        packet_length[15:8] <= buffer_mem_host[read_ptr];
                                        read_remaining <= {buffer_mem_host[read_ptr], packet_length[7:0]};
                                    end else begin
                                        packet_length[15:8] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                        read_remaining <= {buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR], packet_length[7:0]};
                                    end
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd4: begin // Timestamp bytes
                                    // Read timestamp bytes from appropriate buffer
                                    if (read_direction == 0)
                                        read_timestamp[7:0] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[7:0] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd5: begin
                                    if (read_direction == 0)
                                        read_timestamp[15:8] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[15:8] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd6: begin
                                    if (read_direction == 0)
                                        read_timestamp[23:16] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[23:16] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd7: begin
                                    if (read_direction == 0)
                                        read_timestamp[31:24] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[31:24] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd8: begin
                                    if (read_direction == 0)
                                        read_timestamp[39:32] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[39:32] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd9: begin
                                    if (read_direction == 0)
                                        read_timestamp[47:40] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[47:40] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd10: begin
                                    if (read_direction == 0)
                                        read_timestamp[55:48] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[55:48] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                end
                                
                                4'd11: begin
                                    if (read_direction == 0)
                                        read_timestamp[63:56] <= buffer_mem_host[read_ptr];
                                    else
                                        read_timestamp[63:56] <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                                    read_ptr <= read_ptr + 1'b1;
                                    read_header_idx <= read_header_idx + 1'b1;
                                    read_state <= PKT_DATA;
                                end
                                
                                default: read_header_idx <= 4'd0;
                            endcase
                            
                            // Update used space based on direction
                            if (buffer_mode == 2'b01) begin
                                if (read_direction == 0) begin // Host direction
                                    buffer_used_host <= buffer_used_host - 1'b1;
                                end else begin // Device direction
                                    buffer_used_dev <= buffer_used_dev - 1'b1;
                                end
                            end
                        end
                    end
                    
                    PKT_DATA: begin
                        if (read_req && read_remaining > 0) begin
                            // Output packet data bytes
                            // Read data from appropriate buffer
                            if (read_direction == 0)
                                read_data <= buffer_mem_host[read_ptr];
                            else
                                read_data <= buffer_mem_device[read_ptr - BUFFER_SIZE_PER_DIR];
                            read_valid <= 1'b1;
                            read_ptr <= read_ptr + 1'b1;
                            read_remaining <= read_remaining - 1'b1;
                            
                            // Signal packet boundaries
                            if (read_remaining == packet_length) packet_start <= 1'b1;
                            if (read_remaining == 1) packet_end <= 1'b1;
                            
                            // Update used space based on direction
                            if (buffer_mode == 2'b01) begin
                                if (read_direction == 0) begin // Host direction
                                    buffer_used_host <= buffer_used_host - 1'b1;
                                end else begin // Device direction
                                    buffer_used_dev <= buffer_used_dev - 1'b1;
                                end
                            end
                            
                            // Check for empty buffer condition
                            if (buffer_mode == 2'b01) begin
                                if (read_direction == 0) begin // Host direction
                                    buffer_empty_host <= (buffer_used_host <= 1);
                                    buffer_full_host <= 1'b0;
                                end else begin // Device direction
                                    buffer_empty_dev <= (buffer_used_dev <= 1);
                                    buffer_full_dev <= 1'b0;
                                end
                            end
                        end
                        
                        if (read_remaining == 0) begin
                            // Packet complete
                            read_state <= PKT_IDLE;
                            
                            // Update packet counters
                            if (buffer_mode == 2'b01) begin
                                if (read_direction == 0) begin // Host direction
                                    packet_count_host <= packet_count_host - 1'b1;
                                    read_ptr_host <= read_ptr;
                                end else begin // Device direction
                                    packet_count_dev <= packet_count_dev - 1'b1;
                                    read_ptr_dev <= read_ptr;
                                end
                            end else begin
                                if (read_direction == 0) begin
                                    packet_count_host <= packet_count_host - 1'b1;
                                end else begin
                                    packet_count_dev <= packet_count_dev - 1'b1;
                                end
                            end
                            
                            // Toggle direction for fair reading in dual mode
                            if (buffer_mode == 2'b01) begin
                                read_direction <= ~read_direction;
                            end
                        end
                    end
                    
                    default: read_state <= PKT_IDLE;
                endcase
            end
        end
    end

endmodule