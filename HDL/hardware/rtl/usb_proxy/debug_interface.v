///////////////////////////////////////////////////////////////////////////////
// File: debug_interface.v
// Description: Basic debugging interface for Cynthion USB sniffer
//
// This module provides a simple debugging interface to monitor the internal
// state of the USB sniffer and expose diagnostic information.
//
// Target: Lattice ECP5 on Cynthion device
///////////////////////////////////////////////////////////////////////////////

module debug_interface (
    // Clock and Reset
    input  wire        clk,               // System clock
    input  wire        rst_n,             // Active low reset
    
    // Debug Control Interface
    input  wire [7:0]  debug_cmd,         // Debug command input
    input  wire        debug_cmd_valid,   // Debug command valid
    output reg  [7:0]  debug_resp,        // Debug response output
    output reg         debug_resp_valid,  // Debug response valid
    
    // Status Inputs
    input  wire        proxy_active,      // Proxy is active
    input  wire        host_connected,    // Host connection status
    input  wire        device_connected,  // Device connection status
    input  wire [1:0]  host_speed,        // Host connection speed
    input  wire [1:0]  device_speed,      // Device connection speed
    input  wire        buffer_overflow,   // Buffer overflow indicator
    input  wire [15:0] buffer_used,       // Current buffer usage
    input  wire [31:0] packet_count,      // Total packet count
    input  wire [15:0] error_count,       // Error counter
    
    // Monitor Inputs
    input  wire [1:0]  host_line_state,   // Host USB line state
    input  wire [1:0]  device_line_state, // Device USB line state
    input  wire [63:0] timestamp,         // Current timestamp
    
    // Debug Outputs
    output reg  [7:0]  debug_leds,        // Debug LED outputs
    output reg  [7:0]  debug_probe,       // Debug probe outputs for logic analyzer
    
    // Configuration Control
    output reg         force_reset,       // Force system reset
    output reg  [1:0]  debug_mode,        // Debug mode selection
    output reg  [7:0]  trigger_config,    // Trigger configuration
    output reg         loopback_enable    // Enable loopback mode for testing
);

    // Command definitions
    localparam CMD_NOP                = 8'h00;  // No operation
    localparam CMD_GET_STATUS         = 8'h01;  // Get system status
    localparam CMD_GET_BUFFER_STATUS  = 8'h02;  // Get buffer status
    localparam CMD_GET_PACKET_COUNT   = 8'h03;  // Get packet count
    localparam CMD_GET_ERROR_COUNT    = 8'h04;  // Get error count
    localparam CMD_GET_LINE_STATE     = 8'h05;  // Get current line state
    localparam CMD_GET_TIMESTAMP      = 8'h06;  // Get current timestamp
    localparam CMD_SET_DEBUG_LEDS     = 8'h10;  // Set debug LEDs
    localparam CMD_SET_DEBUG_PROBE    = 8'h11;  // Set debug probe outputs
    localparam CMD_SET_DEBUG_MODE     = 8'h12;  // Set debug mode
    localparam CMD_FORCE_RESET        = 8'h20;  // Force system reset
    localparam CMD_LOOPBACK_ENABLE    = 8'h21;  // Enable loopback mode
    localparam CMD_TRIGGER_CONFIG     = 8'h22;  // Configure trigger
    localparam CMD_VERSION            = 8'hF0;  // Get version information
    
    // Response buffer (dual-port RAM implementation)
    (* ram_style = "block", mem_init = "0" *) reg [7:0] response_buffer [0:15];
    reg [3:0] response_length;
    reg [3:0] response_index;
    reg       sending_response;
    reg       buffer_write_en;
    reg [3:0] write_address;
    reg [7:0] write_data;
    
    // Version information
    localparam VERSION_MAJOR = 8'h01;
    localparam VERSION_MINOR = 8'h00;
    localparam VERSION_PATCH = 8'h00;
    
    // Command processing
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debug_resp <= 8'h00;
            debug_resp_valid <= 1'b0;
            debug_leds <= 8'h00;
            debug_probe <= 8'h00;
            debug_mode <= 2'b00;
            trigger_config <= 8'h00;
            force_reset <= 1'b0;
            loopback_enable <= 1'b0;
            response_length <= 4'h0;
            response_index <= 4'h0;
            sending_response <= 1'b0;
        end else begin
            // Default values
            debug_resp_valid <= 1'b0;
            force_reset <= 1'b0;  // Auto-clear reset signal
            
            // Command processing
            // Buffer write control
            buffer_write_en <= 1'b0;
            if (debug_cmd_valid) begin
               response_index <= 4'h0;
               sending_response <= 1'b1;
               write_address <= 4'h0;
               
               case (debug_cmd)
                   CMD_NOP: begin
                       // No operation
                       response_length <= 4'h1;
                       write_data <= CMD_NOP;
                       buffer_write_en <= 1'b1;
                       response_buffer[0] <= write_data;  // Explicit single write
                    end
                    
                    CMD_GET_STATUS: begin
                        // Return system status - single atomic write
                        response_length <= 4'h4;
                        write_data <= CMD_GET_STATUS;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= {
                            write_data,
                            {4'b0000, proxy_active, host_connected, device_connected, 1'b0},
                            {4'b0000, host_speed, device_speed},
                            {7'b0000000, buffer_overflow}
                        };
                        write_address <= write_address + 1;  // Single address increment
                    end
                    
                    CMD_GET_BUFFER_STATUS: begin
                        // Return buffer status
                        response_length <= 4'h3;
                        write_data <= CMD_GET_BUFFER_STATUS;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= buffer_used[7:0];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= buffer_used[15:8];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_GET_PACKET_COUNT: begin
                        // Return packet count
                        response_length <= 4'h5;
                        write_data <= CMD_GET_PACKET_COUNT;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= packet_count[7:0];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= packet_count[15:8];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= packet_count[23:16];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= packet_count[31:24];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_GET_ERROR_COUNT: begin
                        // Return error count
                        response_length <= 4'h3;
                        write_data <= CMD_GET_ERROR_COUNT;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= error_count[7:0];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= error_count[15:8];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_GET_LINE_STATE: begin
                        // Return current line state
                        response_length <= 4'h2;
                        write_data <= CMD_GET_LINE_STATE;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= {4'b0000, device_line_state, host_line_state};
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_GET_TIMESTAMP: begin
                        // Return current timestamp (lower 32 bits)
                        response_length <= 4'h5;
                        write_data <= CMD_GET_TIMESTAMP;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= timestamp[7:0];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= timestamp[15:8];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= timestamp[23:16];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= timestamp[31:24];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_SET_DEBUG_LEDS: begin
                        // Set debug LEDs
                        debug_leds <= debug_cmd[7:0];
                        response_length <= 4'h2;
                        write_data <= CMD_SET_DEBUG_LEDS;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= debug_cmd[7:0];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_SET_DEBUG_PROBE: begin
                        // Set debug probe outputs
                        debug_probe <= debug_cmd[7:0];
                        response_length <= 4'h2;
                        write_data <= CMD_SET_DEBUG_PROBE;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= debug_cmd[7:0];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_SET_DEBUG_MODE: begin
                        // Set debug mode
                        debug_mode <= debug_cmd[1:0];
                        response_length <= 4'h2;
                        write_data <= CMD_SET_DEBUG_MODE;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= {6'b000000, debug_cmd[1:0]};
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_FORCE_RESET: begin
                        // Force system reset
                        force_reset <= 1'b1;
                        response_length <= 4'h1;
                        response_buffer[0] <= CMD_FORCE_RESET;
                    end
                    
                    CMD_LOOPBACK_ENABLE: begin
                        // Enable/disable loopback mode
                        loopback_enable <= debug_cmd[0];
                        response_length <= 4'h2;
                        write_data <= CMD_LOOPBACK_ENABLE;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= {7'b0000000, debug_cmd[0]};
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_TRIGGER_CONFIG: begin
                        // Configure trigger
                        trigger_config <= debug_cmd[7:0];
                        response_length <= 4'h2;
                        write_data <= CMD_TRIGGER_CONFIG;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= debug_cmd[7:0];
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    CMD_VERSION: begin
                        // Return version information
                        response_length <= 4'h4;
                        write_data <= CMD_VERSION;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= VERSION_MAJOR;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= VERSION_MINOR;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= VERSION_PATCH;
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                    
                    default: begin
                        // Unknown command
                        response_length <= 4'h2;
                        write_data <= 8'hFF;  // Error response
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                        
                        write_data <= debug_cmd;  // Echo unknown command
                        buffer_write_en <= 1'b1;
                        response_buffer[write_address] <= write_data;
                        write_address <= write_address + 1;
                    end
                endcase
            end
            
            
            // Response handling with registered output
            if (sending_response) begin
                if (response_index < response_length) begin
                    // Send next byte of response with pipeline register
                    debug_resp <= response_buffer[response_index];
                    debug_resp_valid <= 1'b1;
                    response_index <= response_index + 1'b1;
                end else begin
                    // End of response
                    sending_response <= 1'b0;
                    debug_resp_valid <= 1'b0;
                end
            end
        end
    end


endmodule