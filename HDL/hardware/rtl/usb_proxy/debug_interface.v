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
    
    // Response buffer
    reg [7:0] response_buffer [15:0];
    reg [3:0] response_length;
    reg [3:0] response_index;
    reg       sending_response;
    
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
            if (debug_cmd_valid) begin
                response_index <= 4'h0;
                sending_response <= 1'b1;
                
                case (debug_cmd)
                    CMD_NOP: begin
                        // No operation
                        response_length <= 4'h1;
                        response_buffer[0] <= CMD_NOP;
                    end
                    
                    CMD_GET_STATUS: begin
                        // Return system status
                        response_length <= 4'h4;
                        response_buffer[0] <= CMD_GET_STATUS;
                        response_buffer[1] <= {4'b0000, proxy_active, host_connected, device_connected, 1'b0};
                        response_buffer[2] <= {4'b0000, host_speed, device_speed};
                        response_buffer[3] <= {7'b0000000, buffer_overflow};
                    end
                    
                    CMD_GET_BUFFER_STATUS: begin
                        // Return buffer status
                        response_length <= 4'h3;
                        response_buffer[0] <= CMD_GET_BUFFER_STATUS;
                        response_buffer[1] <= buffer_used[7:0];
                        response_buffer[2] <= buffer_used[15:8];
                    end
                    
                    CMD_GET_PACKET_COUNT: begin
                        // Return packet count
                        response_length <= 4'h5;
                        response_buffer[0] <= CMD_GET_PACKET_COUNT;
                        response_buffer[1] <= packet_count[7:0];
                        response_buffer[2] <= packet_count[15:8];
                        response_buffer[3] <= packet_count[23:16];
                        response_buffer[4] <= packet_count[31:24];
                    end
                    
                    CMD_GET_ERROR_COUNT: begin
                        // Return error count
                        response_length <= 4'h3;
                        response_buffer[0] <= CMD_GET_ERROR_COUNT;
                        response_buffer[1] <= error_count[7:0];
                        response_buffer[2] <= error_count[15:8];
                    end
                    
                    CMD_GET_LINE_STATE: begin
                        // Return current line state
                        response_length <= 4'h2;
                        response_buffer[0] <= CMD_GET_LINE_STATE;
                        response_buffer[1] <= {4'b0000, device_line_state, host_line_state};
                    end
                    
                    CMD_GET_TIMESTAMP: begin
                        // Return current timestamp (lower 32 bits)
                        response_length <= 4'h5;
                        response_buffer[0] <= CMD_GET_TIMESTAMP;
                        response_buffer[1] <= timestamp[7:0];
                        response_buffer[2] <= timestamp[15:8];
                        response_buffer[3] <= timestamp[23:16];
                        response_buffer[4] <= timestamp[31:24];
                    end
                    
                    CMD_SET_DEBUG_LEDS: begin
                        // Set debug LEDs
                        debug_leds <= debug_cmd[7:0];
                        response_length <= 4'h2;
                        response_buffer[0] <= CMD_SET_DEBUG_LEDS;
                        response_buffer[1] <= debug_cmd[7:0];
                    end
                    
                    CMD_SET_DEBUG_PROBE: begin
                        // Set debug probe outputs
                        debug_probe <= debug_cmd[7:0];
                        response_length <= 4'h2;
                        response_buffer[0] <= CMD_SET_DEBUG_PROBE;
                        response_buffer[1] <= debug_cmd[7:0];
                    end
                    
                    CMD_SET_DEBUG_MODE: begin
                        // Set debug mode
                        debug_mode <= debug_cmd[1:0];
                        response_length <= 4'h2;
                        response_buffer[0] <= CMD_SET_DEBUG_MODE;
                        response_buffer[1] <= {6'b000000, debug_cmd[1:0]};
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
                        response_buffer[0] <= CMD_LOOPBACK_ENABLE;
                        response_buffer[1] <= {7'b0000000, debug_cmd[0]};
                    end
                    
                    CMD_TRIGGER_CONFIG: begin
                        // Configure trigger
                        trigger_config <= debug_cmd[7:0];
                        response_length <= 4'h2;
                        response_buffer[0] <= CMD_TRIGGER_CONFIG;
                        response_buffer[1] <= debug_cmd[7:0];
                    end
                    
                    CMD_VERSION: begin
                        // Return version information
                        response_length <= 4'h4;
                        response_buffer[0] <= CMD_VERSION;
                        response_buffer[1] <= VERSION_MAJOR;
                        response_buffer[2] <= VERSION_MINOR;
                        response_buffer[3] <= VERSION_PATCH;
                    end
                    
                    default: begin
                        // Unknown command
                        response_length <= 4'h2;
                        response_buffer[0] <= 8'hFF;  // Error response
                        response_buffer[1] <= debug_cmd;  // Echo unknown command
                    end
                endcase
            end
            
            // Response handling
            if (sending_response) begin
                if (response_index < response_length) begin
                    // Send next byte of response
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

    // Debug mode logic
    always @(posedge clk) begin
        // Debug mode functionality could be expanded here
        // For now, just use the mode selection for different LED display patterns
        case (debug_mode)
            2'b00: begin
                // Normal mode - LEDs as configured
            end
            
            2'b01: begin
                // Line state mode - display line states on lower 4 LEDs
                debug_leds[3:0] <= {device_line_state, host_line_state};
            end
            
            2'b10: begin
                // Activity mode - flash LEDs on packet activity
                if (packet_count != 0) begin
                    debug_leds[7] <= ~debug_leds[7];  // Toggle with packets
                end
            end
            
            2'b11: begin
                // Error mode - indicate errors
                if (error_count != 0) begin
                    debug_leds <= 8'hAA;  // Alternating pattern to indicate errors
                end
            end
        endcase
    end

endmodule