///////////////////////////////////////////////////////////////////////////////
// File: timestamp_generator.v
// Description: Accurate timestamp generation module
//
// This module provides precise timing information for USB packets with
// configurable time resolution and synchronization across clock domains.
//
// Target: Lattice ECP5 on Cynthion device
///////////////////////////////////////////////////////////////////////////////

module timestamp_generator (
    // Clock and Reset
    input  wire        clk,             // Main system clock (60MHz)
    input  wire        clk_high,        // High-frequency clock (240MHz)
    input  wire        rst_n,           // Active low reset

    // Timestamp Outputs
    output reg  [63:0] timestamp,       // Current timestamp value (64-bit)
    output wire [31:0] timestamp_ms,    // Millisecond timestamp
    output wire [15:0] sof_frame_num,   // USB SOF frame number
    
    // Synchronization Inputs
    input  wire        sync_enable,     // Enable external synchronization
    input  wire        sync_pulse,      // External synchronization pulse
    input  wire [63:0] sync_value,      // Synchronization value
    
    // USB Frame Sync
    input  wire        sof_detected,    // SOF packet detected
    input  wire [10:0] sof_frame_num_in,// SOF frame number from USB
    
    // Configuration
    input  wire [3:0]  resolution_ctrl, // Timestamp resolution control
    input  wire        counter_enable,  // Enable/disable counting
    input  wire        reset_counter,   // Reset timestamp counter
    
    // Status
    output reg         timestamp_valid, // Timestamp is valid and synchronized
    output reg  [31:0] timestamp_rate   // Effective timestamp rate (ticks/second)
);

    // Constants
    localparam BASE_TICK_RATE = 60000000; // 60MHz base clock rate
    localparam HIGH_TICK_RATE = 240000000; // 240MHz high clock rate
    
    // Internal registers
    reg [63:0] counter;             // Primary counter
    reg [63:0] high_res_counter;    // High resolution counter
    reg [31:0] ms_counter;          // Millisecond counter
    reg [63:0] ticks_per_ms;        // Ticks per millisecond
    reg [15:0] frame_counter;       // USB frame counter
    reg [31:0] sof_time_reference;  // Time reference for SOF
    reg        sof_sync_valid;      // SOF sync is valid
    reg [3:0]  current_resolution;  // Current resolution setting
    reg [31:0] clock_divider;       // Clock divider
    reg [31:0] divider_counter;     // Divider counter
    reg        tick_enable;         // Tick enable flag
    
    // Resolution settings for timestamp_rate
    always @(*) begin
        case (resolution_ctrl)
            4'd0: begin // 60MHz (full system clock)
                timestamp_rate = BASE_TICK_RATE;
                ticks_per_ms = 60000;
                clock_divider = 1;
            end
            4'd1: begin // 30MHz (1/2 system clock)
                timestamp_rate = BASE_TICK_RATE / 2;
                ticks_per_ms = 30000;
                clock_divider = 2;
            end
            4'd2: begin // 15MHz (1/4 system clock)
                timestamp_rate = BASE_TICK_RATE / 4;
                ticks_per_ms = 15000;
                clock_divider = 4;
            end
            4'd3: begin // 6MHz (1/10 system clock)
                timestamp_rate = BASE_TICK_RATE / 10;
                ticks_per_ms = 6000;
                clock_divider = 10;
            end
            4'd4: begin // 1MHz (1/60 system clock)
                timestamp_rate = 1000000;
                ticks_per_ms = 1000;
                clock_divider = 60;
            end
            4'd5: begin // 1KHz (1ms resolution)
                timestamp_rate = 1000;
                ticks_per_ms = 1;
                clock_divider = 60000;
            end
            4'd6: begin // 240MHz (high frequency clock)
                timestamp_rate = HIGH_TICK_RATE;
                ticks_per_ms = 240000;
                clock_divider = 1;
            end
            default: begin // Default 60MHz
                timestamp_rate = BASE_TICK_RATE;
                ticks_per_ms = 60000;
                clock_divider = 1;
            end
        endcase
    end
    
    // Main timestamp counter
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter <= 64'd0;
            divider_counter <= 32'd0;
            tick_enable <= 1'b0;
            ms_counter <= 32'd0;
            current_resolution <= 4'd0;
            timestamp_valid <= 1'b0;
            frame_counter <= 16'd0;
            sof_sync_valid <= 1'b0;
            sof_time_reference <= 32'd0;
        end else begin
            // Resolution change detection
            if (current_resolution != resolution_ctrl) begin
                current_resolution <= resolution_ctrl;
                divider_counter <= 32'd0;
                tick_enable <= 1'b0;
            end
            
            // Counter operation
            if (counter_enable) begin
                // Clock division logic
                if (clock_divider == 1) begin
                    // No division needed
                    tick_enable <= 1'b1;
                end else begin
                    if (divider_counter >= (clock_divider - 1)) begin
                        divider_counter <= 32'd0;
                        tick_enable <= 1'b1;
                    end else begin
                        divider_counter <= divider_counter + 1'b1;
                        tick_enable <= 1'b0;
                    end
                end
                
                // Counter update on tick enable
                if (tick_enable) begin
                    counter <= counter + 1'b1;
                    
                    // Millisecond counter update
                    if (counter % ticks_per_ms == 0) begin
                        ms_counter <= ms_counter + 1'b1;
                    end
                    
                    // Mark timestamp as valid after a short period
                    if (counter > 32'd1000) begin
                        timestamp_valid <= 1'b1;
                    end
                end
            end
            
            // Counter reset
            if (reset_counter) begin
                counter <= 64'd0;
                ms_counter <= 32'd0;
                timestamp_valid <= 1'b0;
            end
            
            // External synchronization
            if (sync_enable && sync_pulse) begin
                counter <= sync_value;
                timestamp_valid <= 1'b1;
            end
            
            // SOF frame synchronization
            if (sof_detected) begin
                // Update internal frame counter
                frame_counter <= {5'b00000, sof_frame_num_in};
                
                if (!sof_sync_valid) begin
                    // First SOF - initialize sync
                    sof_time_reference <= counter[31:0];
                    sof_sync_valid <= 1'b1;
                end else begin
                    // Calculate time since last SOF (should be ~1ms)
                    // Can use this to fine-tune the timestamp rate if needed
                end
            end
        end
    end
    
    // High resolution timestamp using the high-frequency clock
    always @(posedge clk_high or negedge rst_n) begin
        if (!rst_n) begin
            high_res_counter <= 64'd0;
        end else if (counter_enable) begin
            // Simple free-running counter at high speed
            high_res_counter <= high_res_counter + 1'b1;
        end
        
        if (reset_counter) begin
            high_res_counter <= 64'd0;
        end
        
        // External synchronization for high-res counter
        if (sync_enable && sync_pulse) begin
            high_res_counter <= sync_value;
        end
    end
    
    // Output timestamp selection based on resolution
    always @(*) begin
        if (resolution_ctrl == 4'd6) begin
            // Use high-resolution timestamp
            timestamp = high_res_counter;
        end else begin
            // Use main counter
            timestamp = counter;
        end
    end
    
    // Millisecond timestamp output
    assign timestamp_ms = ms_counter;
    
    // SOF frame number output
    assign sof_frame_num = frame_counter;

endmodule