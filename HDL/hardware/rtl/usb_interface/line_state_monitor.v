///////////////////////////////////////////////////////////////////////////////
// File: line_state_monitor.v
// Description: USB Line State Monitor for reset, suspend, and resume detection
//
// Handles timing-critical bus state monitoring with parameterized timers
///////////////////////////////////////////////////////////////////////////////

module line_state_monitor #(
    parameter CLK_FREQ_MHZ = 60  // Default 60MHz clock frequency
)(
    input wire        clk,
    input wire        rst_n,
    input wire [1:0]  line_state,    // UTMI line state (00=SE0, 01=J, 10=K, 11=SE1)
    output reg        reset_detect,  // USB bus reset detected
    output reg        suspend_detect,// USB suspend detected
    output reg        resume_detect  // USB resume detected
);
    // Constants for bus state detection at configured clock frequency
    localparam RESET_CYCLES = (CLK_FREQ_MHZ * 25) / 10;   // 2.5us of SE0 (converted to integer calculation)
    localparam SUSPEND_CYCLES = CLK_FREQ_MHZ * 3000;      // 3ms of idle J state
    
    // State detection counters
    reg [19:0] se0_counter;      // Counter for SE0 condition (reset detection)
    reg [23:0] idle_counter;     // Counter for idle condition (suspend detection)
    
    // Reset, suspend, and resume detection logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            se0_counter <= 20'd0;
            idle_counter <= 24'd0;
            reset_detect <= 1'b0;
            suspend_detect <= 1'b0;
            resume_detect <= 1'b0;
        end else begin
            // Reset detection (SE0 for >2.5us)
            if (line_state == 2'b00) begin  // SE0 state
                if (se0_counter < 20'hFFFFF) begin
                    se0_counter <= se0_counter + 1'b1;
                end
                
                if (se0_counter == RESET_CYCLES) begin
                    reset_detect <= 1'b1;
                end
            end else begin
                se0_counter <= 20'd0;
                if (se0_counter > 20'd0 && reset_detect) begin
                    reset_detect <= 1'b0;  // End of reset only if it was previously detected
                end
            end
            
            // Suspend detection (idle for >3ms)
            if (line_state == 2'b01) begin  // J state (idle)
                if (idle_counter < 24'hFFFFFF) begin
                    idle_counter <= idle_counter + 1'b1;
                end
                
                if (idle_counter == SUSPEND_CYCLES) begin
                    suspend_detect <= 1'b1;
                end
            end else begin
                idle_counter <= 24'd0;
                if (suspend_detect && line_state == 2'b10) begin  // K state after suspend
                    resume_detect <= 1'b1;
                    suspend_detect <= 1'b0;
                end else begin
                    resume_detect <= 1'b0;
                end
            end
        end
    end

endmodule