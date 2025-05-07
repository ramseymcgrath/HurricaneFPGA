///////////////////////////////////////////////////////////////////////////////
// File: usb_rx_fsm.v
// Description: USB Receive State Machine
//
// Handles USB packet reception, decoding, and CRC verification
///////////////////////////////////////////////////////////////////////////////

module usb_rx_fsm #(
    parameter CLK_FREQ_MHZ = 60  // Default 60MHz clock frequency
)(
    input wire        clk,
    input wire        rst_n,
    
    // UTMI RX Interface
    input wire [7:0]  utmi_rx_data,
    input wire        utmi_rx_valid,
    input wire        utmi_rx_active,
    input wire        utmi_rx_error,
    
    // Protocol Interface Outputs
    output reg [7:0]  packet_data,
    output reg        packet_valid,
    output reg        packet_sop,
    output reg        packet_eop,
    output reg [3:0]  pid,
    output reg [6:0]  dev_addr,
    output reg [3:0]  endp,
    output reg        crc_valid
);
    // PID definitions as an enumerated type
    typedef enum logic [3:0] {
        PID_OUT   = 4'b0001,
        PID_IN    = 4'b1001,
        PID_SETUP = 4'b1101,
        PID_DATA0 = 4'b0011,
        PID_DATA1 = 4'b1011,
        PID_DATA2 = 4'b0111,
        PID_MDATA = 4'b1111,
        PID_ACK   = 4'b0010,
        PID_NAK   = 4'b1010,
        PID_STALL = 4'b1110,
        PID_NYET  = 4'b0110,
        PID_PRE   = 4'b1100,
        PID_SOF   = 4'b0101,
        PID_PING  = 4'b0100,
        PID_SPLIT = 4'b1000
    } pid_t;
    
    // FSM states as an enumerated type
    typedef enum logic [3:0] {
        ST_IDLE       = 4'd0,
        ST_RX_PID     = 4'd1,
        ST_RX_TOKEN   = 4'd2,
        ST_RX_DATA    = 4'd3,
        ST_WAIT_EOP   = 4'd4
    } state_t;
    
    // State register
    state_t rx_state;
    
    // Internal registers
    reg [15:0] token_data;     // Token data buffer
    reg [2:0]  byte_cnt;       // Byte counter
    reg [4:0]  crc5;           // CRC5 register for token packets
    reg [15:0] crc16;          // CRC16 register for data packets
    reg [4:0]  crc5_next;      // Next CRC5 value
    reg [15:0] crc16_next;     // Next CRC16 value
    
    // Iterator for loops
    integer i;
    
    // CRC5 polynomial: x^5 + x^2 + 1
    function [4:0] crc5_update;
        input [4:0] crc_in;
        input data_bit;
        reg feedback;
    begin
        feedback = data_bit ^ crc_in[4];
        crc5_update = {crc_in[3:0], 1'b0};
        if (feedback) begin
            crc5_update = crc5_update ^ 5'b00101;
        end
    end
    endfunction
    
    // CRC16 polynomial: x^16 + x^15 + x^2 + 1
    function [15:0] crc16_update;
        input [15:0] crc_in;
        input data_bit;
        reg feedback;
    begin
        feedback = data_bit ^ crc_in[15];
        crc16_update = {crc_in[14:0], 1'b0};
        if (feedback) begin
            crc16_update = crc16_update ^ 16'h8005;
        end
    end
    endfunction
    
    // Packet reception and processing
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_state <= ST_IDLE;
            packet_data <= 8'd0;
            packet_valid <= 1'b0;
            packet_sop <= 1'b0;
            packet_eop <= 1'b0;
            pid <= 4'd0;
            dev_addr <= 7'd0;
            endp <= 4'd0;
            crc_valid <= 1'b0;
            token_data <= 16'd0;
            byte_cnt <= 3'd0;
            crc5 <= 5'b11111;
            crc16 <= 16'hFFFF;
        end else begin
            // Default assignments
            packet_valid <= 1'b0;
            packet_sop <= 1'b0;
            packet_eop <= 1'b0;
            
            case (rx_state)
                ST_IDLE: begin
                    if (utmi_rx_active && utmi_rx_valid) begin
                        // Start of packet, capture PID
                        rx_state <= ST_RX_PID;
                        packet_sop <= 1'b1;
                        packet_valid <= 1'b1;
                        packet_data <= utmi_rx_data;
                        pid <= utmi_rx_data[3:0]; // Lower nibble is the PID
                        byte_cnt <= 3'd0;
                        crc5 <= 5'b11111;  // Initialize CRC5
                        crc16 <= 16'hFFFF; // Initialize CRC16
                    end
                end
                
                ST_RX_PID: begin
                    // Determine packet type based on PID
                    if (!utmi_rx_active) begin
                        // Short packet, only PID
                        rx_state <= ST_IDLE;
                        packet_eop <= 1'b1;
                    end else if (utmi_rx_valid) begin
                        packet_valid <= 1'b1;
                        packet_data <= utmi_rx_data;
                        
                        unique case (pid)
                            PID_OUT, PID_IN, PID_SETUP, PID_PING, PID_SOF: begin
                                // TOKEN packet
                                rx_state <= ST_RX_TOKEN;
                                token_data[7:0] <= utmi_rx_data;
                                byte_cnt <= 3'd1;
                                
                                // Update CRC5
                                crc5_next = 5'b11111;
                                for (i=0; i<8; i=i+1) begin
                                    crc5_next = crc5_update(crc5_next, utmi_rx_data[i]);
                                end
                                crc5 <= crc5_next;
                            end
                            
                            PID_DATA0, PID_DATA1, PID_DATA2, PID_MDATA: begin
                                // DATA packet
                                rx_state <= ST_RX_DATA;
                                
                                // Update CRC16
                                crc16_next = 16'hFFFF;
                                for (i=0; i<8; i=i+1) begin
                                    crc16_next = crc16_update(crc16_next, utmi_rx_data[i]);
                                end
                                crc16 <= crc16_next;
                            end
                            
                            default: begin
                                // Handshake packet or other - no additional data expected
                                rx_state <= ST_WAIT_EOP;
                            end
                        endcase
                    end
                end
                
                ST_RX_TOKEN: begin
                    if (!utmi_rx_active) begin
                        rx_state <= ST_IDLE;
                        packet_eop <= 1'b1;
                        // Process incomplete token
                    end else if (utmi_rx_valid) begin
                        packet_valid <= 1'b1;
                        packet_data <= utmi_rx_data;
                        
                        if (byte_cnt == 3'd1) begin
                            token_data[15:8] <= utmi_rx_data;
                            byte_cnt <= 3'd2;
                            
                            // Continue CRC5 calculation
                            crc5_next = crc5;
                            for (i=0; i<8; i=i+1) begin
                                crc5_next = crc5_update(crc5_next, utmi_rx_data[i]);
                            end
                            crc5 <= crc5_next;
                            
                            // Extract device address and endpoint LSB from token
                            // Byte-1 format: [endp0 | addr6:0]
                            dev_addr <= utmi_rx_data[6:0];
                            endp[0] <= utmi_rx_data[7];
                        end else begin
                            // End of TOKEN packet, second data byte
                            rx_state <= ST_WAIT_EOP;
                            
                            // Extract remaining endpoint bits and check CRC5
                            // Byte-2 format: [CRC5 | endp3:1]
                            endp[3:1] <= utmi_rx_data[2:0];
                            crc_valid <= (utmi_rx_data[7:3] == ~crc5);
                        end
                    end
                end
                
                ST_RX_DATA: begin
                    if (!utmi_rx_active) begin
                        rx_state <= ST_IDLE;
                        packet_eop <= 1'b1;
                        
                        // Check final CRC16 - compare with correct residue
                        crc_valid <= (crc16 == 16'hB001); // USB-IF specified CRC residue
                    end else if (utmi_rx_valid) begin
                        packet_valid <= 1'b1;
                        packet_data <= utmi_rx_data;
                        
                        // Update CRC16
                        crc16_next = crc16;
                        for (i=0; i<8; i=i+1) begin
                            crc16_next = crc16_update(crc16_next, utmi_rx_data[i]);
                        end
                        crc16 <= crc16_next;
                    end
                end
                
                ST_WAIT_EOP: begin
                    if (!utmi_rx_active) begin
                        rx_state <= ST_IDLE;
                        packet_eop <= 1'b1;
                    end
                end
                
                default: rx_state <= ST_IDLE;
            endcase
            
            // Handle RX errors
            if (utmi_rx_error) begin
                rx_state <= ST_IDLE;
                crc_valid <= 1'b0;
            end
        end
    end
    
endmodule