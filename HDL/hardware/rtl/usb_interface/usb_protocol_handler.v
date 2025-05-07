///////////////////////////////////////////////////////////////////////////////
// File: usb_protocol_handler.v
// Description: USB Protocol Handler for transparent proxy implementation
//
// This module handles USB protocol operations including packet encoding/decoding,
// CRC verification/generation, and protocol-specific operations.
//
// Target: Lattice ECP5 on Cynthion device
///////////////////////////////////////////////////////////////////////////////

module usb_protocol_handler (
    // Clock and Reset
    input  wire        clk,             // System clock
    input  wire        rst_n,           // Active low reset

    // UTMI Interface from PHY
    input  wire [7:0]  utmi_rx_data,    // Received data
    input  wire        utmi_rx_valid,   // Received data valid
    input  wire        utmi_rx_active,  // Receiving packet
    input  wire        utmi_rx_error,   // Receive error
    // UTMI Interface to PHY
    input  wire [1:0]  utmi_line_state,  // USB line state (SE0, J, K, SE1)
    output reg  [7:0]  utmi_tx_data,    // Data to transmit
    output reg         utmi_tx_valid,   // Data valid for transmission
    input  wire        utmi_tx_ready,   // PHY ready for transmission
    output reg  [1:0]  utmi_tx_op_mode, // Transmit operation mode
    output reg  [1:0]  utmi_xcvr_select,// Transceiver speed selection
    output reg         utmi_termselect, // Termination selection
    output reg         utmi_dppulldown, // D+ pulldown enable
    output reg         utmi_dmpulldown, // D- pulldown enable

    // Protocol Decoded Interface
    output reg  [7:0]  packet_data,     // Decoded packet data
    output reg         packet_valid,    // Decoded packet valid
    output reg         packet_sop,      // Start of packet indicator
    output reg         packet_eop,      // End of packet indicator
    output reg  [3:0]  pid,             // Packet ID (TOKEN, DATA, HANDSHAKE, SPECIAL)
    output reg  [6:0]  dev_addr,        // Device address (from TOKEN)
    output reg  [3:0]  endp,            // Endpoint number (from TOKEN)
    output reg         crc_valid,       // CRC validation result
    
    // Protocol Control Interface
    input  wire [7:0]  tx_packet_data,  // Transmit packet data
    input  wire        tx_packet_valid, // Transmit packet valid
    input  wire        tx_packet_sop,   // Start of transmit packet
    input  wire        tx_packet_eop,   // End of transmit packet
    output wire        tx_packet_ready, // Ready to accept transmit data
    input  wire [3:0]  tx_pid,          // PID for transmission
    
    // Configuration and Status
    input  wire [6:0]  device_address,  // Configured device address
    input  wire [1:0]  usb_speed,       // USB speed mode (00=LS, 01=FS, 10=HS)
    output reg         conn_detect,     // Connection detected
    output reg  [1:0]  conn_speed,      // Connection speed
    output reg         reset_detect,    // USB reset detected
    output reg         suspend_detect,  // USB suspend detected
    output reg         resume_detect    // USB resume detected
);

    // Local parameters
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
    localparam PID_PRE   = 4'b1100;
    localparam PID_SOF   = 4'b0101;
    localparam PID_PING  = 4'b0100;
    localparam PID_SPLIT = 4'b1000;

    // FSM states
    localparam ST_IDLE       = 4'd0;
    localparam ST_RX_PID     = 4'd1;
    localparam ST_RX_TOKEN   = 4'd2;
    localparam ST_RX_DATA    = 4'd3;
    localparam ST_RX_CRC     = 4'd4;
    localparam ST_TX_PID     = 4'd5;
    localparam ST_TX_DATA    = 4'd6;
    localparam ST_TX_CRC     = 4'd7;
    localparam ST_TX_EOP     = 4'd8;
    localparam ST_WAIT_EOP   = 4'd9;
    
    // Internal registers
    reg [3:0]  rx_state;             // Receive FSM state
    reg [3:0]  tx_state;             // Transmit FSM state
    reg [15:0] token_data;           // Token data buffer
    reg [15:0] crc16;                // CRC16 register
    reg [15:0] crc16_next;           // Next CRC16 value
    reg [15:0] tx_crc16;             // CRC16 for transmission
    reg [4:0]  crc5;                 // CRC5 register
    reg [4:0]  crc5_next;            // Next CRC5 value
    reg [4:0]  tx_crc5;              // CRC5 for transmission
    reg [2:0]  byte_cnt;             // Byte counter
    (* ram_style = "block" *) reg [7:0] tx_data_buffer [0:15]; // Block RAM for transmit buffer
    reg tx_wr_en;                    // Memory write enable
    reg [3:0] tx_wr_addr;            // Memory write address
    reg [3:0]  tx_byte_cnt;          // Transmit byte counter
    reg [3:0]  tx_length;            // Transmit length
    reg        crc_tx_phase;         // Indicates which CRC byte is being transmitted
    
    // Loop iterator for all for loops in the module
    integer i;
    
    // Line state monitoring
    reg [19:0] se0_counter;          // Counter for SE0 condition (reset detection)
    reg [23:0] idle_counter;         // Counter for idle condition (suspend detection)
    
    // Clock frequency parameter (default 60MHz)
    localparam CLK_FREQ_MHZ = 60;
    localparam RESET_CYCLES = CLK_FREQ_MHZ * 2.5; // 2.5us
    localparam SUSPEND_CYCLES = CLK_FREQ_MHZ * 3000; // 3ms
    
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

    // USB reset, suspend, and resume detection
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            se0_counter <= 20'd0;
            idle_counter <= 24'd0;
            reset_detect <= 1'b0;
            suspend_detect <= 1'b0;
            resume_detect <= 1'b0;
        end else begin
            // Reset detection (SE0 for >2.5us)
            if (utmi_line_state == 2'b00) begin  // SE0 state
                if (se0_counter < 20'hFFFFF) begin
                    se0_counter <= se0_counter + 1'b1;
                end
                
                // At configured clock frequency, RESET_CYCLES = 2.5us
                if (se0_counter == RESET_CYCLES) begin
                    reset_detect <= 1'b1;
                end
            end else begin
                se0_counter <= 20'd0;
                if (se0_counter > RESET_CYCLES) begin
                    reset_detect <= 1'b0;  // End of reset
                end
            end
            
            // Suspend detection (idle for >3ms)
            if (utmi_line_state == 2'b01) begin  // J state (idle)
                if (idle_counter < 24'hFFFFFF) begin
                    idle_counter <= idle_counter + 1'b1;
                end
                
                // At configured clock frequency, SUSPEND_CYCLES = 3ms
                if (idle_counter == SUSPEND_CYCLES) begin
                    suspend_detect <= 1'b1;
                end
            end else begin
                idle_counter <= 24'd0;
                if (idle_counter > SUSPEND_CYCLES && utmi_line_state == 2'b10) begin  // K state after suspend
                    resume_detect <= 1'b1;
                    suspend_detect <= 1'b0;
                end else begin
                    resume_detect <= 1'b0;
                end
            end
        end
    end

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
                        crc5 <= 5'b11111;
                        crc16 <= 16'hFFFF;
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
                        
                        case (pid)
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
                            // Byte-1 = [endp0 | addr6:0]
                            dev_addr <= utmi_rx_data[6:0];
                            endp[0] <= utmi_rx_data[7];
                        end else begin
                            // End of TOKEN packet, second data byte
                            rx_state <= ST_WAIT_EOP;
                            
                            // Extract remaining endpoint bits and check CRC5
                            // Byte-2 = [CRC5 | endp3:1]
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
    
    // Packet transmission state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            utmi_tx_data <= 8'd0;
            utmi_tx_valid <= 1'b0;
            utmi_tx_op_mode <= 2'b00;
            tx_byte_cnt <= 4'd0;
            tx_length <= 4'd0;
            tx_state <= ST_IDLE;
            tx_wr_en <= 1'b0;
            tx_wr_addr <= 4'd0;
            crc_tx_phase <= 1'b0;
            tx_crc5 <= 5'b11111;
            tx_crc16 <= 16'hFFFF;
        end else begin
            // Default assignments
            tx_wr_en <= 1'b0;
            
            case (tx_state)
                ST_IDLE: begin
                    utmi_tx_valid <= 1'b0;
                    crc_tx_phase <= 1'b0;
                    if (tx_packet_valid && tx_packet_sop) begin
                        // Start transmission with PID
                        tx_state <= ST_TX_PID;
                        utmi_tx_valid <= 1'b1;
                        utmi_tx_data <= {~tx_pid, tx_pid}; // PID with complement as per USB spec
                        tx_byte_cnt <= 4'd0;
                        
                        // Initialize CRC values for transmission
                        tx_crc5 <= 5'b11111;
                        tx_crc16 <= 16'hFFFF;
                    end
                end
                
                ST_TX_PID: begin
                    if (utmi_tx_ready) begin
                        // Determine next state based on PID type
                        case (tx_pid)
                            PID_OUT, PID_IN, PID_SETUP, PID_PING, PID_SOF: begin
                                // TOKEN packets have two additional bytes
                                tx_state <= ST_TX_DATA;
                                tx_length <= 4'd2;
                            end
                            
                            PID_DATA0, PID_DATA1, PID_DATA2, PID_MDATA: begin
                                // DATA packets have variable length + 2 byte CRC16
                                tx_state <= ST_TX_DATA;
                            end
                            
                            default: begin
                                // Handshake packets are PID only
                                tx_state <= ST_TX_EOP;
                                utmi_tx_valid <= 1'b0;
                            end
                        endcase
                    end
                end
                
                ST_TX_DATA: begin
                    if (utmi_tx_ready) begin
                        if (tx_packet_valid) begin
                            // Store data in buffer and update CRC
                            tx_wr_en <= 1'b1;
                            tx_wr_addr <= tx_byte_cnt;
                            tx_data_buffer[tx_wr_addr] <= tx_packet_data;
                            
                            // Transmit data
                            utmi_tx_data <= tx_packet_data;
                            utmi_tx_valid <= 1'b1;
                            tx_byte_cnt <= tx_byte_cnt + 1'b1;
                            
                            // Update CRC based on packet type
                            if (tx_pid == PID_OUT || tx_pid == PID_IN || 
                                tx_pid == PID_SETUP || tx_pid == PID_PING || tx_pid == PID_SOF) begin
                                // Update CRC5 for token packets
                                crc5_next = tx_crc5;
                                for (i=0; i<8; i=i+1) begin
                                    crc5_next = crc5_update(crc5_next, tx_packet_data[i]);
                                end
                                tx_crc5 <= crc5_next;
                            end else begin
                                // Update CRC16 for data packets
                                crc16_next = tx_crc16;
                                for (i=0; i<8; i=i+1) begin
                                    crc16_next = crc16_update(crc16_next, tx_packet_data[i]);
                                end
                                tx_crc16 <= crc16_next;
                            end
                            
                            if (tx_packet_eop) begin
                                tx_state <= ST_TX_CRC;
                                utmi_tx_valid <= 1'b0;
                            end
                        end
                    end
                end
                
                ST_TX_CRC: begin
                    if (utmi_tx_ready) begin
                        utmi_tx_valid <= 1'b1;
                        
                        case (tx_pid)
                            PID_OUT, PID_IN, PID_SETUP, PID_PING, PID_SOF: begin
                                // Send CRC5 for token packets
                                utmi_tx_data <= {3'b000, tx_crc5 ^ 5'b11111};
                                tx_state <= ST_TX_EOP;
                            end
                            
                            PID_DATA0, PID_DATA1, PID_DATA2, PID_MDATA: begin
                                // Send CRC16 for data packets (2 bytes)
                                if (crc_tx_phase == 1'b0) begin
                                    // First byte of CRC16
                                    utmi_tx_data <= (tx_crc16[7:0] ^ 8'hFF);
                                    crc_tx_phase <= 1'b1;
                                end else begin
                                    // Second byte of CRC16
                                    utmi_tx_data <= (tx_crc16[15:8] ^ 8'hFF);
                                    tx_state <= ST_TX_EOP;
                                end
                            end
                            
                            default: begin
                                utmi_tx_valid <= 1'b0;
                                tx_state <= ST_TX_EOP;
                            end
                        endcase
                    end
                end
                
                ST_TX_EOP: begin
                    if (utmi_tx_ready) begin
                        utmi_tx_valid <= 1'b0;
                        tx_state <= ST_IDLE;
                    end
                end
                
                default: tx_state <= ST_IDLE;
            endcase
        end
    end
    
    // Connection detection logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            conn_detect <= 1'b0;
            conn_speed <= 2'b00;
        end else begin
            // Simple detection based on line state
            if (utmi_line_state != 2'b00) begin
                conn_detect <= 1'b1;
                
                // Speed detection - rely more on PHY xcvr selection
                // Note: For proper HS detection, we should implement
                // proper chirp K/J detection logic
                conn_speed <= usb_speed; // Use the configured speed
            end else if (se0_counter > 20'd10000) begin
                // Long SE0 indicates no device
                conn_detect <= 1'b0;
            end
        end
    end
    
    // Configuration outputs
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            utmi_xcvr_select <= 2'b01; // Default to full-speed
            utmi_termselect <= 1'b1;   // Full-speed termination
            utmi_dppulldown <= 1'b0;
            utmi_dmpulldown <= 1'b0;
        end else begin
            // Configure based on speed mode
            utmi_xcvr_select <= usb_speed;
            
            case (usb_speed)
                2'b00: begin  // Low-speed
                    utmi_termselect <= 1'b1;
                    utmi_dppulldown <= 1'b0;
                    utmi_dmpulldown <= 1'b1; // Pull-down on D-
                end
                2'b01: begin  // Full-speed
                    utmi_termselect <= 1'b1;
                    utmi_dppulldown <= 1'b0;
                    utmi_dmpulldown <= 1'b0;
                end
                2'b10: begin  // High-speed
                    utmi_termselect <= 1'b0;
                    utmi_dppulldown <= 1'b0;
                    utmi_dmpulldown <= 1'b0;
                end
                default: begin
                    utmi_termselect <= 1'b1;
                    utmi_dppulldown <= 1'b0;
                    utmi_dmpulldown <= 1'b0;
                end
            endcase
        end
    end
    
    // Ready signal for transmit data
    assign tx_packet_ready = (tx_state == ST_IDLE) || (tx_state == ST_TX_DATA && utmi_tx_ready);

endmodule