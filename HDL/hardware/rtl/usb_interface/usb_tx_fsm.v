///////////////////////////////////////////////////////////////////////////////
// File: usb_tx_fsm.v
// Description: USB Transmit State Machine
//
// Handles USB packet transmission, encoding, and CRC generation
///////////////////////////////////////////////////////////////////////////////

module usb_tx_fsm #(
    parameter CLK_FREQ_MHZ = 60  // Default 60MHz clock frequency
)(
    input wire        clk,
    input wire        rst_n,
    
    // UTMI TX Interface
    output reg [7:0]  utmi_tx_data,
    output reg        utmi_tx_valid,
    input wire        utmi_tx_ready,
    
    // Protocol Interface Inputs
    input wire [7:0]  tx_packet_data,
    input wire        tx_packet_valid,
    input wire        tx_packet_sop,
    input wire        tx_packet_eop,
    output wire       tx_packet_ready,
    input wire [3:0]  tx_pid
);
    // PID definitions as localparams
    localparam [3:0]
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
        PID_SPLIT = 4'b1000;
    
    // FSM states as localparams
    localparam [3:0]
        ST_IDLE   = 4'd0,
        ST_TX_PID = 4'd1,
        ST_TX_DATA = 4'd2,
        ST_TX_CRC = 4'd3,
        ST_TX_EOP = 4'd4;
    
    // State register
    reg [3:0] tx_state;
    
    // Internal registers
    reg [3:0]  tx_byte_cnt;          // Transmit byte counter
    reg [3:0]  tx_length;            // Transmit length
    reg        crc_tx_phase;         // Indicates which CRC byte is being transmitted
    reg [4:0]  tx_crc5;              // CRC5 for token packets
    reg [15:0] tx_crc16;             // CRC16 for data packets
    reg [4:0]  crc5_next;            // Next CRC5 value
    reg [15:0] crc16_next;           // Next CRC16 value
    
    // Block RAM for transmit buffer (helps with timing)
    (* ram_style = "block" *) reg [7:0] tx_data_buffer [0:15];
    reg tx_wr_en;                    // Memory write enable
    reg [3:0] tx_wr_addr;            // Memory write address
    
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
    
    // Packet transmission state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            utmi_tx_data <= 8'd0;
            utmi_tx_valid <= 1'b0;
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
                                tx_length <= 4'd2;  // Fixed length for token packets
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
                            // Store data in buffer for possible retransmission
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
                                
                                // Check if we've sent all token bytes
                                if (tx_byte_cnt == (tx_length - 1)) begin
                                    tx_state <= ST_TX_CRC;
                                end
                            end else begin
                                // Update CRC16 for data packets
                                crc16_next = tx_crc16;
                                for (i=0; i<8; i=i+1) begin
                                    crc16_next = crc16_update(crc16_next, tx_packet_data[i]);
                                end
                                tx_crc16 <= crc16_next;
                            end
                            
                            // End of data packet
                            if (tx_packet_eop) begin
                                tx_state <= ST_TX_CRC;
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
                                // Note: CRC5 is inverted as per USB spec
                                utmi_tx_data <= {3'b000, tx_crc5 ^ 5'b11111};
                                tx_state <= ST_TX_EOP;
                            end
                            
                            PID_DATA0, PID_DATA1, PID_DATA2, PID_MDATA: begin
                                // Send CRC16 for data packets (2 bytes)
                                if (crc_tx_phase == 1'b0) begin
                                    // First byte of CRC16 (LSB)
                                    utmi_tx_data <= (tx_crc16[7:0] ^ 8'hFF);
                                    crc_tx_phase <= 1'b1;
                                end else begin
                                    // Second byte of CRC16 (MSB)
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
    
    // Ready signal for transmit data
    assign tx_packet_ready = (tx_state == ST_IDLE) || 
                             (tx_state == ST_TX_DATA && utmi_tx_ready);

endmodule