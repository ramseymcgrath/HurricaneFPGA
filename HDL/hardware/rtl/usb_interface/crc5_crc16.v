///////////////////////////////////////////////////////////////////////////////
// File: crc5_crc16.v
// Description: Dedicated CRC generators for USB protocol
//
// This module provides both CRC5 and CRC16 calculation functionality
// with optimized shift register implementations.
///////////////////////////////////////////////////////////////////////////////

module crc5_generator(
    input wire        clk,
    input wire        rst_n,
    input wire        enable,
    input wire        data_in,
    input wire        clear,
    output wire [4:0] crc_out
);
    // CRC5 polynomial: x^5 + x^2 + 1 (USB standard)
    reg [4:0] crc_reg;
    wire feedback;
    
    assign feedback = data_in ^ crc_reg[4];
    assign crc_out = crc_reg;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg <= 5'b11111;
        end else if (clear) begin
            crc_reg <= 5'b11111;
        end else if (enable) begin
            crc_reg <= {crc_reg[3:0], 1'b0};
            if (feedback) begin
                crc_reg <= {crc_reg[3:0], 1'b0} ^ 5'b00101;
            end
        end
    end
endmodule

module crc16_generator(
    input wire         clk,
    input wire         rst_n,
    input wire         enable,
    input wire         data_in,
    input wire         clear,
    output wire [15:0] crc_out
);
    // CRC16 polynomial: x^16 + x^15 + x^2 + 1 (USB standard)
    reg [15:0] crc_reg;
    wire feedback;
    
    assign feedback = data_in ^ crc_reg[15];
    assign crc_out = crc_reg;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            crc_reg <= 16'hFFFF;
        end else if (clear) begin
            crc_reg <= 16'hFFFF;
        end else if (enable) begin
            crc_reg <= {crc_reg[14:0], 1'b0};
            if (feedback) begin
                crc_reg <= {crc_reg[14:0], 1'b0} ^ 16'h8005;
            end
        end
    end
endmodule

module byte_to_serial(
    input wire        clk,
    input wire        rst_n,
    input wire        start,
    input wire [7:0]  data_in,
    output reg        data_out,
    output reg        valid,
    output wire       done
);
    reg [2:0] bit_counter;
    reg active;
    
    assign done = (bit_counter == 3'd7) && active;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bit_counter <= 3'd0;
            data_out <= 1'b0;
            valid <= 1'b0;
            active <= 1'b0;
        end else begin
            valid <= active;
            
            if (start) begin
                bit_counter <= 3'd0;
                active <= 1'b1;
                data_out <= data_in[0];  // LSB first as per USB spec
            end else if (active) begin
                if (bit_counter < 3'd7) begin
                    bit_counter <= bit_counter + 1'b1;
                    data_out <= data_in[bit_counter + 1'b1];
                end else begin
                    active <= 1'b0;
                end
            end
        end
    end
endmodule