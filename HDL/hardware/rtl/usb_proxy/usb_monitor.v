// usb_monitor.v (cleaned version for Yosys compatibility)

module usb_monitor (
    input  wire        clk,
    input  wire        clk_120mhz,
    input  wire        rst_n,
    // [other ports omitted for brevity...]
    output reg  [7:0]  status_register,
    output reg  [7:0]  status_read_data
);

    // [parameter and reg declarations...]

    // Monitor packet buffer
    (* ram_style = "distributed" *) reg [7:0] monitor_packet_buffer [0:255];

    // FSM and logic
    always @(posedge clk_120mhz or negedge rst_n) begin
        if (!rst_n) begin
            state <= ST_IDLE;
            status_register <= 8'b0;
            error_count <= 0;
            host_packets <= 0;
            device_packets <= 0;
            byte_counter <= 0;
            overflow_detected <= 0;
            device_speed <= 0;
            device_connected <= 0;
        end else begin
            // Update status flags in unified block
            status_register[STAT_CONNECTED]  <= device_connected;
            status_register[STAT_PROXY_EN]   <= proxy_enable;
            status_register[STAT_FILTER_EN]  <= packet_filter_en;
            status_register[STAT_MODIFY_EN]  <= modify_enable;
            {status_register[STAT_SPEED_HIGH], status_register[STAT_SPEED_FULL]} <= device_speed;
            status_register[STAT_OVERFLOW]   <= overflow_detected;
            status_register[STAT_ERROR]      <= 0;

            if ((host_rx_valid && !host_rx_crc_valid && host_rx_eop) ||
                (device_rx_valid && !device_rx_crc_valid && device_rx_eop)) begin
                error_count <= error_count + 1;
                status_register[STAT_ERROR] <= 1;
            end

            // [FSM transitions omitted for brevity...]
        end
    end

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
