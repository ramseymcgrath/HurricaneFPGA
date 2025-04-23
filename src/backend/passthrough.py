from amaranth import Elaboratable, Module, Signal, DomainRenamer, Cat, Record
from amaranth.build.res                  import ResourceError
from amaranth.lib.fifo import SyncFIFOBuffered, AsyncFIFOBuffered
from luna.gateware.interface.ulpi import UTMITranslator
from amaranth.lib.cdc import FFSynchronizer
from .mouse_injector import SimpleMouseInjector, MouseCommandParser
from .fifo import HyperRAMPacketFIFO

class USBPacketID:
    OUT = 0b0001
    IN = 0b1001
    SOF = 0b0101
    SETUP = 0b1101
    DATA0 = 0b0011
    DATA1 = 0b1011
    DATA2 = 0b0111
    MDATA = 0b1111

# --- StreamInterface and related classes remain the same ---
class StreamInterface:
    def __init__(self, payload_width=8):
        self.payload_width = payload_width
        self.valid = Signal()
        self.ready = Signal()
        self.payload = Signal(payload_width)
        self.first = Signal()
        self.last = Signal()

    def stream_eq(self, other, *, omit=None):
        if omit is None:
            omit = set()
        elif isinstance(omit, str):
            omit = {omit}
        connect_list = []
        if 'valid' not in omit:
            connect_list.append(other.valid.eq(self.valid))
        if 'payload' not in omit:
            connect_list.append(other.payload.eq(self.payload))
        if 'first' not in omit:
            connect_list.append(other.first.eq(self.first))
        if 'last' not in omit:
            connect_list.append(other.last.eq(self.last))
        # Add ready connection in reverse
        if 'ready' not in omit:
            connect_list.append(self.ready.eq(other.ready))
        return connect_list

class USBInStreamInterface(StreamInterface):
    def __init__(self, payload_width=8):
        super().__init__(payload_width)

class USBOutStreamInterface(StreamInterface):
    def __init__(self, payload_width=8):
        super().__init__(payload_width)

class PacketArbiter(Elaboratable): # Remains the same
    def __init__(self):
        self.passthrough_in = USBInStreamInterface()
        self.inject_in = USBInStreamInterface()
        self.merged_out = USBInStreamInterface()

    def elaborate(self, platform):
        m = Module()
        passthrough, inject, merged = self.passthrough_in, self.inject_in, self.merged_out
        # Ensure ready is propagated correctly in stream_eq
        m.d.comb += [
            merged.valid.eq(0), merged.payload.eq(0), merged.first.eq(0), merged.last.eq(0),
            passthrough.ready.eq(0), inject.ready.eq(0)
        ]
        with m.FSM(domain="usb", name="arbiter_fsm"):
            with m.State("IDLE"):
                m.d.comb += passthrough.ready.eq(0) # Explicitly deassert ready
                m.d.comb += inject.ready.eq(0)
                with m.If(inject.valid):
                    m.next = "FORWARD_INJECT"
                with m.Elif(passthrough.valid):
                    m.next = "FORWARD_PASSTHROUGH"
            with m.State("FORWARD_PASSTHROUGH"):
                # Use stream_eq which now includes ready
                m.d.comb += passthrough.stream_eq(merged)
                m.d.comb += inject.ready.eq(0) # Don't accept inject data now
                # State transition logic
                with m.If(merged.valid & merged.last & merged.ready):
                    m.next = "IDLE"
                with m.Elif(~merged.valid): # If source becomes invalid mid-packet (shouldn't happen ideally)
                    m.next = "IDLE"

            with m.State("FORWARD_INJECT"):
                # Use stream_eq which now includes ready
                m.d.comb += inject.stream_eq(merged)
                m.d.comb += passthrough.ready.eq(0) # Don't accept passthrough data now
                 # State transition logic
                with m.If(merged.valid & merged.last & merged.ready):
                    m.next = "IDLE"
                with m.Elif(~merged.valid): # If source becomes invalid mid-packet
                    m.next = "IDLE"
        return m


class USBPassthroughAnalyzer(Elaboratable):
    DEFAULT_UART_BAUD = 115200

    def __init__(self, uart_baud_rate = None):
        self.uart_baud_rate = uart_baud_rate if uart_baud_rate else self.DEFAULT_UART_BAUD

        # Outputs for status LEDs
        self.o_host_packet_activity = Signal() # Activity on AUX port (to/from PC)
        self.o_dev_packet_activity = Signal()  # Activity on TARGET port (to/from mouse)
        self.o_uart_rx_activity = Signal() # Activity on UART RX
        self.o_uart_tx_activity = Signal() # Activity on UART TX

    def elaborate(self, platform):
        m = Module()

        # --- Get Clock Frequency ---
        try:
            clk_freq = platform.lookup("clk_60MHz").clock.frequency
        except ResourceError:
             # Assume 60MHz if platform doesn't define it (e.g., simulation)
             clk_freq = 60e6
             print("WARNING: Platform does not define clk_60MHz frequency, assuming 60MHz.")

        # --- PHY Setup and Power Control ---
        use_hyperram = False # Default to false for simulation if platform/ram missing
        if platform is None:
             print("USBPassthroughAnalyzer: Running in offline/simulation mode. HyperRAM FIFOs disabled.")
             # Dummy setup for offline...
             ulpi_bus_layout = [ ("data", 8), ("clk", 1), ("dir", 1), ("nxt", 1), ("stp", 1), ("rst", 1)]
             # Host is now AUX
             aux_ulpi_res = Record([('i', ulpi_bus_layout), ('o', ulpi_bus_layout)])
             # Device is now TARGET
             target_ulpi_res = Record([('i', ulpi_bus_layout), ('o', ulpi_bus_layout)])

             # Instantiate translators with dummy resources
             m.submodules.host_translator = host_translator = DomainRenamer("sync")(UTMITranslator(ulpi=aux_ulpi_res))
             m.submodules.dev_translator = dev_translator = DomainRenamer("sync")(UTMITranslator(ulpi=target_ulpi_res))

             # Need dummy UART pins a VBUS enables if platform is None
             uart_pins = Record([("rx", [("i", 1)]), ("tx", [("o", 1)])])
             control_vbus_en = Record([("o", 1)])
             aux_vbus_en = Record([("o", 1)])

        else:
            # Request actual PHY resources
            # Host connection uses AUX port
            aux_ulpi_res = platform.request('aux_phy', 0)
            # Device connection uses TARGET port
            target_ulpi_res = platform.request('target_phy', 0)
            # UART uses dedicated pins
            uart_pins = platform.request("uart", 0)
            # VBUS input enable signals (Active Low)
            control_vbus_en = platform.request("control_vbus_in_en", 0)
            aux_vbus_en     = platform.request("aux_vbus_in_en", 0)


            # Instantiate translators with actual resources
            # Host translator connects to AUX PHY
            m.submodules.host_translator = host_translator = DomainRenamer("sync")(UTMITranslator(ulpi=aux_ulpi_res))
            # Device translator connects to TARGET PHY
            m.submodules.dev_translator = dev_translator = DomainRenamer("sync")(UTMITranslator(ulpi=target_ulpi_res))

            # Configure PHYs
            m.d.comb += [ host_translator.op_mode.eq(0b00), host_translator.xcvr_select.eq(0b01), host_translator.term_select.eq(1), host_translator.suspend.eq(0) ]
            m.d.comb += [ dev_translator.op_mode.eq(0b00), dev_translator.xcvr_select.eq(0b01), dev_translator.term_select.eq(1), dev_translator.suspend.eq(0) ]

            # Enable VBUS input from both CONTROL and AUX ports (Active LOW enable)
            m.d.comb += [
                control_vbus_en.o.eq(0), # Enable Control VBUS input
                aux_vbus_en.o.eq(0),     # Enable Aux VBUS input
            ]

            # Check if RAM resource exists before enabling HyperRAM
            try:
                platform.request('ram') # This will raise ResourceError if missing
                use_hyperram = True
                print("USBPassthroughAnalyzer: Found 'ram' resource. HyperRAM FIFOs will be used.")
            except ResourceError as e:
                print(f"USBPassthroughAnalyzer: ResourceError ('{e}'). HyperRAM FIFOs disabled (using fallback BRAM buffers).")
                use_hyperram = False # Disable if RAM is missing

        # --- FIFOs ---
        cdc_fifo_width = 10 # 8 data + first + last
        cdc_fifo_depth = 32 # Depth for domain crossing FIFOs

        # RX path FIFOs (sync -> usb)
        # Host data comes from AUX PHY
        m.submodules.host_rx_fifo = host_rx_fifo = AsyncFIFOBuffered(width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="sync", r_domain="usb")
        # Device data comes from TARGET PHY
        m.submodules.dev_rx_fifo = dev_rx_fifo = AsyncFIFOBuffered(width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="sync", r_domain="usb")

        # TX path CDC FIFOs (usb -> sync)
        # Host data goes to AUX PHY
        m.submodules.host_tx_cdc = host_tx_cdc = AsyncFIFOBuffered(width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="usb", r_domain="sync")
        # Device data goes to TARGET PHY
        m.submodules.dev_tx_cdc = dev_tx_cdc = AsyncFIFOBuffered(width=cdc_fifo_width, depth=cdc_fifo_depth, w_domain="usb", r_domain="sync")

        # HyperRAM FIFOs / Fallback Buffers (sync domain bulk storage)
        if use_hyperram:
            hyperram_out_buf_depth = 32
            # Host path (to/from AUX) uses host_hyperram_fifo
            m.submodules.host_hyperram_fifo = host_hyperram_fifo = HyperRAMPacketFIFO(out_fifo_depth=hyperram_out_buf_depth)
            # Device path (to/from TARGET) uses dev_hyperram_fifo
            m.submodules.dev_hyperram_fifo = dev_hyperram_fifo = HyperRAMPacketFIFO(out_fifo_depth=hyperram_out_buf_depth)
        else:
            sync_buffer_depth = 64 # Fallback BRAM depth
            # Host path buffer (to/from AUX)
            m.submodules.host_sync_buffer = host_sync_buffer = SyncFIFOBuffered(width=8, depth=sync_buffer_depth)
            # Device path buffer (to/from TARGET)
            m.submodules.dev_sync_buffer = dev_sync_buffer = SyncFIFOBuffered(width=8, depth=sync_buffer_depth)


        # --- Host RX Path (AUX PHY -> CDC -> USB Domain Logic) ---
        host_rx_first = Signal()
        host_rx_last = Signal()
        host_prev_rx_active = Signal(reset_less=True)
        m.d.sync += host_prev_rx_active.eq(host_translator.rx_active)
        m.d.comb += host_rx_first.eq(host_translator.rx_valid & (host_translator.rx_active & ~host_prev_rx_active))
        host_rx_last_comb = Signal()
        m.d.comb += host_rx_last_comb.eq(~host_translator.rx_active & host_prev_rx_active)
        m.d.sync += host_rx_last.eq(host_rx_last_comb & host_translator.rx_valid) # Align with valid data

        m.d.comb += [
            host_rx_fifo.w_en.eq(host_translator.rx_valid),
            host_rx_fifo.w_data.eq(Cat(host_translator.rx_data, host_rx_first, host_rx_last_comb)),
            host_translator.rx_ready.eq(host_rx_fifo.w_rdy), # Backpressure
        ]

        # --- Device RX Path (TARGET PHY -> CDC -> USB Domain Logic) ---
        dev_rx_first = Signal()
        dev_rx_last = Signal()
        dev_prev_rx_active = Signal(reset_less=True)
        m.d.sync += dev_prev_rx_active.eq(dev_translator.rx_active)
        m.d.comb += dev_rx_first.eq(dev_translator.rx_valid & (dev_translator.rx_active & ~dev_prev_rx_active))
        dev_rx_last_comb = Signal()
        m.d.comb += dev_rx_last_comb.eq(~dev_translator.rx_active & dev_prev_rx_active)
        m.d.sync += dev_rx_last.eq(dev_rx_last_comb & dev_translator.rx_valid) # Align with valid data

        m.d.comb += [
            dev_rx_fifo.w_en.eq(dev_translator.rx_valid),
            dev_rx_fifo.w_data.eq(Cat(dev_translator.rx_data, dev_rx_first, dev_rx_last_comb)),
            dev_translator.rx_ready.eq(dev_rx_fifo.w_rdy), # Backpressure
        ]

        # --- Stream Interfaces from RX FIFOs (Output in 'usb' domain) ---
        host_rx_stream_raw = StreamInterface(payload_width=cdc_fifo_width) # From AUX PHY
        dev_rx_stream_raw = StreamInterface(payload_width=cdc_fifo_width)  # From TARGET PGY

        m.d.comb += [
            host_rx_stream_raw.valid.eq(host_rx_fifo.r_rdy),
            host_rx_fifo.r_en.eq(host_rx_stream_raw.ready),
            host_rx_stream_raw.payload.eq(host_rx_fifo.r_data),
        ]
        m.d.comb += [
            dev_rx_stream_raw.valid.eq(dev_rx_fifo.r_rdy),
            dev_rx_fifo.r_en.eq(dev_rx_stream_raw.ready),
            dev_rx_stream_raw.payload.eq(dev_rx_fifo.r_data),
        ]

        # --- UART Setup (Hardware Interface) ---
        uart_divisor = int(clk_freq // self.uart_baud_rate)
        # RX path: Hardware pins -> AsyncSerialRX
        m.submodules.uart_rx = uart_rx = AsyncSerialRX(divisor=uart_divisor, pins=uart_pins, domain='sync')
        # TX path: AsyncSerialTX -> Hardware pins
        m.submodules.uart_tx = uart_tx = AsyncSerialTX(divisor=uart_divisor, pins=uart_pins, domain='sync')

        # --- Instantiate the Command Parser ---
        m.submodules.cmd_parser = cmd_parser = MouseCommandParser(input_fifo_depth=4) # Use smaller FIFO maybe

        # --- UART Data Flow ---
        # Connect Hardware UART RX output stream directly to Parser input stream
        m.d.comb += [
            cmd_parser.i_uart_stream.valid.eq(uart_rx.rdy),     # Valid byte received from HW UART
            cmd_parser.i_uart_stream.payload.eq(uart_rx.data),  # The byte itself
            uart_rx.ack.eq(cmd_parser.i_uart_stream.ready), # Acknowledge HW RX if parser FIFO can take it
        ]

        # Optional: UART TX path (perhaps for debug messages or acknowledging commands?)
        # Currently unconnected - parser doesn't generate TX data.
        # If you want to send data back, create a stream/FIFO feeding uart_tx.data/uart_tx.ack
        m.d.comb += [
            uart_tx.data.eq(0), # Default TX data
            uart_tx.ack.eq(0)   # Default don't send
            # uart_fifo.r_en.eq(uart_tx.rdy), # No echo FIFO anymore
        ]


        # --- Data Path Logic (USB domain) ---
        m.submodules.injector = injector = SimpleMouseInjector()
        m.submodules.arbiter = arbiter = PacketArbiter()

        # Connect PARSER outputs to the INJECTOR inputs
        # Need CDC for cmd_parser outputs (sync) to injector inputs (usb)
        # Option 1: Simple FFSynchronizer for trigger
        m.submodules.trigger_cdc = FFSynchronizer(i=cmd_parser.o_cmd_ready, o=injector.trigger, o_domain='usb')

        # Option 2: Buffer parsed commands via another Async FIFO if backpressure needed
        # Or assume injector consumes immediately triggered (simpler, check timing)
        # Let's try direct connection with FFSync for signals crossing domain boundary
        cmd_buttons_usb = Signal(8)
        cmd_dx_usb = Signal(8)
        cmd_dy_usb = Signal(8)
        m.submodules.buttons_cdc = FFSynchronizer(i=cmd_parser.o_buttons, o=cmd_buttons_usb, o_domain='usb')
        m.submodules.dx_cdc = FFSynchronizer(i=cmd_parser.o_dx, o=cmd_dx_usb, o_domain='usb')
        m.submodules.dy_cdc = FFSynchronizer(i=cmd_parser.o_dy, o=cmd_dy_usb, o_domain='usb')

        m.d.comb += [
            # injector.trigger is connected via trigger_cdc
            injector.buttons.eq(cmd_buttons_usb),
            injector.dx.eq(cmd_dx_usb),
            injector.dy.eq(cmd_dy_usb)
        ]

        # Convert 10-bit streams to standard 8-bit payload streams
        # Data stream coming FROM the host (PC via AUX PHY)
        host_logic_in_stream = USBOutStreamInterface() # Terminology: OUT of PC
        # Data stream coming FROM the device (Mouse via TARGET PHY)
        dev_logic_in_stream = USBInStreamInterface() # Terminology: IN to PC

        m.d.comb += [
            host_logic_in_stream.valid.eq(host_rx_stream_raw.valid),
            host_logic_in_stream.payload.eq(host_rx_stream_raw.payload[0:8]),
            host_logic_in_stream.first.eq(host_rx_stream_raw.payload[8]),
            host_logic_in_stream.last.eq(host_rx_stream_raw.payload[9]),
            host_rx_stream_raw.ready.eq(host_logic_in_stream.ready),
        ]
        m.d.comb += [
            dev_logic_in_stream.valid.eq(dev_rx_stream_raw.valid),
            dev_logic_in_stream.payload.eq(dev_rx_stream_raw.payload[0:8]),
            dev_logic_in_stream.first.eq(dev_rx_stream_raw.payload[8]),
            dev_logic_in_stream.last.eq(dev_rx_stream_raw.payload[9]),
            dev_rx_stream_raw.ready.eq(dev_logic_in_stream.ready),
        ]

        # --- Data Path Logic (Mostly 'usb' domain) ---
        m.submodules.injector = injector = SimpleMouseInjector()
        m.submodules.arbiter = arbiter = PacketArbiter() # Arbitrates DUT data and injected data

        m.d.comb += [
            injector.trigger.eq(self.i_inject_trigger),
            injector.buttons.eq(self.i_buttons),
            injector.dx.eq(self.i_dx),
            injector.dy.eq(self.i_dy)
        ]

        # --- Host -> Device Path (AUX -> TARGET) ---
        # Data from PC (host_logic_in_stream) goes directly to the device TX CDC fifo
        dev_tx_cdc_input_stream = StreamInterface(payload_width=8)
        m.d.comb += host_logic_in_stream.stream_eq(dev_tx_cdc_input_stream)

        m.d.comb += [
            dev_tx_cdc.w_en.eq(dev_tx_cdc_input_stream.valid),
            dev_tx_cdc.w_data.eq(Cat(dev_tx_cdc_input_stream.payload, dev_tx_cdc_input_stream.first, dev_tx_cdc_input_stream.last)),
            dev_tx_cdc_input_stream.ready.eq(dev_tx_cdc.w_rdy),
        ]


        # --- Device -> Host Path (TARGET -> AUX) ---
        # Data from Mouse (dev_logic_in_stream) and Injector go into Arbiter
        m.d.comb += dev_logic_in_stream.stream_eq(arbiter.passthrough_in)
        m.d.comb += injector.source.stream_eq(arbiter.inject_in)

        # Arbiter output goes to the host TX CDC fifo
        arbiter_out_stream = arbiter.merged_out
        m.d.comb += [
            host_tx_cdc.w_en.eq(arbiter_out_stream.valid),
            host_tx_cdc.w_data.eq(Cat(arbiter_out_stream.payload, arbiter_out_stream.first, arbiter_out_stream.last)),
            arbiter_out_stream.ready.eq(host_tx_cdc.w_rdy)
        ]


        # --- TX Path CDC FIFOs -> HyperRAM/Buffer -> PHYs (All 'sync' domain from here) ---
        host_tx_cdc_out_stream = StreamInterface(payload_width=cdc_fifo_width) # Data going TO host (AUX)
        dev_tx_cdc_out_stream = StreamInterface(payload_width=cdc_fifo_width)  # Data going TO device (TARGET)

        m.d.comb += [
            host_tx_cdc_out_stream.valid.eq(host_tx_cdc.r_rdy),
            host_tx_cdc.r_en.eq(host_tx_cdc_out_stream.ready),
            host_tx_cdc_out_stream.payload.eq(host_tx_cdc.r_data),
        ]
        m.d.comb += [
            dev_tx_cdc_out_stream.valid.eq(dev_tx_cdc.r_rdy),
            dev_tx_cdc.r_en.eq(dev_tx_cdc_out_stream.ready),
            dev_tx_cdc_out_stream.payload.eq(dev_tx_cdc.r_data),
        ]


        if use_hyperram:
            # --- HyperRAM Path ---
            host_hyperram_in_stream = StreamInterface(payload_width=16) # Input to host FIFO (to AUX)
            dev_hyperram_in_stream = StreamInterface(payload_width=16)  # Input to device FIFO (to TARGET)

            # Adapter: CDC Output -> HyperRAM Input (Host Path - To AUX PHY)
            m.d.comb += [
                host_hyperram_in_stream.valid.eq(host_tx_cdc_out_stream.valid),
                host_hyperram_in_stream.payload.eq(Cat(host_tx_cdc_out_stream.payload[0:8], C(0, 8))),
                host_hyperram_in_stream.first.eq(host_tx_cdc_out_stream.payload[8]),
                host_hyperram_in_stream.last.eq(host_tx_cdc_out_stream.payload[9]),
                host_tx_cdc_out_stream.ready.eq(host_hyperram_in_stream.ready),
                host_hyperram_fifo.input.stream_eq(host_hyperram_in_stream)
            ]
            # Adapter: CDC Output -> HyperRAM Input (Device Path - To TARGET PHY)
            m.d.comb += [
                dev_hyperram_in_stream.valid.eq(dev_tx_cdc_out_stream.valid),
                dev_hyperram_in_stream.payload.eq(Cat(dev_tx_cdc_out_stream.payload[0:8], C(0, 8))),
                dev_hyperram_in_stream.first.eq(dev_tx_cdc_out_stream.payload[8]),
                dev_hyperram_in_stream.last.eq(dev_tx_cdc_out_stream.payload[9]),
                dev_tx_cdc_out_stream.ready.eq(dev_hyperram_in_stream.ready),
                dev_hyperram_fifo.input.stream_eq(dev_hyperram_in_stream)
            ]

            # Connect HyperRAM output to PHY TX
            # Host Path (To AUX PHY)
            m.d.comb += [
                host_translator.tx_data.eq(host_hyperram_fifo.output.payload[0:8]),
                host_translator.tx_valid.eq(host_hyperram_fifo.output.valid),
                host_hyperram_fifo.output.ready.eq(host_translator.tx_ready),
            ]
            # Device Path (To TARGET PHY)
            m.d.comb += [
                dev_translator.tx_data.eq(dev_hyperram_fifo.output.payload[0:8]),
                dev_translator.tx_valid.eq(dev_hyperram_fifo.output.valid),
                dev_hyperram_fifo.output.ready.eq(dev_translator.tx_ready),
            ]

        else:
            # --- Non-HyperRAM Fallback Path (CDC -> Sync Buffer -> PHY TX) ---
            # Host Path (To AUX PHY)
            m.d.comb += [
                host_sync_buffer.w_en.eq(host_tx_cdc_out_stream.valid),
                host_sync_buffer.w_data.eq(host_tx_cdc_out_stream.payload[0:8]),
                host_tx_cdc_out_stream.ready.eq(host_sync_buffer.w_rdy),
            ]
             # Device Path (To TARGET PHY)
            m.d.comb += [
                dev_sync_buffer.w_en.eq(dev_tx_cdc_out_stream.valid),
                dev_sync_buffer.w_data.eq(dev_tx_cdc_out_stream.payload[0:8]),
                dev_tx_cdc_out_stream.ready.eq(dev_sync_buffer.w_rdy),
            ]

            # Connect Sync Buffer Output to PHY TX
            # Host Path (To AUX PHY)
            m.d.comb += [
                host_translator.tx_data.eq(host_sync_buffer.r_data),
                host_translator.tx_valid.eq(host_sync_buffer.r_rdy),
                host_sync_buffer.r_en.eq(host_translator.tx_ready),
            ]
             # Device Path (To TARGET PHY)
            m.d.comb += [
                dev_translator.tx_data.eq(dev_sync_buffer.r_data),
                dev_translator.tx_valid.eq(dev_sync_buffer.r_rdy),
                dev_sync_buffer.r_en.eq(dev_translator.tx_ready),
            ]
        activity_duration = int(clk_freq * 0.05)
        host_activity_counter = Signal(range(activity_duration))
        dev_activity_counter = Signal(range(activity_duration))
        uart_rx_activity_counter = Signal(range(activity_duration))
        uart_tx_activity_counter = Signal(range(activity_duration)) # May not light up if TX unused

        # ... (Host/Device activity logic based on host_rx_fifo.w_en / dev_rx_fifo.w_en) ...

        # UART RX activity (based on hardware UART RX ready)
        with m.If(uart_rx.rdy):
            m.d.sync += uart_rx_activity_counter.eq(activity_duration - 1)
        with m.Elif(uart_rx_activity_counter > 0):
            m.d.sync += uart_rx_activity_counter.eq(uart_rx_activity_counter - 1)
        m.d.comb += self.o_uart_rx_activity.eq(uart_rx_activity_counter > 0)

        # UART TX activity (based on Hardware UART TX taking data)
        with m.If(uart_tx.ack & uart_tx.rdy): # If TX accepts data
            m.d.sync += uart_tx_activity_counter.eq(activity_duration - 1)
        with m.Elif(uart_tx_activity_counter > 0):
            m.d.sync += uart_tx_activity_counter.eq(uart_tx_activity_counter - 1)
        m.d.comb += self.o_uart_tx_activity.eq(uart_tx_activity_counter > 0)

        m.d.comb += [
            self.o_host_packet_activity.eq(host_activity_counter > 0),
            self.o_dev_packet_activity.eq(dev_activity_counter > 0),
        ]

        return m
