from amaranth import Elaboratable, Module, Signal, Cat
from amaranth.lib.fifo import SyncFIFOBuffered
# Assuming StreamInterface is defined as before
# from .passthrough import StreamInterface # Or wherever it's defined

class MouseCommandParser(Elaboratable):
    """
    Parses a 3-byte mouse command sequence (buttons, dx, dy) from
    an incoming byte stream. Operates in the 'sync' clock domain.

    Input Stream format:
        - byte 0: buttons
        - byte 1: dx
        - byte 2: dy

    Attributes
    ----------
    i_uart_stream : StreamInterface, input
        Stream carrying received UART bytes.
    o_buttons : Signal(8), output
        Parsed button state from the command sequence.
    o_dx : Signal(8), output
        Parsed dx value from the command sequence.
    o_dy : Signal(8), output
        Parsed dy value from the command sequence.
    o_cmd_ready : Signal, output
        Strobes high for one clock cycle when a complete 3-byte
        command sequence has been successfully parsed.
    """
    def __init__(self, input_fifo_depth=16):
        # Input Stream (sync domain)
        self.i_uart_stream = StreamInterface(payload_width=8)

        # Outputs (sync domain)
        self.o_buttons = Signal(8)
        self.o_dx = Signal(8)
        self.o_dy = Signal(8)
        self.o_cmd_ready = Signal() # Pulse high for one cycle when command complete

        self._input_fifo_depth = input_fifo_depth

    def elaborate(self, platform):
        m = Module()

        # --- Input FIFO (Buffers incoming stream bytes) ---
        # This decouples the FSM from the input stream timing slightly.
        # Can be made shallower if stream source is reliable.
        m.submodules.rx_fifo = rx_fifo = SyncFIFOBuffered(width=8, depth=self._input_fifo_depth)

        # Connect input stream to the FIFO write port
        m.d.comb += [
            rx_fifo.w_en.eq(self.i_uart_stream.valid),
            rx_fifo.w_data.eq(self.i_uart_stream.payload),
            self.i_uart_stream.ready.eq(rx_fifo.w_rdy),
        ]

        # --- Command Parsing FSM ---
        temp_buttons = Signal(8)
        temp_dx = Signal(8)

        # Default outputs
        m.d.comb += self.o_cmd_ready.eq(0) # o_cmd_ready is combinatorially pulsed

        # Control FIFO read enable
        # Read enable is asserted combinatorially based on state and fifo ready,
        # but registered assignment happens within the FSM sync block if needed,
        # however, safer to keep it combinatorial reacting to r_rdy.
        fifo_read_enable = Signal()
        m.d.comb += rx_fifo.r_en.eq(fifo_read_enable) # Connect combo signal to r_en


        with m.FSM(domain="sync", name="cmd_parse_fsm"):
            # IDLE: Wait for the first byte (buttons)
            with m.State("IDLE"):
                m.d.comb += fifo_read_enable.eq(0) # Default don't read
                with m.If(rx_fifo.r_rdy): # If a byte is available
                    m.d.comb += fifo_read_enable.eq(1) # Enable read for this cycle
                    m.d.sync += temp_buttons.eq(rx_fifo.r_data) # Latch data on next cycle
                    m.next = "WAIT_DX"

            # WAIT_DX: Wait for the second byte (dx)
            with m.State("WAIT_DX"):
                m.d.comb += fifo_read_enable.eq(0)
                with m.If(rx_fifo.r_rdy):
                    m.d.comb += fifo_read_enable.eq(1)
                    m.d.sync += temp_dx.eq(rx_fifo.r_data)
                    m.next = "WAIT_DY"

            # WAIT_DY: Wait for the third byte (dy)
            with m.State("WAIT_DY"):
                m.d.comb += fifo_read_enable.eq(0)
                with m.If(rx_fifo.r_rdy):
                    m.d.comb += fifo_read_enable.eq(1)
                    # Got the last byte - commit the command
                    m.d.sync += [
                        self.o_buttons.eq(temp_buttons),
                        self.o_dx.eq(temp_dx),
                        self.o_dy.eq(rx_fifo.r_data),
                    ]
                    # Pulse o_cmd_ready combinatorially for this cycle only
                    m.d.comb += self.o_cmd_ready.eq(1)
                    m.next = "IDLE" # Ready for the next command

        return m

# Remove the old UARTReceiver class entirely, as amaranth.lib.uart.AsyncSerialRX
# is used in the main USBPassthroughAnalyzer module.
# class UARTReceiver(Elaboratable):
#    ... (implementation removed) ...


from amaranth import Elaboratable, Module, Signal, Cat
from amaranth.lib.fifo import SyncFIFOBuffered
# Assuming StreamInterface is defined as before
# from .passthrough import StreamInterface # Or wherever it's defined

class MouseCommandParser(Elaboratable):
    """
    Parses a 3-byte mouse command sequence (buttons, dx, dy) from
    an incoming byte stream. Operates in the 'sync' clock domain.

    Input Stream format:
        - byte 0: buttons
        - byte 1: dx
        - byte 2: dy

    Attributes
    ----------
    i_uart_stream : StreamInterface, input
        Stream carrying received UART bytes.
    o_buttons : Signal(8), output
        Parsed button state from the command sequence.
    o_dx : Signal(8), output
        Parsed dx value from the command sequence.
    o_dy : Signal(8), output
        Parsed dy value from the command sequence.
    o_cmd_ready : Signal, output
        Strobes high for one clock cycle when a complete 3-byte
        command sequence has been successfully parsed.
    """
    def __init__(self, input_fifo_depth=16):
        # Input Stream (sync domain)
        self.i_uart_stream = StreamInterface(payload_width=8)

        # Outputs (sync domain)
        self.o_buttons = Signal(8)
        self.o_dx = Signal(8)
        self.o_dy = Signal(8)
        self.o_cmd_ready = Signal() # Pulse high for one cycle when command complete

        self._input_fifo_depth = input_fifo_depth

    def elaborate(self, platform):
        m = Module()

        # --- Input FIFO (Buffers incoming stream bytes) ---
        # This decouples the FSM from the input stream timing slightly.
        # Can be made shallower if stream source is reliable.
        m.submodules.rx_fifo = rx_fifo = SyncFIFOBuffered(width=8, depth=self._input_fifo_depth)

        # Connect input stream to the FIFO write port
        m.d.comb += [
            rx_fifo.w_en.eq(self.i_uart_stream.valid),
            rx_fifo.w_data.eq(self.i_uart_stream.payload),
            self.i_uart_stream.ready.eq(rx_fifo.w_rdy),
        ]

        # --- Command Parsing FSM ---
        temp_buttons = Signal(8)
        temp_dx = Signal(8)

        # Default outputs
        m.d.comb += self.o_cmd_ready.eq(0) # o_cmd_ready is combinatorially pulsed

        # Control FIFO read enable
        # Read enable is asserted combinatorially based on state and fifo ready,
        # but registered assignment happens within the FSM sync block if needed,
        # however, safer to keep it combinatorial reacting to r_rdy.
        fifo_read_enable = Signal()
        m.d.comb += rx_fifo.r_en.eq(fifo_read_enable) # Connect combo signal to r_en


        with m.FSM(domain="sync", name="cmd_parse_fsm"):
            # IDLE: Wait for the first byte (buttons)
            with m.State("IDLE"):
                m.d.comb += fifo_read_enable.eq(0) # Default don't read
                with m.If(rx_fifo.r_rdy): # If a byte is available
                    m.d.comb += fifo_read_enable.eq(1) # Enable read for this cycle
                    m.d.sync += temp_buttons.eq(rx_fifo.r_data) # Latch data on next cycle
                    m.next = "WAIT_DX"

            # WAIT_DX: Wait for the second byte (dx)
            with m.State("WAIT_DX"):
                m.d.comb += fifo_read_enable.eq(0)
                with m.If(rx_fifo.r_rdy):
                    m.d.comb += fifo_read_enable.eq(1)
                    m.d.sync += temp_dx.eq(rx_fifo.r_data)
                    m.next = "WAIT_DY"

            # WAIT_DY: Wait for the third byte (dy)
            with m.State("WAIT_DY"):
                m.d.comb += fifo_read_enable.eq(0)
                with m.If(rx_fifo.r_rdy):
                    m.d.comb += fifo_read_enable.eq(1)
                    # Got the last byte - commit the command
                    m.d.sync += [
                        self.o_buttons.eq(temp_buttons),
                        self.o_dx.eq(temp_dx),
                        self.o_dy.eq(rx_fifo.r_data),
                    ]
                    # Pulse o_cmd_ready combinatorially for this cycle only
                    m.d.comb += self.o_cmd_ready.eq(1)
                    m.next = "IDLE" # Ready for the next command

        return m

# Remove the old UARTReceiver class entirely, as amaranth.lib.uart.AsyncSerialRX
# is used in the main USBPassthroughAnalyzer module.
# class UARTReceiver(Elaboratable):
#    ... (implementation removed) ...
