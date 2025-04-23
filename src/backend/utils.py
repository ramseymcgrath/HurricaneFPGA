#!/usr/bin/env python3

# -*- coding: utf-8 -*-

from amaranth import Elaboratable, Module, Signal, Cat, C, Record
from amaranth.build.res import ResourceError
from amaranth.lib.cdc import FFSynchronizer
from amaranth.utils import bits_for

__all__ = [
    "AsyncSerialRX",
    "AsyncSerialTX",
    "AsyncSerial",
    "LEDController",
    "ActivityMonitor",
]


class LEDController(Elaboratable):
    def __init__(self, platform, num_leds=4):
        self.platform = platform
        self.num_leds = num_leds
        self.led_outputs = [Signal(name=f"led_{i}") for i in range(self.num_leds)]

    def elaborate(self, platform):
        m = Module()
        for i in range(self.num_leds):
            try:
                led_pin = platform.request("led", i)
                m.d.comb += led_pin.o.eq(self.led_outputs[i])
            except (ResourceError, AttributeError, IndexError):
                print(f"Note: LED {i} not available")
        return m


class ActivityMonitor(Elaboratable):
    """Handles activity monitoring for LEDs or other indicators.
    Detects activity on various signals and extends the indication for a configurable duration.
    """

    def __init__(self, clk_freq, activity_duration_ms=50):
        self.clk_freq = clk_freq
        self.activity_duration = int(clk_freq * (activity_duration_ms / 1000))

        # Activity signals (inputs)
        self.i_host_activity = Signal()  # Activity on host (PC) side
        self.i_device_activity = Signal()  # Activity on device side
        self.i_uart_rx_activity = Signal()  # UART RX activity
        self.i_uart_tx_activity = Signal()  # UART TX activity

        # Output signals (to LEDs)
        self.o_host_activity = Signal()
        self.o_device_activity = Signal()
        self.o_uart_rx_activity = Signal()
        self.o_uart_tx_activity = Signal()

    def elaborate(self, platform):
        m = Module()

        # Activity counters
        host_activity_counter = Signal(range(self.activity_duration))
        dev_activity_counter = Signal(range(self.activity_duration))
        uart_rx_activity_counter = Signal(range(self.activity_duration))
        uart_tx_activity_counter = Signal(range(self.activity_duration))

        # Host activity logic
        with m.If(self.i_host_activity):
            m.d.sync += host_activity_counter.eq(self.activity_duration - 1)
        with m.Elif(host_activity_counter > 0):
            m.d.sync += host_activity_counter.eq(host_activity_counter - 1)

        # Device activity logic
        with m.If(self.i_device_activity):
            m.d.sync += dev_activity_counter.eq(self.activity_duration - 1)
        with m.Elif(dev_activity_counter > 0):
            m.d.sync += dev_activity_counter.eq(dev_activity_counter - 1)

        # UART RX activity logic
        with m.If(self.i_uart_rx_activity):
            m.d.sync += uart_rx_activity_counter.eq(self.activity_duration - 1)
        with m.Elif(uart_rx_activity_counter > 0):
            m.d.sync += uart_rx_activity_counter.eq(uart_rx_activity_counter - 1)

        # UART TX activity logic
        with m.If(self.i_uart_tx_activity):
            m.d.sync += uart_tx_activity_counter.eq(self.activity_duration - 1)
        with m.Elif(uart_tx_activity_counter > 0):
            m.d.sync += uart_tx_activity_counter.eq(uart_tx_activity_counter - 1)

        # Connect counters to output signals
        m.d.comb += [
            self.o_host_activity.eq(host_activity_counter > 0),
            self.o_device_activity.eq(dev_activity_counter > 0),
            self.o_uart_rx_activity.eq(uart_rx_activity_counter > 0),
            self.o_uart_tx_activity.eq(uart_tx_activity_counter > 0),
        ]

        return m


def _check_divisor(divisor, bound):
    if divisor < bound:
        raise ValueError(
            "Invalid divisor {!r}; must be greater than or equal to {}".format(
                divisor, bound
            )
        )


def _check_parity(parity):
    choices = ("none", "mark", "space", "even", "odd")
    if parity not in choices:
        raise ValueError(
            "Invalid parity {!r}; must be one of {}".format(parity, ", ".join(choices))
        )


def _compute_parity_bit(data, parity):
    if parity == "none":
        return C(0, 0)
    if parity == "mark":
        return C(1, 1)
    if parity == "space":
        return C(0, 1)
    if parity == "even":
        return data.xor()
    if parity == "odd":
        return ~data.xor()
    assert False


def _wire_layout(data_bits, parity="none"):
    return [
        ("start", 1),
        ("data", data_bits),
        ("parity", 0 if parity == "none" else 1),
        ("stop", 1),
    ]


class AsyncSerialRX(Elaboratable):
    """Asynchronous serial receiver.

    Parameters
    ----------
    divisor : int
        Clock divisor reset value. Should be set to ``int(clk_frequency // baudrate)``.
    divisor_bits : int
        Optional. Clock divisor width. If omitted, ``bits_for(divisor)`` is used instead.
    data_bits : int
        Data width.
    parity : ``"none"``, ``"mark"``, ``"space"``, ``"even"``, ``"odd"``
        Parity mode.
    pins : :class:`amaranth.lib.io.Pin`
        Optional. UART pins. See :class:`amaranth_boards.resources.UARTResource` for layout.

    Attributes
    ----------
    divisor : Signal, in
        Clock divisor.
    data : Signal, out
        Read data. Valid only when ``rdy`` is asserted.
    err.overflow : Signal, out
        Error flag. A new frame has been received, but the previous one was not acknowledged.
    err.frame : Signal, out
        Error flag. The received bits do not fit in a frame.
    err.parity : Signal, out
        Error flag. The parity check has failed.
    rdy : Signal, out
        Read strobe.
    ack : Signal, in
        Read acknowledge. Must be held asserted while data can be read out of the receiver.
    i : Signal, in
        Serial input. If ``pins`` has been specified, ``pins.rx.i`` drives it.
    """

    def __init__(
        self, *, divisor, divisor_bits=None, data_bits=8, parity="none", pins=None
    ):
        _check_parity(parity)
        self._parity = parity

        # The clock divisor must be at least 5 to keep the FSM synchronized with the serial input
        # during a DONE->IDLE->BUSY transition.
        _check_divisor(divisor, 5)
        self.divisor = Signal(divisor_bits or bits_for(divisor), reset=divisor)

        self.data = Signal(data_bits)
        self.err = Record(
            [
                ("overflow", 1),
                ("frame", 1),
                ("parity", 1),
            ]
        )
        self.rdy = Signal()
        self.ack = Signal()

        self.i = Signal(reset=1)

        self._pins = pins

    def elaborate(self, platform):
        m = Module()

        timer = Signal.like(self.divisor)
        shreg = Record(_wire_layout(len(self.data), self._parity))
        bitno = Signal(range(len(shreg)))

        if self._pins is not None:
            m.submodules += FFSynchronizer(self._pins.rx.i, self.i, reset=1)

        with m.FSM() as fsm:
            with m.State("IDLE"):
                with m.If(~self.i):
                    m.d.sync += [
                        bitno.eq(len(shreg) - 1),
                        timer.eq(self.divisor >> 1),
                    ]
                    m.next = "BUSY"

            with m.State("BUSY"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer - 1)
                with m.Else():
                    m.d.sync += [
                        shreg.eq(Cat(shreg[1:], self.i)),
                        bitno.eq(bitno - 1),
                        timer.eq(self.divisor - 1),
                    ]
                    with m.If(bitno == 0):
                        m.next = "DONE"

            with m.State("DONE"):
                with m.If(self.ack):
                    m.d.sync += [
                        self.data.eq(shreg.data),
                        self.err.frame.eq(~((shreg.start == 0) & (shreg.stop == 1))),
                        self.err.parity.eq(
                            ~(
                                shreg.parity
                                == _compute_parity_bit(shreg.data, self._parity)
                            )
                        ),
                    ]
                m.d.sync += self.err.overflow.eq(~self.ack)
                m.next = "IDLE"

        with m.If(self.ack):
            m.d.sync += self.rdy.eq(fsm.ongoing("DONE"))

        return m


class AsyncSerialTX(Elaboratable):
    """Asynchronous serial transmitter.

    Parameters
    ----------
    divisor : int
        Clock divisor reset value. Should be set to ``int(clk_frequency // baudrate)``.
    divisor_bits : int
        Optional. Clock divisor width. If omitted, ``bits_for(divisor)`` is used instead.
    data_bits : int
        Data width.
    parity : ``"none"``, ``"mark"``, ``"space"``, ``"even"``, ``"odd"``
        Parity mode.
    pins : :class:`amaranth.lib.io.Pin`
        Optional. UART pins. See :class:`amaranth_boards.resources.UARTResource` for layout.

    Attributes
    ----------
    divisor : Signal, in
        Clock divisor.
    data : Signal, in
        Write data. Valid only when ``ack`` is asserted.
    rdy : Signal, out
        Write ready. Asserted when the transmitter is ready to transmit data.
    ack : Signal, in
        Write strobe. Data gets transmitted when both ``rdy`` and ``ack`` are asserted.
    o : Signal, out
        Serial output. If ``pins`` has been specified, it drives ``pins.tx.o``.
    """

    def __init__(
        self, *, divisor, divisor_bits=None, data_bits=8, parity="none", pins=None
    ):
        _check_parity(parity)
        self._parity = parity

        _check_divisor(divisor, 1)
        self.divisor = Signal(divisor_bits or bits_for(divisor), reset=divisor)

        self.data = Signal(data_bits)
        self.rdy = Signal()
        self.ack = Signal()

        self.o = Signal(reset=1)

        self._pins = pins

    def elaborate(self, platform):
        m = Module()

        timer = Signal.like(self.divisor)
        shreg = Record(_wire_layout(len(self.data), self._parity))
        bitno = Signal(range(len(shreg)))

        if self._pins is not None:
            m.d.comb += self._pins.tx.o.eq(self.o)

        with m.FSM():
            with m.State("IDLE"):
                m.d.comb += self.rdy.eq(1)
                with m.If(self.ack):
                    m.d.sync += [
                        shreg.start.eq(0),
                        shreg.data.eq(self.data),
                        shreg.parity.eq(_compute_parity_bit(self.data, self._parity)),
                        shreg.stop.eq(1),
                        bitno.eq(len(shreg) - 1),
                        timer.eq(self.divisor - 1),
                    ]
                    m.next = "BUSY"

            with m.State("BUSY"):
                with m.If(timer != 0):
                    m.d.sync += timer.eq(timer - 1)
                with m.Else():
                    m.d.sync += [
                        Cat(self.o, shreg).eq(shreg),
                        bitno.eq(bitno - 1),
                        timer.eq(self.divisor - 1),
                    ]
                    with m.If(bitno == 0):
                        m.next = "IDLE"

        return m


class AsyncSerial(Elaboratable):
    """Asynchronous serial transceiver.

    Parameters
    ----------
    divisor : int
        Clock divisor reset value. Should be set to ``int(clk_frequency // baudrate)``.
    divisor_bits : int
        Optional. Clock divisor width. If omitted, ``bits_for(divisor)`` is used instead.
    data_bits : int
        Data width.
    parity : ``"none"``, ``"mark"``, ``"space"``, ``"even"``, ``"odd"``
        Parity mode.
    pins : :class:`amaranth.lib.io.Pin`
        Optional. UART pins. See :class:`amaranth_boards.resources.UARTResource` for layout.

    Attributes
    ----------
    divisor : Signal, in
        Clock divisor.
    rx : :class:`AsyncSerialRX`
        See :class:`AsyncSerialRX`.
    tx : :class:`AsyncSerialTX`
        See :class:`AsyncSerialTX`.
    """

    def __init__(self, *, divisor, divisor_bits=None, **kwargs):
        self.divisor = Signal(divisor_bits or bits_for(divisor), reset=divisor)

        self.rx = AsyncSerialRX(divisor=divisor, divisor_bits=divisor_bits, **kwargs)
        self.tx = AsyncSerialTX(divisor=divisor, divisor_bits=divisor_bits, **kwargs)

    def elaborate(self, platform):
        m = Module()
        m.submodules.rx = self.rx
        m.submodules.tx = self.tx
        m.d.comb += [
            self.rx.divisor.eq(self.divisor),
            self.tx.divisor.eq(self.divisor),
        ]
        return m
