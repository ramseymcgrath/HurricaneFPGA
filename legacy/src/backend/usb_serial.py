#!/usr/bin/env python3

# -*- coding: utf-8 -*-

from amaranth import Elaboratable, Module, Signal, DomainRenamer, Cat
from luna.gateware.usb.usb2.device import USBDevice
from luna.gateware.usb.usb2.request import USBRequestHandler
from luna.gateware.usb.usb2.endpoints.stream import (
    USBStreamInEndpoint,
    USBStreamOutEndpoint,
)
from luna.gateware.stream import StreamInterface

from luna.gateware.usb.usb2.descriptor import *

__all__ = ["USBSerialDevice"]


class USBSerialDevice(Elaboratable):
    """Device that acts as a CDC-ACM 'serial converter'.

    Exposes a stream interface.

    Attributes
    ----------
    connect: Signal(), input
        When asserted, the USB-to-serial device will be presented to the host
        and allowed to communicate.
    rx: StreamInterface(), output stream
        A stream carrying data received from the host.
    tx: StreamInterface(), input stream
        A stream carrying data to be transmitted to the host.

    Parameters
    ----------
    bus: Record()
        The raw input record that provides our USB connection. Should be a connection to a USB PHY,
        SerDes, or raw USB lines as described at: https://luna.readthedocs.io/en/latest/custom_hardware.html.
    idVendor: int, <65536
        The Vendor ID that should be presented for the relevant USB device.
    idProduct: int, <65536
        The Product ID that should be presented for the relevant USB device.

    manufacturer_string: str, optional
        A string describing this device's manufacturer.
    product_str: str, optional
        A string describing this device.
    serial_number: str, optional
        A string describing this device's serial number.

    max_packet_size: int in {64, 246, 512}, optional
        The maximum packet size for communications.
    """

    _STATUS_ENDPOINT_NUMBER = 5
    _DATA_ENDPOINT_NUMBER = 6

    def __init__(
        self,
        *,
        bus,
        idVendor,
        idProduct,
        manufacturer_string="LUNA",
        product_string="USB-to-serial",
        serial_number="",
        max_packet_size=64
    ):

        self._bus = bus
        self._idVendor = idVendor
        self._idProduct = idProduct
        self._manufacturer_string = manufacturer_string
        self._product_string = product_string
        self._serial_number = serial_number
        self._max_packet_size = max_packet_size

        #
        # I/O port
        #
        self.connect = Signal()
        self.rx = StreamInterface()
        self.tx = StreamInterface()

    def create_descriptors(self):
        """Creates the descriptors that describe our serial topology."""

        descriptors = DeviceDescriptorCollection()

        # Create a device descriptor with our user parameters...
        with descriptors.DeviceDescriptor() as d:
            d.idVendor = self._idVendor
            d.idProduct = self._idProduct

            d.iManufacturer = self._manufacturer_string
            d.iProduct = self._product_string
            d.iSerialNumber = self._serial_number

            d.bNumConfigurations = 1

        # ... and then describe our CDC-ACM setup.
        with descriptors.ConfigurationDescriptor() as c:

            # First, we'll describe the Communication Interface, which contains most
            # of our description; but also an endpoint that does effectively nothing in
            # our case, since we don't have interrupts we want to send up to the host.
            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 0

                i.bInterfaceClass = 0x02  # CDC
                i.bInterfaceSubclass = 0x02  # ACM
                i.bInterfaceProtocol = 0x01  # AT commands / UART

                # Provide the default CDC version.
                i.add_subordinate_descriptor(HeaderDescriptorEmitter())

                # ... specify our interface associations ...
                union = UnionFunctionalDescriptorEmitter()
                union.bControlInterface = 0
                union.bSubordinateInterface0 = 1
                i.add_subordinate_descriptor(union)

                # ... and specify the interface that'll carry our data...
                call_management = CallManagementFunctionalDescriptorEmitter()
                call_management.bDataInterface = 1
                i.add_subordinate_descriptor(call_management)

                # CDC communications endpoint
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x80 | self._STATUS_ENDPOINT_NUMBER  # EP5 IN
                    e.bmAttributes = 0x03
                    e.wMaxPacketSize = self._max_packet_size
                    e.bInterval = 11

            # Finally, we'll describe the communications interface, which just has the
            # endpoints for our data in and out.
            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 1
                i.bInterfaceClass = 0x0A  # CDC data
                i.bInterfaceSubclass = 0x00
                i.bInterfaceProtocol = 0x00

                # Data IN to host (tx, from our side)
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x80 | self._DATA_ENDPOINT_NUMBER  # EP6 IN
                    e.wMaxPacketSize = self._max_packet_size

                # Data OUT from host (rx, from our side)
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = self._DATA_ENDPOINT_NUMBER  # EP6 OUT
                    e.wMaxPacketSize = self._max_packet_size

        return descriptors

    def elaborate(self, platform):
        m = Module()

        # Create our USB device implementation.
        m.submodules.usb = usb = DomainRenamer("usb")(USBDevice(bus=self._bus))

        # Add our standard control endpoint to the device.
        descriptors = self.create_descriptors()
        control_ep = usb.add_standard_control_endpoint(descriptors)

        # Create our CDC-ACM endpoints...
        status_ep = USBStreamInEndpoint(
            endpoint_number=self._STATUS_ENDPOINT_NUMBER,
            max_packet_size=self._max_packet_size,
        )
        in_ep = USBStreamInEndpoint(
            endpoint_number=self._DATA_ENDPOINT_NUMBER,
            max_packet_size=self._max_packet_size,
        )
        out_ep = USBStreamOutEndpoint(
            endpoint_number=self._DATA_ENDPOINT_NUMBER,
            max_packet_size=self._max_packet_size,
        )

        # Add them to our device.
        usb.add_endpoint(status_ep)
        usb.add_endpoint(in_ep)
        usb.add_endpoint(out_ep)

        # CDC-specific requests handlers
        class ACMRequestHandler(USBRequestHandler):
            # USB-CDC requests
            SET_LINE_CODING = 0x20
            GET_LINE_CODING = 0x21
            SET_CONTROL_LINE_STATE = 0x22

            def elaborate(self, platform):
                m = Module()
                interface = self.interface
                setup = self.interface.setup

                # Stash the relevant setup information
                bRequest = setup.request
                is_device_to_host = setup.is_in_request()

                # Handle each request valid for our interface
                with m.If(setup.type == USBRequestHandler.TYPE_CLASS):
                    with m.Switch(bRequest):

                        # Line Coding requests - respond with default values (115200, 8N1)
                        with m.Case(self.GET_LINE_CODING):
                            with m.If(is_device_to_host):
                                m.d.comb += [
                                    # Send default line coding: 115200 baud, 8-N-1
                                    interface.tx.valid.eq(1),
                                    interface.tx.data.eq(
                                        Cat(
                                            # Rate: 115200 bps
                                            Signal(
                                                8, reset=0x00
                                            ),  # 115200 = 0x0001C200, little-endian
                                            Signal(8, reset=0xC2),
                                            Signal(8, reset=0x01),
                                            Signal(8, reset=0x00),
                                            # Stop Bits: 1
                                            Signal(8, reset=0x00),
                                            # Parity: None
                                            Signal(8, reset=0x00),
                                            # Data Bits: 8
                                            Signal(8, reset=0x08),
                                        )
                                    ),
                                    interface.handshakes_out.ack.eq(1),
                                ]

                        # Line Coding - we acknowledge but don't store it
                        with m.Case(self.SET_LINE_CODING):
                            with m.If(~is_device_to_host):
                                m.d.comb += [interface.handshakes_out.ack.eq(1)]

                        # Control Line State - we acknowledge
                        with m.Case(self.SET_CONTROL_LINE_STATE):
                            m.d.comb += [interface.handshakes_out.ack.eq(1)]

                return m

        control_ep.add_request_handler(ACMRequestHandler())

        # Connect up our endpoints to our I/O ports.
        m.d.comb += [
            # Connect our endpoints to each other and to our I/O.
            status_ep.stream.ready.eq(1),
            # Connect data IN endpoint to our stream input
            in_ep.stream.stream_eq(self.tx),
            # Connect data OUT endpoint to our stream output
            self.rx.stream_eq(out_ep.stream),
            # Only allow this device to respond once the user requests connection.
            usb.connect.eq(self.connect),
        ]

        return m
