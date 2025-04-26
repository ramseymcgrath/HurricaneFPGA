#!/usr/bin/env python3

from amaranth import (
    Elaboratable,
    Module,
    ClockDomain,
    ResetSignal,
    Const,
    ClockSignal,
    DomainRenamer,
    Signal,
)
from luna.gateware.interface.ulpi import UTMITranslator
from luna.gateware.usb.usb2.device import USBDevice
from luna.gateware.usb.usb2.endpoints.stream import (
    USBStreamInEndpoint,
    USBStreamOutEndpoint,
)
from luna.gateware.stream import StreamInterface
from luna import top_level_cli


class USBProxy(Elaboratable):
    """Cynthion USB HID Mouse Proxy: passes packets from real mouse to PC"""

    def elaborate(self, platform):
        m = Module()
        # Define clock domains connected to the platform clock
        m.domains.sync = ClockDomain()
        m.domains.usb = ClockDomain()  # Assuming USB domain runs at sync clock speed

        # ---- Grab ULPI PHYs ----
        aux_phy = platform.request("aux_phy")  # This talks to PC
        target_phy = platform.request("target_phy")  # This connects to mouse

        # ---- ULPI -> UTMI Translators ----
        aux_utmi = UTMITranslator(ulpi=aux_phy)
        target_utmi = UTMITranslator(ulpi=target_phy)

        # ---- PC-facing USB Device ----
        device = USBDevice(bus=aux_phy)

        # Add control endpoint (basic device descriptors)
        from usb_protocol.emitters import DeviceDescriptorCollection

        descriptors = DeviceDescriptorCollection()
        with descriptors.DeviceDescriptor() as d:
            d.idVendor = 0x1209
            d.idProduct = 0x5BF0
            d.iManufacturer = "Proxy"
            d.iProduct = "Mouse Proxy"
            d.iSerialNumber = "0001"
            d.bNumConfigurations = 1

        with descriptors.ConfigurationDescriptor() as c:
            with c.InterfaceDescriptor() as i:
                i.bInterfaceNumber = 0
                i.bInterfaceClass = 0x03  # HID
                i.bInterfaceSubclass = 0x01
                i.bInterfaceProtocol = 0x02  # Mouse
                with i.EndpointDescriptor() as e:
                    e.bEndpointAddress = 0x81
                    e.wMaxPacketSize = 8

        device.add_standard_control_endpoint(descriptors)

        # HID Mouse IN endpoint
        mouse_in_ep = USBStreamInEndpoint(endpoint_number=1, max_packet_size=8)
        device.add_endpoint(mouse_in_ep)

        # ---- Simple Packet Forwarder ----

        # Target RX: capture incoming packets from real mouse
        target_in = StreamInterface(payload_width=8)
        m.d.comb += [
            target_in.valid.eq(target_utmi.rx_valid),
            target_in.payload.eq(target_utmi.rx_data),
            target_utmi.tx_data.eq(0),
            target_utmi.tx_valid.eq(0),
        ]

        # AUX TX: push data to host
        m.d.comb += [
            mouse_in_ep.stream.valid.eq(target_in.valid),
            mouse_in_ep.stream.payload.eq(target_in.payload),
            target_in.ready.eq(mouse_in_ep.stream.ready),
        ]

        # ---- Always connect ----
        m.d.comb += device.connect.eq(1)

        return m


if __name__ == "__main__":
    top_level_cli(USBProxy)
