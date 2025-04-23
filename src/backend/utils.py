from amaranth import Elaboratable, Module, Signal, ResourceError

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
            except (ResourceError, AttributeError, IndexError): print(f"Note: LED {i} not available")
        return m

