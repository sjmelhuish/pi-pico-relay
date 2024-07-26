import asyncio
import time

import framebuf
import micropython
from machine import I2C, Pin, Timer

from graphics import framebuffers

import my_secrets
from mqtt_as import MQTTClient, config
from ssd1306 import SSD1306_I2C

# Required on Pyboard D and ESP32. On ESP8266 these may be omitted (see above).
config["ssid"] = my_secrets.SSID
config["wifi_pw"] = my_secrets.PASSWORD
config["server"] = my_secrets.MQTT_SERVER
config["user"] = my_secrets.MQTT_USER
config["password"] = my_secrets.MQTT_PASSWORD
config["queue_len"] = 1  # Use event interface with default queue size

micropython.alloc_emergency_exception_buf(100)

RELAY_STATE_INVALID = 0
RELAY_STATE_THROUGH = 1
RELAY_STATE_CROSS = 2

RELAY_THRU = 1
RELAY_CROSS = 2

unit_number = 1  # Serial number for this relay


class Oled:
    """Class to display text and graphics on an I2C OLED display.
    The graphics shown depict relay status.
    """

    # FB_WIDTH = 128  # oled display width
    # FB_HEIGHT = 64  # oled display height

    def __init__(self, width, height):
        # Set up OLED
        sda = Pin(4)
        scl = Pin(5)
        i2c = I2C(0, sda=sda, scl=scl, freq=400000)
        self._oled = SSD1306_I2C(width, height, i2c)

        self._oled_backoff = 0

    def message(self, message: str):
        """Display a textual message on the OLED.

        Args:
            message (str): The string to display
        """
        self._oled.fill(0)
        lines = message.split("\n")
        for line_num, line in enumerate(lines):
            self._oled.text(line, 0, 8 * line_num)
        self._oled.show()

        self._oled_backoff = (
            20  # This will prevent the next 20 graphical updates
        )

    def update(self, relay_state: int):
        """Update the graphic on the OLED according to relay status.

        Args:
            relay_state (int): An integer describing the relay status
        """

        if self._oled_backoff <= 0:
            self._oled.blit(framebuffers[relay_state], 0, 0)
            self._oled.show()
        else:
            self._oled_backoff -= 1


class Relay:
    PULSE_TIME = const(50)
    DISARM_TIME = const(500)

    def __init__(self):
        # GPIO inputs to read relay state
        # print("Creating Relay object")
        self._relay_state_1 = Pin(10, Pin.IN, Pin.PULL_UP)
        self._relay_state_2 = Pin(11, Pin.IN, Pin.PULL_UP)

        # GPIO outputs for relay control pulses
        self._relay1 = Pin(14, Pin.OUT)
        self._relay2 = Pin(15, Pin.OUT)

        # Clear relay control outputs
        self._relay1.value(0)
        self._relay2.value(0)

        # Respond to button pushes when armed
        self._armed = True

        # Timer to produced defined pulse width
        self._pulse_timer = Timer()

        # Timer to re-enable relay pulses
        self._disarm_timer = Timer()

    def set(self, mode: int):
        """Set the relay to the chosen mode by sending a pulse.

        :param mode: The desired relay mode.
        :type mode: int
        """

        if self._armed:
            if mode == RELAY_CROSS:
                self._relay1.value(0)
                self._relay2.value(1)
            else:
                self._relay1.value(1)
                self._relay2.value(0)
            self._pulse_timer.init(
                period=Relay.PULSE_TIME,
                mode=Timer.ONE_SHOT,
                callback=self.clear,
            )
            self._armed = False
            self._disarm_timer.init(
                period=Relay.DISARM_TIME,
                mode=Timer.ONE_SHOT,
                callback=self.re_arm,
            )

    def clear(self, timr: Timer):
        """Clear the relay drive output pins.

        :param timr: The timer that triggered this call
        :type timr: Timer
        """

        self._relay1.value(0)
        self._relay2.value(0)

    def re_arm(self, timr: Timer):
        """Re-arm after timer time-out to allow for contact bounce.

        :param timr: The timer that triggered this call.
        :type timr: Timer
        """

        self._armed = True

    def read(self) -> int:
        """Read the state of the relay.

        Returns:
            int: The relays status: 0 (invalid), 1 (through) or 2 (cross)
        """

        if self._relay_state_1.value():
            if not self._relay_state_2.value():
                return RELAY_STATE_THROUGH
            else:
                return RELAY_STATE_INVALID
        else:
            if self._relay_state_2.value():
                return RELAY_STATE_CROSS
            else:
                return RELAY_STATE_INVALID


class Button:
    def __init__(self, relay: Relay):
        self._relay = relay

        # GPIO inputs for user control
        self._button1 = Pin(20, Pin.IN, Pin.PULL_UP)
        self._button2 = Pin(21, Pin.IN, Pin.PULL_UP)

        # Button IRQs will instigate pulses
        self._button1.irq(
            trigger=Pin.IRQ_FALLING, handler=lambda a: self.pushed_id(1)
        )
        self._button2.irq(
            trigger=Pin.IRQ_FALLING, handler=lambda a: self.pushed_id(2)
        )

    # IRQ handler for button push
    def pushed_id(self, id: int):
        """Respond to button push.

        :param id: id of the button
        :type id: int
        """

        self._relay.set(RELAY_THRU if id == 1 else RELAY_CROSS)


class MqttRelay:
    def __init__(self, config, relay, oled):
        self._relay = relay
        self._oled = oled
        self._desired = None
        print("Creating client object")
        print(config)
        self._client = MQTTClient(config)
        print(self._client)
        mqtt_task = asyncio.create_task(self.mqtt_main())

    async def mqtt_handler(self):
        """Respond to incoming messages

        Args:
            client (MQTTClient): MQTT client object
        """

        async for topic, msg, retained in self._client.queue:
            print((topic, msg, retained))
            # message_oled(f'MQTT -> {msg}')
            self._desired = int(msg.decode())

            # print(f"{relay_state} {mqtt_desired}")

    async def mqtt_up(self):
        """Respond to connectivity being (re)established

        Args:
            client (MQTTClient): MQTT client object
        """

        while True:
            await self._client.up.wait()  # Wait on an Event
            self._client.up.clear()

            await self._client.subscribe(
                f"radio_relay/{unit_number}/desired", 1
            )  # renew subscriptions

    async def mqtt_main(self):
        """Main update loop to monitor the MQTT broker.

        Args:
            client (MQTTClient): MQTT client object
        """

        last_state = RELAY_STATE_INVALID
        try:
            print("Try to connect to MQTT")
            print(self._client)
            await self._client.connect()
            print("Connected")
            oled.message("Relay controller\nMQTT Online")

            await asyncio.sleep(1)  # noqa: E722
            for coroutine in (self.mqtt_up, self.mqtt_handler):
                asyncio.create_task(coroutine())

            while True:
                relay_state = self._relay.read()

                if self._desired:
                    if (
                        self._desired != relay_state
                    ):  # Trigger relay if necessary
                        self._relay.set(self._desired)
                    else:  # Otherwise clear the desired state
                        self._desired = None

                # Keep monitoring for relay state changes

                if last_state != relay_state:
                    await self._client.publish(
                        f"radio_relay/{unit_number}/state",
                        f"{relay_state}",
                        qos=1,
                    )
                last_state = relay_state
                await asyncio.sleep(0.1)
        except Exception as e:
            print("Exception")
            print(e)
            self._oled.message("Relay controller\nMQTT Error")


oled = Oled(128, 64)
relay = Relay()
_ = Button(relay)
mqtt_relay = MqttRelay(config, relay, oled)

# print('Starting')
# print(config)
# MQTTClient.DEBUG = True  # Optional: print diagnostic messages
oled.message("Relay controller\nMQTT Starting")


async def relay_update_loop():
    """Loop to monitor relay state and update the OLED accordingly"""
    while True:
        relay_state = relay.read()
        oled.update(relay_state)
        await asyncio.sleep_ms(100)


oled.message("Pico\nRelay controller")
time.sleep(2)

try:
    asyncio.run(relay_update_loop())
finally:
    asyncio.new_event_loop()
