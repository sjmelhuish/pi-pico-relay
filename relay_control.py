from machine import Pin, Timer, I2C
from ssd1306 import SSD1306_I2C
import framebuf
import time
import sys
import micropython

micropython.alloc_emergency_exception_buf(100)

RELAY_THRU = 1
RELAY_CROSS = 2
PULSE_TIME = 50
DISARM_TIME = 500

FB_WIDTH = 128  # oled display width
FB_HEIGHT = 64  # oled display height

armed = True  # Respond to button pushes when armed

# GPIO inputs to read relay state
relay_state_1 = Pin(10, Pin.IN, Pin.PULL_UP)
relay_state_2 = Pin(11, Pin.IN, Pin.PULL_UP)

# GPIO inputs for user control
button1 = Pin(20, Pin.IN, Pin.PULL_UP)
button2 = Pin(21, Pin.IN, Pin.PULL_UP)

# GPIO outputs for relay control pulses
relay1 = Pin(14, Pin.OUT)
relay2 = Pin(15, Pin.OUT)

# Clear relay control outputs
relay1.value(0)
relay2.value(0)

# Button IRQs will instigate pulses
button1.irq(trigger=Pin.IRQ_FALLING, handler=lambda a: pushed_id(1))
button2.irq(trigger=Pin.IRQ_FALLING, handler=lambda a: pushed_id(2))

# Set up OLED
sda = Pin(4)
scl = Pin(5)
i2c = I2C(0, sda=sda, scl=scl, freq=400000)
oled = SSD1306_I2C(128, 64, i2c)

cross = bytearray(
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x006\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00>\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00>\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00?\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x7f\x00\x00\x07\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00?\xc0\x00\x07\xf0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1f\xe0\x00\x0f\xf0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0f\xe0\x00?\xc0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x07\xf8\x00?\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\xf8\x00\x7f\x80\x008\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\xfe\x00\xfe\x00\x00\xee\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\xfe\x03\xfe\x00\x01\x83\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x7f\x83\xf8\x00\x03\x01\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00?\x87\xf8\x00\x02\x01\x86a\xc0\x1c\x01\xc0\x00\x00\x00\x00?\xef\xe0\x00\x06\x00\x07\xa7pw\x87x\x00\x00\x00\x00\x0f\xff\xe0\x00\x06\x00\x06\x0c\x18@\x84\x08\x00\x00\x00\x00\x07\xff\x80\x00\x06\x00\x06\x08\x08\xc0\xcc\x0c\x00\x00\x00\x00\x03\xff\x80\x00\x04\x00\x06\x18\x0c\xc0\x0c\x00\x00\x00\x00\x00\x03\xfe\x00\x00\x06\x00\x04\x18\x0cp\x07\x00\x00\x00\x00\x00\x01\xff\x00\x00\x06\x00\x06\x18\x0c\x1f\x01\xf0\x00\x00\x00\x00\x07\xff\x00\x00\x02\x00\x06\x08\x0c\x01\x80\x18\x00\x00\x00\x00\x07\xff\x80\x00\x02\x01\x86\x18\x08\x00\xc0\x0c\x00\x00\x00\x00\x1f\xff\xe0\x00\x03\x01\x86\x08\x08\xc0\xcc\x0c\x00\x00\x00\x00\x1f\xdf\xe0\x00\x01\x83\x04\x0c\x18@\x84\x08\x00\x00\x00\x00?\xc7\xf0\x00\x00\xee\x06\x07\xf0\x7f\x87\xb8\x00\x00\x00\x00\x7f\x07\xf8\x00\x008\x06\x01@\x14\x01\xe0\x00\x00\x00\x00\xff\x01\xfc\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x01\xfc\x01\xfe\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\xfc\x00\x7f\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x07\xf0\x00\x7f\x80\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0f\xf0\x00\x1f\xc0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x1f\xc0\x00\x1f\xe0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00?\xc0\x00\x07\xf0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x7f\x00\x00\x07\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x7f\x00\x00\x03\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00>\x00\x00\x01\xf0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00~\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00*\x00\x00\x01\xf8\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
)
fbcross = framebuf.FrameBuffer(cross, 128, 64, framebuf.MONO_HLSB)
thru = bytearray(
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x10\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00P\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x10\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00P\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x10\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00P\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x0c\x00\x00\x00\x00\x00\x00\x00\x00\x10\x00\x00\x00@\x07\xff\xcc\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x02\xba\x8c\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x08\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00P\x000\x0c\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x0c\xe0f``\x00\x00\x00\x00\x10\x00\x00\x00@\x00 \r\xb8z``\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x0e\x08```\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x0c\x0c`@@\x00\x00\x00\x00\x14\x00\x00\x00P\x000\x08\x0c```\x00\x00\x00\x00\x18\x00\x00\x00`\x00 \x0c\x0c@``\x00\x00\x00\x00\x10\x00\x00\x00@\x000\x0c\x0c```\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x0c\x0c`@@\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x08\x08```\x00\x00\x00\x00\x14\x00\x00\x00P\x00 \x0c\x0c@``\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x0c\x0c` \xe0\x00\x00\x00\x00\x10\x00\x00\x00@\x000\x0c\x0c`?`\x00\x00\x00\x00\x18\x00\x00\x00`\x000\x08\x0c`\x0c`\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00P\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x10\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00\xa0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00P\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x10\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00P\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x10\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00`\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x18\x00\x00\x00@\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
)
fbthru = framebuf.FrameBuffer(thru, 128, 64, framebuf.MONO_HLSB)
invalid = bytearray(
    b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x14\x00\x00\x00\x01@\x00\x00\x00)\x00\x00\x00\x00\x00\x07\xff\xf0\x00\x00\xbf\xfe\x00\x00\x0f\xff\xe0\x00\x00\x00\x00\x7f\xff\xfc\x00\x07\xff\xff\xc0\x00\x7f\xff\xfc\x00\x00\x00\x01\xff\xff\xff\x00\x1f\xff\xff\xf0\x03\xff\xff\xfe\x00\x00\x00\x07\xff\xff\xff\x80\xff\xff\xff\xf8\x0f\xff\xff\xff\x80\x00\x00\x0f\xff\xff\xff\xc0\xff\xff\xff\xfc\x1f\xff\xff\xff\xc0\x00\x00\x0f\xff\xff\xff\xe0\xff\xff\xff\xfe\x0f\xff\xff\xff\xc0\x00\x00\x07\xff\xff\xff\xf0\xff\xff\xff\xfe\x0f\xff\xff\xff\xe0\x00\x00\x07\xff\xff\xff\xf0\x7f\xff\xff\xff\x07\xff\xff\xff\xf0\x00\x00\x03\xff\xd7\xff\xf8?\xfd\x7f\xff\x07\xff\xaf\xff\xf0\x00\x00\x03\xfc\x00\x7f\xf8?\x80\x0f\xff\x83\xf8\x00\xff\xf0\x00\x00\x01\xe0\x00?\xf8>\x00\x03\xff\x83\xc0\x00\x7f\xf8\x00\x00\x01\x80\x00?\xf8\x10\x00\x03\xff\x81\x00\x00?\xf8\x00\x00\x00\x00\x00\x1f\xf8\x00\x00\x03\xff\x80\x00\x00?\xf8\x00\x00\x00\x00\x00\x1f\xf8\x00\x00\x01\xff\x80\x00\x00?\xf8\x00\x00\x00\x00\x00?\xf8\x00\x00\x03\xff\x80\x00\x00?\xf0\x00\x00\x00\x00\x00\x1f\xf8\x00\x00\x03\xff\x80\x00\x00?\xf8\x00\x00\x00\x00\x00?\xf8\x00\x00\x03\xff\x80\x00\x00\x7f\xf0\x00\x00\x00\x00\x00\x7f\xf8\x00\x00\x07\xff\x00\x00\x00\x7f\xf0\x00\x00\x00\x00\x00\xff\xf0\x00\x00\x0f\xff\x00\x00\x00\xff\xf0\x00\x00\x00\x00\x01\xff\xf0\x00\x00\x1f\xfe\x00\x00\x01\xff\xe0\x00\x00\x00\x00\x03\xff\xe0\x00\x00?\xfe\x00\x00\x07\xff\xc0\x00\x00\x00\x00\x07\xff\xc0\x00\x00\xff\xfc\x00\x00\x0f\xff\xc0\x00\x00\x00\x00\x1f\xff\x80\x00\x01\xff\xf8\x00\x00\x1f\xff\x00\x00\x00\x00\x00?\xff\x00\x00\x03\xff\xf0\x00\x00?\xfe\x00\x00\x00\x00\x00\x7f\xfe\x00\x00\x07\xff\xc0\x00\x00\xff\xfc\x00\x00\x00\x00\x00\xff\xf8\x00\x00\x0f\xff\x80\x00\x00\xff\xf8\x00\x00\x00\x00\x00\xff\xf0\x00\x00\x1f\xff\x00\x00\x01\xff\xf0\x00\x00\x00\x00\x01\xff\xe0\x00\x00?\xfe\x00\x00\x03\xff\xc0\x00\x00\x00\x00\x03\xff\xc0\x00\x00?\xfc\x00\x00\x03\xff\x80\x00\x00\x00\x00\x03\xff\x80\x00\x00?\xf0\x00\x00\x07\xff\x00\x00\x00\x00\x00\x03\xff\x00\x00\x00\x7f\xf0\x00\x00\x07\xfe\x00\x00\x00\x00\x00\x07\xfe\x00\x00\x00\x7f\xe0\x00\x00\x07\xfe\x00\x00\x00\x00\x00\x03\xfe\x00\x00\x00\x7f\xe0\x00\x00\x07\xfe\x00\x00\x00\x00\x00\x07\xfe\x00\x00\x00\x7f\xe0\x00\x00\x07\xfc\x00\x00\x00\x00\x00\x03\xfe\x00\x00\x00\x7f\xe0\x00\x00\x07\xfe\x00\x00\x00\x00\x00\x07\xfe\x00\x00\x00\x7f\xe0\x00\x00\x07\xfc\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00h\x00\x00\x00\x06\x80\x00\x00\x00\xa8\x00\x00\x00\x00\x00\x03\xfe\x00\x00\x00?\xc0\x00\x00\x03\xfc\x00\x00\x00\x00\x00\x07\xff\x00\x00\x00\x7f\xf0\x00\x00\x07\xff\x00\x00\x00\x00\x00\x07\xff\x00\x00\x00\x7f\xf0\x00\x00\x0f\xff\x00\x00\x00\x00\x00\x0f\xff\x80\x00\x00\xff\xf8\x00\x00\x0f\xff\x00\x00\x00\x00\x00\x0f\xff\x80\x00\x00\xff\xf8\x00\x00\x0f\xff\x80\x00\x00\x00\x00\x0f\xff\x80\x00\x00\xff\xf8\x00\x00\x0f\xff\x00\x00\x00\x00\x00\x0f\xff\x80\x00\x00\xff\xf8\x00\x00\x0f\xff\x80\x00\x00\x00\x00\x07\xff\x80\x00\x00\xff\xf0\x00\x00\x0f\xff\x00\x00\x00\x00\x00\x07\xff\x00\x00\x00\x7f\xf0\x00\x00\x0f\xff\x00\x00\x00\x00\x00\x07\xff\x00\x00\x00\x7f\xf0\x00\x00\x07\xfe\x00\x00\x00\x00\x00\x03\xfe\x00\x00\x00?\xe0\x00\x00\x03\xfe\x00\x00\x00\x00\x00\x00p\x00\x00\x00\x0f\x00\x00\x00\x00\xf0\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
)
fbinvalid = framebuf.FrameBuffer(invalid, 128, 64, framebuf.MONO_HLSB)

# Timer to produced defined pulse width
pulse_timer = Timer()

# Timer to defeat contact bounce
disarm_timer = Timer()

# IRQ handler for button push
def pushed_id(id: int):
    """Respond to button push.

    :param id: id of the button
    :type id: int
    """
    set_relay(RELAY_THRU if id == 1 else RELAY_CROSS)


def set_relay(mode: int):
    """Set the relay to the chosen mode by sending a pulse.

    :param mode: The desired relay mode.
    :type mode: int
    """
    global armed

    if armed:
        if mode == RELAY_CROSS:
            relay1.value(0)
            relay2.value(1)
        else:
            relay1.value(1)
            relay2.value(0)
        pulse_timer.init(period=PULSE_TIME, mode=Timer.ONE_SHOT, callback=clear_relay)
        armed = False
        disarm_timer.init(period=DISARM_TIME, mode=Timer.ONE_SHOT, callback=re_arm)


def clear_relay(timr: Timer):
    """Clear the relay drive output pins.

    :param timr: The timer that triggered this call
    :type timr: Timer
    """

    relay1.value(0)
    relay2.value(0)


def re_arm(timr: Timer):
    """Re-arm after timer time-out to allow for contact bounce.

    :param timr: The timer that triggered this call.
    :type timr: Timer
    """
    global armed
    armed = True


def message_oled(message:str):
    oled.fill(0)
    oled.text(message, 0, 0)
    oled.show()

def update_oled():
    """Update the graphic on the OLED according to relay status.
    """

    if relay_state_1.value():
        if not relay_state_2.value():
            oled.blit(fbthru, 0, 0)
        else:
            oled.blit(fbinvalid, 0, 0)
    else:
        if relay_state_2.value():
            oled.blit(fbcross, 0, 0)
        else:
            oled.blit(fbinvalid, 0, 0)
    oled.show()

message_oled("Relay controller")
time.sleep(2)

while True:
    update_oled()
    time.sleep_ms(100)
