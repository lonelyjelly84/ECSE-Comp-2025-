"""I2C LCD Display Module, could communicat with an Arduino which can then communicate
over I2C to a LCD display.

However, we could try just to use the I2C Arduino Compatible Module."""

from datetime import time
import smbus2


def lcd_init():
    """Initialize the LCD display"""
    # I2C channel


channel = 1

# Hold all the address pins high
module_address = 0x27

# Register Addresses


# Initialise I2C (SMBus)
bus = smbus2.SMBus(channel)
# Initialise display
bus.write_byte_data(module_address, 0x00, 0x33)  # 110011 Initialise
bus.write_byte_data(module_address, 0x00, 0x32)  # 110010 Initialise
bus.write_byte_data(module_address, 0x00, 0x06)  # 000110 Cursor move direction
bus.write_byte_data(
    module_address, 0x00, 0x0C
)  # 001100 Display On,Cursor Off, Blink Off
bus.write_byte_data(
    module_address, 0x00, 0x28
)  # 101000 Data length, number of lines, font size
bus.write_byte_data(module_address, 0x00, 0x01)  # 000001 Clear display
time.sleep(0.0005)  # Delay to allow commands to process


def lcd_display_string(string, line):
    """Send string to display"""
    if line == 1:
        pos = 0x80
    elif line == 2:
        pos = 0xC0
    elif line == 3:
        pos = 0x94
    elif line == 4:
        pos = 0xD4

    bus.write_byte_data(module_address, 0x00, pos)

    for char in string:
        bus.write_byte_data(module_address, 0x40, ord(char))
