# SPDX-FileCopyrightText: 2026 Matt Trentini
# SPDX-License-Identifier: MIT

"""
Minimal register helper classes for MicroPython compatibility.
Replaces adafruit_register functionality for PCF8523 RTC.
"""

from time import struct_time


class RWBit:
    """Single bit register accessor."""

    def __init__(self, register_address, bit):
        self.register_address = register_address
        self.bit = bit
        self.bit_mask = 1 << bit

    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]))
            result = bytearray(1)
            i2c.readinto(result)
            return bool(result[0] & self.bit_mask)

    def __set__(self, obj, value):
        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]))
            result = bytearray(1)
            i2c.readinto(result)
            if value:
                result[0] |= self.bit_mask
            else:
                result[0] &= ~self.bit_mask
            i2c.write(bytes([self.register_address, result[0]]))


class ROBit:
    """Read-only single bit register accessor."""

    def __init__(self, register_address, bit):
        self.register_address = register_address
        self.bit = bit
        self.bit_mask = 1 << bit

    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]))
            result = bytearray(1)
            i2c.readinto(result)
            return bool(result[0] & self.bit_mask)


class RWBits:
    """Multiple bits register accessor."""

    def __init__(self, num_bits, register_address, lowest_bit, signed=False):
        self.register_address = register_address
        self.lowest_bit = lowest_bit
        self.bit_mask = ((1 << num_bits) - 1) << lowest_bit
        self.signed = signed
        self.num_bits = num_bits

    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]))
            result = bytearray(1)
            i2c.readinto(result)
            value = (result[0] & self.bit_mask) >> self.lowest_bit
            if self.signed and value & (1 << (self.num_bits - 1)):
                value -= 1 << self.num_bits
            return value

    def __set__(self, obj, value):
        if self.signed:
            if value < 0:
                value += 1 << self.num_bits
        value = (value << self.lowest_bit) & self.bit_mask
        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]))
            result = bytearray(1)
            i2c.readinto(result)
            result[0] = (result[0] & ~self.bit_mask) | value
            i2c.write(bytes([self.register_address, result[0]]))


class BCDDateTimeRegister:
    """BCD datetime register accessor for RTC chips."""

    def __init__(self, register_address, weekday_shared, weekday_start):
        self.register_address = register_address
        self.weekday_shared = weekday_shared
        self.weekday_start = weekday_start

    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        # Read 7 bytes starting from register_address
        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]))
            buffer = bytearray(7)
            i2c.readinto(buffer)

        # Decode BCD values
        # buffer[0] = seconds (ignore bit 7 - oscillator stop flag)
        # buffer[1] = minutes
        # buffer[2] = hours
        # buffer[3] = day of month
        # buffer[4] = weekday
        # buffer[5] = month (ignore bit 7 - century)
        # buffer[6] = year

        return struct_time((
            self._bcd2bin(buffer[6]) + 2000,  # year
            self._bcd2bin(buffer[5] & 0x1F),  # month
            self._bcd2bin(buffer[3]),         # day
            self._bcd2bin(buffer[2]),         # hour
            self._bcd2bin(buffer[1]),         # minute
            self._bcd2bin(buffer[0] & 0x7F), # second
            self._bcd2bin(buffer[4]),         # weekday
            -1,  # yearday (not supported)
            -1   # isdst (not supported)
        ))

    def __set__(self, obj, value):
        # Encode struct_time to BCD
        buffer = bytearray([
            self._bin2bcd(value.tm_sec),
            self._bin2bcd(value.tm_min),
            self._bin2bcd(value.tm_hour),
            self._bin2bcd(value.tm_mday),
            self._bin2bcd(value.tm_wday),
            self._bin2bcd(value.tm_mon),
            self._bin2bcd(value.tm_year - 2000)
        ])

        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]) + buffer)

    @staticmethod
    def _bcd2bin(value):
        """Convert BCD to binary."""
        return (value >> 4) * 10 + (value & 0x0F)

    @staticmethod
    def _bin2bcd(value):
        """Convert binary to BCD."""
        return ((value // 10) << 4) | (value % 10)


class BCDAlarmTimeRegister:
    """BCD alarm time register accessor."""

    def __init__(self, register_address, has_seconds, weekday_shared, weekday_start):
        self.register_address = register_address
        self.has_seconds = has_seconds
        self.weekday_shared = weekday_shared
        self.weekday_start = weekday_start

    def __get__(self, obj, objtype=None):
        if obj is None:
            return self
        # Read 4 bytes (minute, hour, day, weekday)
        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]))
            buffer = bytearray(4)
            i2c.readinto(buffer)

        # Check if alarm is disabled (bit 7 set means disabled)
        minute = self._bcd2bin(buffer[0] & 0x7F) if not (buffer[0] & 0x80) else None
        hour = self._bcd2bin(buffer[1] & 0x3F) if not (buffer[1] & 0x80) else None
        day = self._bcd2bin(buffer[2] & 0x3F) if not (buffer[2] & 0x80) else None
        weekday = self._bcd2bin(buffer[3] & 0x07) if not (buffer[3] & 0x80) else None

        return (minute, hour, day, weekday)

    def __set__(self, obj, value):
        # value should be a tuple: (minute, hour, day, weekday)
        # None values mean alarm disabled for that field
        minute, hour, day, weekday = value

        buffer = bytearray([
            self._bin2bcd(minute) if minute is not None else 0x80,
            self._bin2bcd(hour) if hour is not None else 0x80,
            self._bin2bcd(day) if day is not None else 0x80,
            self._bin2bcd(weekday) if weekday is not None else 0x80
        ])

        with obj.i2c_device as i2c:
            i2c.write(bytes([self.register_address]) + buffer)

    @staticmethod
    def _bcd2bin(value):
        """Convert BCD to binary."""
        return (value >> 4) * 10 + (value & 0x0F)

    @staticmethod
    def _bin2bcd(value):
        """Convert binary to BCD."""
        return ((value // 10) << 4) | (value % 10)


class I2CDevice:
    """Simple I2C device context manager for MicroPython."""

    def __init__(self, i2c, device_address):
        self.i2c = i2c
        self.device_address = device_address

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        return False

    def write(self, buf):
        """Write buffer to I2C device."""
        self.i2c.writeto(self.device_address, buf)

    def readinto(self, buf):
        """Read from I2C device into buffer."""
        result = self.i2c.readfrom(self.device_address, len(buf))
        buf[:] = result
