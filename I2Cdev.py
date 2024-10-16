import time
import smbus2

class I2Cdev:
    def __init__(self, bus=None, i2c_addr=0, debug=False):
        self.smbus = smbus2.SMBus(bus=bus)
        self.i2c_addr = i2c_addr
        self.debug = debug

    def _debug_print(self, message):
        if self.debug:
            print(message)

    def write_bit(self, register, bit_position, value):
        """Write a single bit to a specified register."""
        current_value = self.smbus.read_byte_data(self.i2c_addr, register)
        self._debug_print(f"Current Value (before writing bit): {current_value:#04x}")

        mask = 1 << bit_position
        if value:
            new_value = current_value | mask
        else:
            new_value = current_value & ~mask

        self.smbus.write_byte_data(self.i2c_addr, register, new_value)
        self._debug_print(f"Wrote Bit: {value}, Register: {register:#04x}, New Value: {new_value:#04x}")

    def read_bit(self, register, bit_position):
        """Read a single bit from a specified register."""
        current_value = self.smbus.read_byte_data(self.i2c_addr, register)
        self._debug_print(f"Read Value: {current_value:#04x} from Register: {register:#04x}")

        mask = 1 << bit_position
        bit_value = (current_value & mask) >> bit_position
        self._debug_print(f"Bit Value at Position {bit_position}: {bit_value}")
        return bit_value

    def write_bits(self, register, bit_start, length, value):
        """Write bits to a specified register."""
        current_value = self.smbus.read_byte_data(self.i2c_addr, register)
        self._debug_print(f"Current Value (before writing bits): {current_value:#04x}")

        mask = ((1 << length) - 1) << (bit_start - length + 1)

        # Shift data into the correct position
        value <<= (bit_start - length + 1)

        # Zero out all non-important bits in data
        value &= mask

        new_value = value

        # Zero out all important bits in existing byte
        new_value &= ~mask

        # Combine data with existing byte
        new_value |= value

        self.smbus.write_byte_data(self.i2c_addr, register, new_value)
        self._debug_print(f"Wrote Bits: {value}, Register: {register:#04x}, New Value: {new_value:#04x}")

    def read_bits(self, register, bit_start, length):
        """Read bits from a specified register."""
        current_value = self.smbus.read_byte_data(self.i2c_addr, register)
        self._debug_print(f"Read Value: {current_value:#04x} from Register: {register:#04x}")

        mask = ((1 << length) - 1) << (bit_start - length + 1)
    
        # Apply the mask and shift the bits to the right
        extracted_bits = (current_value & mask) >> (bit_start - length + 1)

        self._debug_print(f"Bits Value from Position {bit_start} Length {length}: {extracted_bits}")
        return extracted_bits

    def write_bytes(self, register, values):
        self.smbus.write_i2c_block_data(self.i2c_addr, register, values)
        self._debug_print(f"Wrote {len(values)} Bytes: {values} to Register: {register:#04x}")

    def write_bytes_s(self, register, values):
        for i, byte_value in enumerate(values):
            self.smbus.write_byte_data(self.i2c_addr, register + i, byte_value)
        self._debug_print(f"Wrote {len(values)} Bytes: {values} to Register: {register:#04x}")

    def read_bytes(self, register, length):
        values = self.smbus.read_i2c_block_data(self.i2c_addr, register, length)
        self._debug_print(f"Read {length} Bytes from Register: {register:#04x}, Values: {bytearray(values)}")
        return bytearray(values)
    
    def read_bytes_s(self, register, length):
        values = []
        for i in range(length):
            byte_value = self.smbus.read_byte_data(self.i2c_addr, register + i)
            values.append(byte_value)
        self._debug_print(f"Read {len(values)} Bytes from Register: {register:#04x}, Values: {bytearray(values)}")
        return bytearray(values)

    def write_byte(self, register, value):
        self.smbus.write_byte_data(self.i2c_addr, register, value)
        self._debug_print(f"Wrote Byte: {value} to Register: {register:#04x}")

    def read_byte(self, register):
        value = self.smbus.read_byte_data(self.i2c_addr, register)
        self._debug_print(f"Read Byte from Register: {register:#04x}, Value: {value}")
        return value

    def write_word(self, register, value):
        if not (0 <= value <= 0xFFFF):
            raise ValueError(f"Value out of range for 16-bit integer: {value}")

        # Split the 16-bit value into two bytes
        msb = (value >> 8) & 0xFF  # Most significant byte
        lsb = value & 0xFF         # Least significant byte

        self.smbus.write_i2c_block_data(self.i2c_addr, register, bytes([msb, lsb]))
    
        self._debug_print(f"Wrote Word: {value} to Register: {register:#04x}")

    def read_word(self, register):
        word_bytes = self.smbus.read_i2c_block_data(self.i2c_addr, register, 2)
        if len(word_bytes) < 2:
            print("Failed to read words.")
            return []

            # Combine the two bytes into one word (16 bits)
        word = (word_bytes[0] << 8) | word_bytes[1]
        
        self._debug_print(f"Read Word from Register: {register:#04x}, Value: {word}")
        return word

    def read_word_s(self, register):
        # Read two bytes from the specified register
        value = self.smbus.read_word_data(self.i2c_addr, register)

        # Debug print for the raw value read
        self._debug_print(f"Read Word from Register: {register:#04x}, Raw Value: {value}")

        # Convert the value to a signed integer
        # The value returned by read_word_data is unsigned, so we convert it.
        signed_value = value if value < 0x8000 else value - 0x10000

        # Debug print for the signed value
        self._debug_print(f"Converted to Signed Value: {signed_value}")

        return signed_value
    
    def write_words(self, register, values):
        i = 0
        for value in values:
            # Ensure the value is between 0 and 0xFFFF
            if not (0 <= value <= 0xFFFF):
                raise ValueError(f"Value out of range for 16-bit integer: {value}")

            # Split the 16-bit value into two bytes
            msb = (value >> 8) & 0xFF  # Most significant byte
            lsb = value & 0xFF         # Least significant byte

            self.smbus.write_i2c_block_data(self.i2c_addr, register + i*2, bytes([msb, lsb]))
            i+= 1
            

    def read_words(self, register, count):
        data = []
        for i in range(count):

            # Read two bytes (one word) from the device
            word_bytes = self.smbus.read_i2c_block_data(self.i2c_addr, register + i*2, 2)
            if len(word_bytes) < 2:
                print("Failed to read words.")
                return []

            # Combine the two bytes into one word (16 bits)
            word = (word_bytes[0] << 8) | word_bytes[1]
            data.append(word)

            # Debug output for read words; Uncomment if you'd like to see debug info
            # print(f"{word:X}", end=" ")
        
        return data

