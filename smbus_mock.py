
# Mock class for SMbus
class SMBus:
    def __init__(self, bus=None):
        self._bus = bus

    def read_byte_data(self, i2c_addr, register, force=False):
        # Implement your mock read_byte_data logic here
        return 0xFF  # For example, always return 0xFF for testing

    def write_byte_data(self, i2c_addr, register, value, force=False):
        # Implement your mock write_byte_data logic here
        pass  # For example, do nothing for testing

    def read_i2c_block_data(self, i2c_addr, register, length, force=False):
        # Implement your mock read_i2c_block_data logic here
        return [0x00] * length  # For example, return a list of 0x00 values for testing

    def write_i2c_block_data(self, i2c_addr, register, values, force=False):
        # Implement your mock write_i2c_block_data logic here
        pass  # For example, do nothing for testing
    
    def read_word_data(self, i2c_addr, register, force=False):
        # Return stored written value or a default (using little-endian format)
        return 0xFFFF

    def write_word_data(self, i2c_addr, register, value, force=False):
        # Implement your mock write_word_datalogic here
        pass