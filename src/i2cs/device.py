""" The submodule provides a base class for I2C devices """

import dataclasses

@dataclasses.dataclass(frozen=True)
class Block:
    """ Represents a register block """
    offset: int
    length: int

    def zero(self):
        """ Returns a zeroed block """
        return self.length * (0,)

class Device:
    """ Base class for I2C devices """

    def __init__(self, bus, address):
        self.__bus = bus
        self.__address = address

    def __enter__(self):
        """ Enter the device context """
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """ Exit the device context """

    def read(self, entity):
        """ Read data from the device """
        if isinstance(entity, Block):
            return self.__bus.read_i2c_block_data(self.__address, entity.offset, entity.length)

        return self.__bus.read_byte(self.__address, entity)

    def write(self, offset, data):
        """ Write data to the device """
        self.__bus.write_byte_data(self.__address, offset, data)
