""" The submodule provides I2C related helpers """

import smbus2

RPI_DEFAULT_BUS=1

def i2c():
    """ Returns a new SMbus instance for default Raspberry PI I2C bus """
    return smbus2.SMBus(RPI_DEFAULT_BUS)
