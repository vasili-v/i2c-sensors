import smbus2

RPI_DEFAULT_BUS=1

def i2c():
    return smbus2.SMBus(RPI_DEFAULT_BUS)
