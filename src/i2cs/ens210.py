""" The submodule provides a class for the I2C interface of ENS210 relative humidity and
    temperature sensor """

import time

from .error import Error
from .device import Block, Device

_ADDRESS=0x43

_NANOSECONDS=1e9
_T_BOOTING=0.0012
_T_CONV_TH_SINGLE_SHORT=0.130

_REG_PART_ID=Block(0x00, 2)
_REG_DIE_REV=Block(0x02, 2)
_REG_UID=Block(0x04, 8)

_REG_SYS_CTRL=0x10

_REG_SYS_CTRL_RESET=0x80
_REG_SYS_CTRL_LOW_POWER=0x01

_REG_SYS_STAT=0x11

_REG_SYS_STAT_SYS_ACTIVE=0x1

_REG_SENS_START=0x22

_REG_SENS_START_T_START=0x01
_REG_SENS_START_H_START=0x02

_REG_VAL=Block(0x30, 6)

def _b_to_u(block):
    shift = 0
    u = 0
    for b in block:
        u |= b << shift
        shift += 8

    return u

def _split(data):
    return _b_to_u(data[:2]), bool(data[2] & 0x1), (data[2] >> 1) & 0x7f

_DATA7_WIDTH=17
_DATA7_MASK=(1<<_DATA7_WIDTH)-1
_DATA7_MSB=1<<_DATA7_WIDTH-1

_CRC7_WIDTH=7
_CRC7_POLY=0x89
_CRC7_IVEC=0x7f

_DATA7_MASK_HIGH=_DATA7_MASK<<_CRC7_WIDTH
_CRC7_IPOL=_CRC7_POLY<<_DATA7_WIDTH-1
_CRC7_IBIT=_DATA7_MSB<<_CRC7_WIDTH

def _crc7(data):
    x = ((_b_to_u(data) & _DATA7_MASK) << _CRC7_WIDTH) | _CRC7_IVEC
    p = _CRC7_IPOL
    b = _CRC7_IBIT

    while b & _DATA7_MASK_HIGH:
        if b & x:
            x ^= p
        p >>= 1
        b >>= 1

    return x

def _repr_crc7(crc, crc_cmp):
    if crc == crc_cmp:
        return f'CRC: 0x{crc:02x} - ok'

    return f'CRC: 0x{crc:02x}, 0x{crc_cmp:02x} - fail'

class ENS210(Device):
    """ The class provides access to the basic functionality of ENS210 through I2C bus """
    def __init__(self, bus):
        super().__init__(bus, _ADDRESS)

    def __wait_booting(self, timeout=None):
        start = 0
        if timeout is not None:
            timeout *= _NANOSECONDS
            start = time.monotonic_ns()

        while timeout is None or time.monotonic_ns() - start < timeout:
            time.sleep(_T_BOOTING)
            if self.read(_REG_SYS_STAT) & _REG_SYS_STAT_SYS_ACTIVE:
                return True

        return False

    def read_ids(self):
        """ Read device ID registers """
        self.write(_REG_SYS_CTRL, 0)
        try:
            if not self.__wait_booting():
                raise Error("Timeout while reading IDs")

            return (
                    _b_to_u(self.read(_REG_PART_ID)),
                    _b_to_u(self.read(_REG_DIE_REV)),
                    _b_to_u(self.read(_REG_UID)),
            )
        finally:
            self.write(_REG_SYS_CTRL, _REG_SYS_CTRL_LOW_POWER)

    def measure(self):
        """ Measure temperature and relative humidity in one shot """
        start = time.monotonic_ns()
        self.write(_REG_SENS_START, _REG_SENS_START_H_START | _REG_SENS_START_T_START)
        time.sleep(_T_CONV_TH_SINGLE_SHORT)

        data = self.read(_REG_VAL)

        t_raw, t_valid, t_crc = _split(data[:3])
        t_crc_str = _repr_crc7(t_crc, _crc7(data[:3]))

        h_raw, h_valid, h_crc = _split(data[3:])
        h_crc_str = _repr_crc7(h_crc, _crc7(data[3:]))

        print(f'RH ({(time.monotonic_ns() - start)/1e6:.2f} ms)\n' \
              f'\tT: 0x{t_raw:02x} (Valid: {t_valid}, {t_crc_str})\n' \
              f'\tH: 0x{h_raw:02x} (Valid: {h_valid}, {h_crc_str})')

        return t_raw/64 - 273.15, h_raw/512
