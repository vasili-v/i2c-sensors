""" The submodule provides a class for the I2C interface of SPL07-003 digital pressure sensor """

import dataclasses
import enum
import time
import errno

from .error import Error
from .device import Block, Device

DEFAULT_ADDRESS=0x77
ALTERNATE_ADDRESS=0x76

_REG_BLOCK_PRS=Block(0x00, 3)
_REG_BLOCK_TMP=Block(0x03, 3)

_REG_PRS_CFG=0x06
_REG_TMP_CFG=0x07
_REG_MEAS_CFG=0x08
_REG_CFG_REG=0x09
_REG_INT_STS=0x0a
_REG_FIFO_STS=0x0b
_REG_RESET=0x0c

_REG_BLOCK_COEF=Block(0x10, 21)

_REG_PT_CFG_PRC_MASK = 0x0f

_MEAS_CTRL_VALUES = {
    0b000: 'Idle',
    0b001: 'P',
    0b010: 'T',
    0b011: 'N/A',
    0b100: 'N/A',
    0b101: 'P cont.',
    0b110: 'T cont.',
    0b111: 'P+T cont.'
}

_REG_MEAS_CFG_MEAS_CTRL_NA = 0x3
_REG_MEAS_CFG_MEAS_CTRL_MASK = 0x07
_REG_MEAS_CFG_PRS_RDY = 0x10
_REG_MEAS_CFG_TMP_RDY = 0x20
_REG_MEAS_CFG_SENSOR_RDY = 0x40
_REG_MEAS_CFG_COEF_RDY = 0x80

_REG_MEAS_CFG_DEV_RDY = _REG_MEAS_CFG_SENSOR_RDY|_REG_MEAS_CFG_COEF_RDY
_REG_MEAS_CFG_DEV_RDY_MASK = _REG_MEAS_CFG_DEV_RDY|_REG_MEAS_CFG_MEAS_CTRL_MASK

_REG_MEAS_CFG_PRS_DONE = _REG_MEAS_CFG_PRS_RDY|_REG_MEAS_CFG_SENSOR_RDY
_REG_MEAS_CFG_PRS_DONE_MASK = _REG_MEAS_CFG_PRS_DONE|_REG_MEAS_CFG_MEAS_CTRL_MASK

_REG_MEAS_CFG_TMP_DONE = _REG_MEAS_CFG_TMP_RDY|_REG_MEAS_CFG_SENSOR_RDY
_REG_MEAS_CFG_TMP_DONE_MASK = _REG_MEAS_CFG_TMP_DONE|_REG_MEAS_CFG_MEAS_CTRL_MASK

_REG_CFG_REG_FIFO_EN = 0x02
_REG_CFG_REG_P_SHIFT = 0x04
_REG_CFG_REG_T_SHIFT = 0x08
_REG_CFG_REG_INT_PRS = 0x10
_REG_CFG_REG_INT_TMP = 0x20
_REG_CFG_REG_INT_FIFO = 0x40
_REG_CFG_REG_INT_HL = 0x80

class Oversampling(enum.IntEnum):
    """ The enumeration defines the oversampling rates for pressure and temperature of SPL07-003
        sensor"""
    X1 = 0 # single (low precision for pressure and default for temperature)
    # Note: Following are optional, and may not be relevant for temperature measurements
    X2 = 1 # 2 times (low power)
    X4 = 2 # 4 times
    X8 = 3 # 8 times
    X16 = 4 # 16 times (standard)
    X32 = 5 # 32 times
    X64 = 6 # 64 times (high precision)
    X128 = 7 # 128 times

_MEAS_TIME = {
    Oversampling.X1: 0.0036,
    Oversampling.X2: 0.0052,
    Oversampling.X4: 0.0084,
    Oversampling.X8: 0.0148,
    Oversampling.X16: 0.0276,
    Oversampling.X32: 0.0532,
    Oversampling.X64: 0.1044,
    Oversampling.X128: 0.2068,
}

_SCALE_FACTOR = {
    Oversampling.X1: 0x080000,
    Oversampling.X2: 0x180000,
    Oversampling.X4: 0x380000,
    Oversampling.X8: 0x780000,
    Oversampling.X16: 0x03e000,
    Oversampling.X32: 0x07e000,
    Oversampling.X64: 0x0fe000,
    Oversampling.X128: 0x1fe000,
}

def _u_to_s(u, bits):
    """ Convert 2's complement integer to signed number """
    mask = 1 << (bits-1)
    return -mask + (u & (mask-1)) if u & mask else u

@dataclasses.dataclass(frozen=True)
class Calibration: # pylint: disable=too-many-instance-attributes
    """ The class stores the calibration coefficients of SPL07-003 sensor """

    block: list[int] = _REG_BLOCK_COEF.zero()
    c0: int = 0
    c1: int = 0
    c00: int = 0
    c10: int = 0
    c01: int = 0
    c11: int = 0
    c20: int = 0
    c21: int = 0
    c30: int = 0
    c31: int = 0
    c40: int = 0

    def __init__(self, block=None):
        if block is not None:
            block = tuple(block[:_REG_BLOCK_COEF.length])
            if len(block) < _REG_BLOCK_COEF.length:
                block += (_REG_BLOCK_COEF.length - len(self.block)) * (0,)
            object.__setattr__(self, 'block', block)

        object.__setattr__(self, 'c0',
            _u_to_s((self.block[0] << 4) | (self.block[1] >> 4), 12))
        object.__setattr__(self, 'c1',
            _u_to_s(((self.block[1] & 0xf) << 8) | self.block[2], 12))
        object.__setattr__(self, 'c00',
            _u_to_s((self.block[3] << 12) | (self.block[4] << 4) | (self.block[5] >> 4), 20))
        object.__setattr__(self, 'c10',
            _u_to_s(((self.block[5] & 0xf) << 16) | (self.block[6] << 8) | self.block[7], 20))
        object.__setattr__(self, 'c01',
            _u_to_s((self.block[8] << 8) | self.block[9], 16))
        object.__setattr__(self, 'c11',
            _u_to_s((self.block[10] << 8) | self.block[11], 16))
        object.__setattr__(self, 'c20',
            _u_to_s((self.block[12] << 8) | self.block[13], 16))
        object.__setattr__(self, 'c21',
            _u_to_s((self.block[14] << 8) | self.block[15], 16))
        object.__setattr__(self, 'c30',
            _u_to_s((self.block[16] << 8) | self.block[17], 16))
        object.__setattr__(self, 'c31',
            _u_to_s((self.block[18] << 4) | (self.block[19] >> 4), 12))
        object.__setattr__(self, 'c40',
            _u_to_s(((self.block[19] & 0xf) << 8) | self.block[20], 12))

    def __str_c(self, c, start, end):
        """ Returns the string representation of the given coefficient """
        return f'{c} (0x{", 0x".join(f"{b:02x}" for b in self.block[start:end])})'

    def __str__(self):
        return 'COEF:' \
            f'\n\tc0.: {self.__str_c(self.c0, 0, 2)}' \
            f'\n\tc1.: {self.__str_c(self.c1, 1, 3)}' \
            f'\n\tc00: {self.__str_c(self.c00, 3, 6)}' \
            f'\n\tc10: {self.__str_c(self.c10, 5, 8)}' \
            f'\n\tc01: {self.__str_c(self.c01, 8, 10)}' \
            f'\n\tc11: {self.__str_c(self.c11, 10, 12)}' \
            f'\n\tc20: {self.__str_c(self.c20, 12, 14)}' \
            f'\n\tc21: {self.__str_c(self.c21, 14, 16)}' \
            f'\n\tc30: {self.__str_c(self.c30, 16, 18)}' \
            f'\n\tc31: {self.__str_c(self.c31, 18, 20)}' \
            f'\n\tc40: {self.__str_c(self.c40, 19, 21)}'

    def t(self, t):
        """ Calculate temperature from scaled raw temperature value """
        return self.c0*0.5 + self.c1*t

    def p(self, p, t):
        """ Calculate pressure from scaled raw pressure and temperature values """
        p2 = p*p
        p3 = p2*p
        p4 = p3*p

        pt_corr = t*(self.c01 + self.c11*p + self.c21*p2 + self.c31*p3)
        return self.c00 + self.c10*p + self.c20*p2 + self.c30*p3 + self.c40*p4 + pt_corr

class Spl07003(Device):
    """ The class provides access to the basic functionality of SPL07-003 through I2C bus """
    def __init__(self, bus, address=DEFAULT_ADDRESS):
        super().__init__(bus, address)
        self.__c = Calibration()

        self.__clear_interrupt = False
        self.__prs_meas_time = None
        self.__prs_scale_factor = None
        self.__tmp_meas_time = None
        self.__tmp_scale_factor = None

    def __str_meas_cfg(self, cfg):
        coef_rdy = 'Yes' if cfg & 0x80 else 'No'
        sensor_rdy = 'Yes' if cfg & 0x40 else 'No'
        tmp_rdy = 'Yes' if cfg & 0x20 else 'No'
        prs_rdy = 'Yes' if cfg & 0x10 else 'No'
        tmp_ext = 'Yes' if cfg & 0x08 else 'No'
        meas_ctrl = _MEAS_CTRL_VALUES[cfg & 0x07]
        return f'COEF_RDY: {coef_rdy}, SENSOR_RDY: {sensor_rdy}, TMP_RDY: {tmp_rdy}, ' \
               f'PRS_RDY: {prs_rdy}, TMP_EXT: {tmp_ext}, MEAS_CTRL: {meas_ctrl}'

    def __str_int_sts(self, sts):
        fifo_full = 'Yes' if sts & 0x4 else 'No'
        tmp = 'Yes' if sts & 0x2 else 'No'
        prs = 'Yes' if sts & 0x1 else 'No'
        return f'INT_FIFO_FULL: {fifo_full}, INT_TMP: {tmp}, INT_PRS: {prs}'

    def __str_fifo_sts(self, sts):
        full = 'Yes' if sts & 0x2 else 'No'
        empty = 'Yes' if sts & 0x2 else 'No'
        return f'FIFO_FULL: {full}, FIFO_EMPTY: {empty}'

    def __read_cfg_safe(self):
        try:
            return self.read(_REG_MEAS_CFG), False
        except OSError as e:
            if e.errno == errno.EIO:
                return _REG_MEAS_CFG_MEAS_CTRL_NA, True
            raise

    def __wait_for_sensor(self, value, mask=None, timeout=None, step=0.0005):
        now = time.monotonic_ns()
        limit = now
        if timeout is not None:
            limit += timeout*1e9

        if mask is None:
            mask_str = f' (0x{value:02x})'
            mask = value
        else:
            mask_str = f' (0x{value:02x}:0x{mask:02x})'

        count = 0
        while timeout is None or now <= limit:
            time.sleep(step)
            count += 1

            cfg, error = self.__read_cfg_safe()
            if error:
                print(f'W {count}: I/O error - {self.__str_meas_cfg(cfg)}{mask_str}')
            else:
                print(f'W {count}: {self.__str_meas_cfg(cfg)}{mask_str}')
            if cfg & mask == value:
                return True

            now = time.monotonic_ns()

        return False

    def reset(self):
        """ Resets the sensor """
        self.write(_REG_RESET, 0x09)

    def read_coef(self):
        """ Reads the calibration coefficients from the sensor """
        start = time.monotonic_ns()
        if not self.__wait_for_sensor(_REG_MEAS_CFG_COEF_RDY, timeout=0.1, step=0.002):
            raise Error("Timeout while reading calibration coefficients")
        print(f'CC ({(time.monotonic_ns() - start)/1e6} ms)')

        self.__c = Calibration(self.read(_REG_BLOCK_COEF))
        return self.__c

    def write_cfg(self,
                  prs_os_rate=Oversampling.X16,
                  tmp_os_rate=Oversampling.X1,
                  interrupts=False):
        """ Writes the configuration of FIFO, interuppts, pressure and temperature measurements """
        prs_cfg = prs_os_rate.value & _REG_PT_CFG_PRC_MASK
        self.write(_REG_PRS_CFG, prs_cfg)

        tmp_cfg = tmp_os_rate.value & _REG_PT_CFG_PRC_MASK
        self.write(_REG_TMP_CFG, tmp_cfg)

        cfg = 0
        self.__clear_interrupt = interrupts
        if interrupts:
            cfg = _REG_CFG_REG_INT_PRS | _REG_CFG_REG_INT_TMP

        self.__prs_meas_time = _MEAS_TIME[prs_os_rate]
        self.__prs_scale_factor = _SCALE_FACTOR[prs_os_rate]
        if prs_os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_P_SHIFT

        self.__tmp_meas_time = _MEAS_TIME[tmp_os_rate]
        self.__tmp_scale_factor = _SCALE_FACTOR[tmp_os_rate]
        if tmp_os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_T_SHIFT

        self.write(_REG_CFG_REG, cfg)

        print(f'PRS_CFG: 0x{prs_cfg:02x}, TMP_CFG: 0x{tmp_cfg:02x}, CFG_REG: 0x{cfg:02x}')

    def measure_all(self):
        """ Runs continuous pressure and temperature measurements """
        if not self.__wait_for_sensor(_REG_MEAS_CFG_SENSOR_RDY, timeout=0.1):
            raise Error("Timeout while waiting for sensor to be ready for measurements")

        self.write(_REG_MEAS_CFG, 0b111)

        while True:
            time.sleep(0.001)

            count = 0
            t_rdy, p_rdy = False, False
            while count < 10000:
                time.sleep(0.001)
                count += 1

                cfg = self.read(_REG_MEAS_CFG)
                print(f'P+T {count}: {self.__str_meas_cfg(cfg)}')
                print(f'FIFO_STS: 0x{self.read(_REG_FIFO_STS):02x}')
                if not p_rdy and (cfg & 0x50 == 0x50):
                    data = self.read(_REG_BLOCK_PRS)
                    # print(f'PRS_BX: {" ".join(f"{x:02x}" for x in data)}')
                    p_raw_u = (data[0] << 16) | (data[1] << 8) | data[2]
                    # print(f'P_raw: 0x{p_raw_u:06x}')
                    p_raw = _u_to_s(p_raw_u, 24)
                    # print(f'P_raw: {p_raw}')
                    if t_rdy:
                        break
                    p_rdy = True

                if not t_rdy and (cfg & 0x60 == 0x60):
                    data = self.read(_REG_BLOCK_TMP)
                    # print(f'TMP_BX: {" ".join(f"{x:02x}" for x in data)}')
                    t_raw_u = (data[0] << 16) | (data[1] << 8) | data[2]
                    # print(f'T_raw: 0x{t_raw_u:06x}')
                    t_raw = _u_to_s(t_raw_u, 24)
                    # print(f'T_raw: {t_raw}')
                    if p_rdy:
                        break
                    t_rdy = True
            else:
                raise Error("Timeout while measuring pressure and temperature")

            p_raw_sc = p_raw/1572864
            t_raw_sc = t_raw/524288
            # print(f'P_raw_sc: {p_raw_sc:.6f}, T_raw_sc: {t_raw_sc:.6f}')

            yield self.__c.p(p_raw_sc, t_raw_sc), self.__c.t(t_raw_sc)

    def __wait_for_prs(self, timeout=None):
        time.sleep(self.__prs_meas_time)
        return True

    def __wait_for_tmp(self, timeout=None):
        time.sleep(self.__tmp_meas_time)
        return True

    def __measure_pressure(self, wait):
        self.write(_REG_MEAS_CFG, 0b001)

        if not wait:
            wait = self.__wait_for_prs

        start = time.monotonic_ns()
        if not wait(0.4):
            raise Error("Timeout while measuring pressure")
        end = time.monotonic_ns()

        cfg = None
        int_sts = None
        if self.__clear_interrupt:
            cfg = self.read(_REG_MEAS_CFG)
            int_sts = self.read(_REG_INT_STS)

        data = self.read(_REG_BLOCK_PRS)
        p_raw = _u_to_s((data[0] << 16) | (data[1] << 8) | data[2], 24)
        p_raw_sc = p_raw/self.__prs_scale_factor

        print(f'P ({(end - start)/1e6} ms)\n\t'
            f'PRS_B:    {" ".join(f"{x:02x}" for x in data)}\n\t'
            f'P_raw:    {p_raw}\n\t'
            f'P_raw_sc: {p_raw_sc:.6f}')
        if self.__clear_interrupt:
            print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                  f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')

        return p_raw_sc

    def __measure_temperature(self, wait):
        self.write(_REG_MEAS_CFG, 0b010)

        if not wait:
            wait = self.__wait_for_tmp

        start = time.monotonic_ns()
        if not wait(0.4):
            raise Error("Timeout while measuring temperature")
        end = time.monotonic_ns()

        cfg = None
        int_sts = None
        if self.__clear_interrupt:
            cfg = self.read(_REG_MEAS_CFG)
            int_sts = self.read(_REG_INT_STS)

        data = self.read(_REG_BLOCK_TMP)
        t_raw = _u_to_s((data[0] << 16) | (data[1] << 8) | data[2], 24)
        t_raw_sc = t_raw/self.__tmp_scale_factor

        print(f'T ({(end - start)/1e6} ms)\n\t'
            f'TMP_B:    {" ".join(f"{x:02x}" for x in data)}\n\t'
            f'T_raw:    {t_raw}\n\t'
            f'T_raw_sc: {t_raw_sc:.6f}')
        if self.__clear_interrupt:
            print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                  f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')

        return t_raw_sc

    def measure(self, wait=None):
        """ Measures pressure and temperature """
        p_raw_sc = self.__measure_pressure(wait)
        t_raw_sc = self.__measure_temperature(wait)
        return self.__c.p(p_raw_sc, t_raw_sc), self.__c.t(t_raw_sc)
