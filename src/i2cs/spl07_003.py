""" The submodule provides a class for the I2C interface of SPL07-003 digital pressure sensor """

import dataclasses
import enum
import time
import errno

from .error import Error
from .device import Block, Device
from .alarm import set_alarm, set_alarm_external

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
_REG_PT_CFG_RATE_MASK = 0xf0

_REG_MEAS_CFG_MEAS_CTRL_IDLE = 0
_REG_MEAS_CFG_MEAS_CTRL_CMD_P = 1
_REG_MEAS_CFG_MEAS_CTRL_CMD_T = 2
_REG_MEAS_CFG_MEAS_CTRL_NA = 3
_REG_MEAS_CFG_MEAS_CTRL_BKG_P = 5
_REG_MEAS_CFG_MEAS_CTRL_BKG_T = 6
_REG_MEAS_CFG_MEAS_CTRL_BKG_PT = 7

_MEAS_CTRL_VALUES = {
    _REG_MEAS_CFG_MEAS_CTRL_IDLE: 'Idle',
    _REG_MEAS_CFG_MEAS_CTRL_CMD_P: 'P',
    _REG_MEAS_CFG_MEAS_CTRL_CMD_T: 'T',
    _REG_MEAS_CFG_MEAS_CTRL_NA: 'N/A',
    0b100: 'N/A',
    _REG_MEAS_CFG_MEAS_CTRL_BKG_P: 'P cont.',
    _REG_MEAS_CFG_MEAS_CTRL_BKG_T: 'T cont.',
    _REG_MEAS_CFG_MEAS_CTRL_BKG_PT: 'P+T cont.'
}

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

_REG_INT_STS_PRS = 0x01
_REG_INT_STS_TMP = 0x02

_FIFO_EMPTY = [0x80, 0x00, 0x00]
_FIFO_LSB = 0x1

class Rate(enum.IntEnum):
    """ The enumeration defines the pressure and temperature measurement rates for background mode
    """
    M1 = 0 << 4 # 1 measurements pr. sec.
    M2 = 1 << 4 # 2 measurements pr. sec.
    M4 = 2 << 4 # 4 measurements pr. sec.
    M8 = 3 << 4 # 8 measurements pr. sec.
    M16 = 4 << 4 # 16 measurements pr. sec.
    M32 = 5 << 4 # 32 measurements pr. sec.
    M64 = 6 << 4 # 64 measurements pr. sec.
    M128 = 7 << 4 # 128 measurements pr. sec.
    M25_16 = 8 << 4 # 25/16 (1.5625) sample/sec
    M25_8 = 9 << 4 # 25/8 (3.125) sample/sec
    M25_4 = 10 << 4 # 25/4 (6.25) sample/sec
    M25_2 = 11 << 4 # 25/2 (12.5) sample/sec
    M25 = 12 << 4 # 25 sample/sec
    M50 = 13 << 4 # 50 sample/sec
    M100 = 14 << 4 # 100 sample/sec
    M200 = 15 << 4 # 200 sample/sec

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

_DEFAULT_PRS_RATES = (Rate.M1, Oversampling.X16)
_DEFAULT_TMP_RATES = (Rate.M1, Oversampling.X1)

def _unpack_rates(rates, default):
    """ Unpacks the rates from the Rate enumeration """
    if isinstance(rates, Rate):
        return rates, default[1]

    if isinstance(rates, Oversampling):
        return default[0], rates

    try:
        first, second = rates
    except TypeError as e:
        raise TypeError(f'expected Rate, Oversampling pair, got {rates!r}') from e

    if isinstance(first, Rate) and isinstance(second, Oversampling):
        return first, second
    if isinstance(first, Oversampling) and isinstance(second, Rate):
        return second, first

    raise TypeError(f'expected Rate, Oversampling pair, got {rates!r}')

_MAX_OVERSAMPLING = {
    Rate.M1: Oversampling.X128,
    Rate.M25_16: Oversampling.X128,
    Rate.M2: Oversampling.X128,
    Rate.M25_8: Oversampling.X128,
    Rate.M4: Oversampling.X128,
    Rate.M25_4: Oversampling.X64,
    Rate.M8: Oversampling.X64,
    Rate.M25_2: Oversampling.X32,
    Rate.M16: Oversampling.X32,
    Rate.M25: Oversampling.X16,
    Rate.M32: Oversampling.X16,
    Rate.M50: Oversampling.X8,
    Rate.M64: Oversampling.X8,
    Rate.M100: Oversampling.X2,
    Rate.M128: Oversampling.X2,
    Rate.M200: Oversampling.X1,
}

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

_CYCLE_TIME = {
    Rate.M1: 1,
    Rate.M25_16: 16/25,
    Rate.M2: 1/2,
    Rate.M25_8: 8/25,
    Rate.M4: 1/4,
    Rate.M25_4: 4/25,
    Rate.M8: 1/8,
    Rate.M25_2: 2/25,
    Rate.M16: 1/16,
    Rate.M25: 1/25,
    Rate.M32: 1/32,
    Rate.M50: 1/50,
    Rate.M64: 1/64,
    Rate.M100: 1/100,
    Rate.M128: 1/128,
    Rate.M200: 1/200,
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

    def t_raw_sc(self, t):
        """ Calculate scaled raw temperature value for given temperature """
        return (t - self.c0*0.5)/self.c1

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
        self.__calibration_tmp = None

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

    @property
    def calibration_tmp(self):
        """ Returns temperature used for pressure measurements calibration """
        return self.__c.t(self.__calibration_tmp) if self.__calibration_tmp is not None else None

    @calibration_tmp.setter
    def calibration_tmp(self, t):
        """ Sets the temperature used for pressure measurements calibration """
        self.__calibration_tmp = self.__c.t_raw_sc(t) if t is not None else None

    def __wait_for_prs(self, timeout=None): # pylint: disable=unused-argument
        time.sleep(self.__prs_meas_time)
        return True

    def __wait_for_tmp(self, timeout=None): # pylint: disable=unused-argument
        time.sleep(self.__tmp_meas_time)
        return True

    def __calculate_prs(self, data):
        """ Calculates the scaled pressure from the raw data """
        raw = _u_to_s((data[0] << 16) | (data[1] << 8) | data[2], 24)
        raw_sc = raw/self.__prs_scale_factor

        print(f'\tPRS_B:    {" ".join(f"{x:02x}" for x in data)}\n'
              f'\tP_raw:    {raw}\n'
              f'\tP_raw_sc: {raw_sc:.6f}')

        return raw_sc

    def __calculate_tmp(self, data):
        """ Calculates the scaled temperature from the raw data """
        raw = _u_to_s((data[0] << 16) | (data[1] << 8) | data[2], 24)
        raw_sc = raw/self.__tmp_scale_factor

        print(f'\tTMP_B:    {" ".join(f"{x:02x}" for x in data)}\n'
              f'\tT_raw:    {raw}\n'
              f'\tT_raw_sc: {raw_sc:.6f}')

        return raw_sc

    def __measure_pressure(self, wait):
        self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_CMD_P)

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

        print(f'P ({(end - start)/1e6} ms)')
        p_raw_sc = self.__calculate_prs(self.read(_REG_BLOCK_PRS))

        if self.__clear_interrupt:
            print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                  f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')

        return p_raw_sc

    def __measure_temperature(self, wait):
        self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_CMD_T)

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

        print(f'T ({(end - start)/1e6} ms)')
        t_raw_sc = self.__calculate_tmp(self.read(_REG_BLOCK_TMP))

        if self.__clear_interrupt:
            print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                  f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')

        return t_raw_sc

    def cmd_mode_measure_prs(self,
                             os_rate=Oversampling.X16,
                             interrupts=False,
                             wait=None):
        """ Measures pressure in command mode """
        prs_cfg = os_rate.value & _REG_PT_CFG_PRC_MASK
        self.write(_REG_PRS_CFG, prs_cfg)

        self.write(_REG_TMP_CFG, 0)

        cfg = 0
        self.__clear_interrupt = interrupts
        if interrupts:
            cfg = _REG_CFG_REG_INT_PRS

        self.__prs_meas_time = _MEAS_TIME[os_rate]
        self.__prs_scale_factor = _SCALE_FACTOR[os_rate]
        if os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_P_SHIFT

        self.write(_REG_CFG_REG, cfg)

        print(f'PRS_CFG: 0x{prs_cfg:02x}, CFG_REG: 0x{cfg:02x}')

        while True:
            p_raw_sc = self.__measure_pressure(wait)
            t_raw_sc = self.__calibration_tmp
            yield self.__c.p(p_raw_sc, t_raw_sc) if t_raw_sc is not None else p_raw_sc

    def cmd_mode_measure_tmp(self,
                             os_rate=Oversampling.X1,
                             interrupts=False,
                             wait=None):
        """ Measures temperature in command mode """
        self.write(_REG_PRS_CFG, 0)

        tmp_cfg = os_rate.value & _REG_PT_CFG_PRC_MASK
        self.write(_REG_TMP_CFG, tmp_cfg)

        cfg = 0
        self.__clear_interrupt = interrupts
        if interrupts:
            cfg = _REG_CFG_REG_INT_TMP

        self.__tmp_meas_time = _MEAS_TIME[os_rate]
        self.__tmp_scale_factor = _SCALE_FACTOR[os_rate]
        if os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_T_SHIFT

        self.write(_REG_CFG_REG, cfg)

        print(f'TMP_CFG: 0x{tmp_cfg:02x}, CFG_REG: 0x{cfg:02x}')

        while True:
            t_raw_sc = self.__measure_temperature(wait)
            yield self.__c.t(t_raw_sc)

    def cmd_mode_measure(self,
                         prs_os_rate=Oversampling.X16,
                         tmp_os_rate=Oversampling.X1,
                         interrupts=False,
                         wait=None):
        """ Measures pressure and temperature in command mode """
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

        while True:
            p_raw_sc = self.__measure_pressure(wait)
            t_raw_sc = self.__measure_temperature(wait)
            yield self.__c.p(p_raw_sc, t_raw_sc), self.__c.t(t_raw_sc)

    def __configure_bkg_prs_measurement(self, rates, interrupts):
        """ Configures the background mode measurement for pressure """
        rate, os_rate = _unpack_rates(rates, _DEFAULT_PRS_RATES)
        os_rate = min(os_rate, _MAX_OVERSAMPLING[rate])
        prs_cfg = (rate & _REG_PT_CFG_RATE_MASK) | (os_rate.value & _REG_PT_CFG_PRC_MASK)
        self.write(_REG_PRS_CFG, prs_cfg)

        self.write(_REG_TMP_CFG, 0)

        cfg = 0
        self.__clear_interrupt = interrupts
        if interrupts:
            cfg |= _REG_CFG_REG_INT_PRS

        self.__prs_meas_time = _CYCLE_TIME[rate]
        self.__prs_scale_factor = _SCALE_FACTOR[os_rate]
        if os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_P_SHIFT

        self.write(_REG_CFG_REG, cfg)

        meas_cfg = _REG_MEAS_CFG_MEAS_CTRL_BKG_P
        print(f'PRS_CFG: 0x{prs_cfg:02x}, '
              f'CFG_REG: 0x{cfg:02x}, '
              f'MEAS_CFG: 0x{meas_cfg:x} ({_MEAS_CTRL_VALUES[meas_cfg]})')

    def bkg_mode_measure_prs(self,
                             rates=_DEFAULT_PRS_RATES,
                             interrupts=False,
                             wait=None):
        """ Measures pressure in background mode """
        self.__configure_bkg_prs_measurement(rates, interrupts)
        if not wait:
            wait = self.__wait_for_prs

        while True:
            self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_BKG_P)
            start = time.monotonic_ns()
            if not wait(2):
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

            t_raw_sc = self.__calibration_tmp
            yield self.__c.p(p_raw_sc, t_raw_sc) if t_raw_sc is not None else p_raw_sc

    def __configure_bkg_tmp_measurement(self, rates, interrupts):
        """ Configures the background mode measurement for temperature """
        self.write(_REG_PRS_CFG, 0)

        rate, os_rate = _unpack_rates(rates, _DEFAULT_TMP_RATES)
        os_rate = min(os_rate, _MAX_OVERSAMPLING[rate])
        tmp_cfg = (rate & _REG_PT_CFG_RATE_MASK) | (os_rate.value & _REG_PT_CFG_PRC_MASK)
        self.write(_REG_TMP_CFG, tmp_cfg)

        cfg = 0
        self.__clear_interrupt = interrupts
        if interrupts:
            cfg |= _REG_CFG_REG_INT_TMP

        self.__tmp_meas_time = _CYCLE_TIME[rate]
        self.__tmp_scale_factor = _SCALE_FACTOR[os_rate]
        if os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_T_SHIFT

        self.write(_REG_CFG_REG, cfg)

        meas_cfg = _REG_MEAS_CFG_MEAS_CTRL_BKG_T

        print(f'TMP_CFG: 0x{tmp_cfg:02x}, '
              f'CFG_REG: 0x{cfg:02x}, '
              f'MEAS_CFG: 0x{meas_cfg:x} ({_MEAS_CTRL_VALUES[meas_cfg]})')

    def bkg_mode_measure_tmp(self,
                             rates=_DEFAULT_TMP_RATES,
                             interrupts=False,
                             wait=None):
        """ Measures temperature in background mode """
        self.__configure_bkg_tmp_measurement(rates, interrupts)
        if not wait:
            wait = self.__wait_for_tmp

        while True:
            self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_BKG_T)
            start = time.monotonic_ns()
            if not wait(2):
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
                f't_raw:    {t_raw}\n\t'
                f't_raw_sc: {t_raw_sc:.6f}')
            if self.__clear_interrupt:
                print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                      f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')

            yield self.__c.t(t_raw_sc)

    def __configure_bkg_measurement(self, prs_rates, tmp_rates, interrupts, fifo, wait):
        """ Configures the background mode measurement for pressure and temperature """
        prs_rate, prs_os_rate = _unpack_rates(prs_rates, _DEFAULT_PRS_RATES)
        prs_os_rate = min(prs_os_rate, _MAX_OVERSAMPLING[prs_rate])
        prs_cfg = (prs_rate & _REG_PT_CFG_RATE_MASK) | (prs_os_rate.value & _REG_PT_CFG_PRC_MASK)
        self.write(_REG_PRS_CFG, prs_cfg)

        tmp_rate, tmp_os_rate = _unpack_rates(tmp_rates, _DEFAULT_PRS_RATES)
        tmp_os_rate = min(tmp_os_rate, _MAX_OVERSAMPLING[tmp_rate])
        tmp_cfg = (tmp_rate & _REG_PT_CFG_RATE_MASK) | (tmp_os_rate.value & _REG_PT_CFG_PRC_MASK)
        self.write(_REG_TMP_CFG, tmp_cfg)

        cfg = 0
        self.__clear_interrupt = True
        if interrupts:
            if fifo:
                cfg |= _REG_CFG_REG_INT_FIFO
            else:
                cfg |= _REG_CFG_REG_INT_PRS | _REG_CFG_REG_INT_TMP

        if fifo:
            cfg |= _REG_CFG_REG_FIFO_EN

        self.__prs_scale_factor = _SCALE_FACTOR[prs_os_rate]
        self.__tmp_scale_factor = _SCALE_FACTOR[tmp_os_rate]
        if prs_os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_P_SHIFT
        if tmp_os_rate > Oversampling.X8:
            cfg |= _REG_CFG_REG_T_SHIFT

        self.write(_REG_CFG_REG, cfg)

        meas_cfg = _REG_MEAS_CFG_MEAS_CTRL_BKG_PT
        print(f'PRS_CFG: 0x{prs_cfg:02x}, '
              f'TMP_CFG: 0x{tmp_cfg:02x}, '
              f'CFG_REG: 0x{cfg:02x}, '
              f'MEAS_CFG: 0x{meas_cfg:x} ({_MEAS_CTRL_VALUES[meas_cfg]})')

        if wait is None:
            return set_alarm(
                    _CYCLE_TIME[prs_rate], _MEAS_TIME[prs_os_rate],
                    _CYCLE_TIME[tmp_rate], _MEAS_TIME[tmp_os_rate],
                )

        return set_alarm_external(wait, 64 if fifo else 2)

    def bkg_mode_measure(self, # pylint: disable=too-many-locals
                         prs_rates=_DEFAULT_PRS_RATES,
                         tmp_rates=_DEFAULT_TMP_RATES,
                         interrupts=False,
                         wait=None):
        """ Measures temperature in background mode """
        alarm = self.__configure_bkg_measurement(prs_rates, tmp_rates, interrupts, False, wait)

        p_raw_sc, p, t_raw_sc, t = None, None, None, None

        self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_BKG_PT)

        for prs_rdy, tmp_rdy, dt in alarm:
            cfg = None
            int_sts = None
            if self.__clear_interrupt:
                cfg = self.read(_REG_MEAS_CFG)
                int_sts = self.read(_REG_INT_STS)
                if not prs_rdy and not tmp_rdy:
                    prs_rdy = bool(int_sts & _REG_INT_STS_PRS)
                    tmp_rdy = bool(int_sts & _REG_INT_STS_TMP)

                if not prs_rdy and not tmp_rdy:
                    print(f'M ({dt*1e3:.2f} ms)')
                    print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                          f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')
                    yield None

            prs_data = self.read(_REG_BLOCK_PRS)
            tmp_data = self.read(_REG_BLOCK_TMP)

            print(f'M ({dt*1e3:.2f} ms)')
            if prs_rdy:
                p_raw_sc = self.__calculate_prs(prs_data)

            if tmp_rdy:
                t_raw_sc = self.__calculate_tmp(tmp_data)
                t = self.__c.t(t_raw_sc)

            if self.__clear_interrupt:
                print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                      f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')

            if p_raw_sc is not None and t_raw_sc is not None:
                p = self.__c.p(p_raw_sc, t_raw_sc)

            yield p, t

            self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_BKG_PT)

    def bkg_mode_measure_fifo(self, # pylint: disable=too-many-locals
                         prs_rates=_DEFAULT_PRS_RATES,
                         tmp_rates=_DEFAULT_TMP_RATES,
                         interrupts=False,
                         wait=None):
        """ Measures temperature in background mode """
        alarm = self.__configure_bkg_measurement(prs_rates, tmp_rates, interrupts, True, wait)

        p_raw_sc, p, t_raw_sc, t = None, None, None, None

        self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_BKG_PT)

        for _, _, dt in alarm:
            cfg = None
            int_sts = None
            if self.__clear_interrupt:
                cfg = self.read(_REG_MEAS_CFG)
                int_sts = self.read(_REG_INT_STS)

            data = self.read(_REG_BLOCK_PRS)
            if data == _FIFO_EMPTY:
                if self.__clear_interrupt:
                    print(f'M ({dt*1e3:.2f} ms) - FIFO is empty')
                    print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                          f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')
                yield None
            else:
                print(f'M ({dt*1e3:.2f} ms)')
                while data != _FIFO_EMPTY:
                    if data[-1] & _FIFO_LSB:
                        p_raw_sc = self.__calculate_prs(data)

                    if not data[-1] & _FIFO_LSB:
                        t_raw_sc = self.__calculate_tmp(data)
                        t = self.__c.t(t_raw_sc)

                    if p_raw_sc is not None and t_raw_sc is not None:
                        p = self.__c.p(p_raw_sc, t_raw_sc)

                    yield p, t

                    data = self.read(_REG_BLOCK_PRS)

                if self.__clear_interrupt:
                    print(f'\tINT_STS:  {self.__str_int_sts(int_sts)}\n'
                          f'\tMEAS_CFG: {self.__str_meas_cfg(cfg)}')

            self.write(_REG_MEAS_CFG, _REG_MEAS_CFG_MEAS_CTRL_BKG_PT)
