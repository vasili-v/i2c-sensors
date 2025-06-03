# The submodule provides a class for the I2C interface of SPL07-003 digital pressure sensor

import time

from .error import Error

DEFAULT_ADDRESS=0x77
ALTERNATE_ADDRESS=0x76

_REG_BLOCK_PRS=0x00
_REG_BLOCK_SIZE_PRS=3

_REG_BLOCK_TMP=0x03
_REG_BLOCK_SIZE_TMP=3

_REG_PRS_CFG=0x06
_REG_TMP_CFG=0x07
_REG_MEAS_CFG=0x08
_REG_CFG_REG=0x09
_REG_INT_STS=0x0a
_REG_FIFO_STS=0x0b
_REG_RESET=0x0c

_REG_BLOCK_COEF=0x10
_REG_BLOCK_SIZE_COEF=21

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

def _u_to_s(u, bits):
    ''' Convert 2's complement integer to signed number '''
    mask = (1 << (bits-1))
    return -mask + (u & (mask-1)) if u & mask else u

class Spl07_003:
    ''' The class provides access to the basic functionality of SPL07-003 through I2C bus '''
    def __init__(self, address=DEFAULT_ADDRESS):
        self.__address = address
        self.coef = _REG_BLOCK_SIZE_COEF*[0]
        self.c0 = 0
        self.c1 = 0
        self.c00 = 0
        self.c10 = 0
        self.c01 = 0
        self.c11 = 0
        self.c20 = 0
        self.c21 = 0
        self.c30 = 0
        self.c31 = 0
        self.c40 = 0

    def __str_meas_cfg(self, cfg):
        coef_rdy = 'Yes' if cfg & 0x80 else 'No'
        sensor_rdy = 'Yes' if cfg & 0x40 else 'No'
        tmp_rdy = 'Yes' if cfg & 0x20 else 'No'
        prs_rdy = 'Yes' if cfg & 0x10 else 'No'
        tmp_ext = 'Yes' if cfg & 0x08 else 'No'
        meas_ctrl = _MEAS_CTRL_VALUES[cfg & 0x07]
        return f'COEF_RDY: {coef_rdy}, SENSOR_RDY: {sensor_rdy}, TMP_RDY: {tmp_rdy}, PRS_RDY: {prs_rdy}, TMP_EXT: {tmp_ext}, MEAS_CTRL: {meas_ctrl}'
    
    def __str_int_sts(self, sts):
        fifo_full = 'Yes' if sts & 0x4 else 'No'
        tmp = 'Yes' if sts & 0x2 else 'No'
        prs = 'Yes' if sts & 0x1 else 'No'
        return f'INT_FIFO_FULL: {fifo_full}, INT_TMP: {tmp}, INT_PRS: {prs}'
    
    def __str_fifo_sts(self, sts):
        full = 'Yes' if sts & 0x2 else 'No'
        empty = 'Yes' if sts & 0x2 else 'No'
        return f'FIFO_FULL: {full}, FIFO_EMPTY: {empty}'

    def __wait_for_sensor(self, bus, mask):
        cfg = bus.read_byte(self.__address, _REG_MEAS_CFG)
        # print(f'W: {self.__str_meas_cfg(cfg)} (0x{mask:02x})')
        count = 0
        while count < 10:
            count += 1
            cfg = bus.read_byte(self.__address, _REG_MEAS_CFG)
            # print(f'W {count}: {self.__str_meas_cfg(cfg)} (0x{mask:02x})')
            if cfg & mask == mask:
                break

            time.sleep(0.001)
        else:
            raise Error("Timeout while waiting for sensor to be ready")

    def reset(self, bus):
        bus.write_byte_data(self.__address, _REG_RESET, 0x09)
        self.__wait_for_sensor(bus, 0xc0)

    def read_coef(self, bus):
        # self.__wait_for_sensor(bus, 0xc0)

        self.coef = bus.read_i2c_block_data(self.__address, _REG_BLOCK_COEF, _REG_BLOCK_SIZE_COEF)
        self.c0 = _u_to_s((self.coef[0] << 4) | (self.coef[1] >> 4), 12)
        self.c1 = _u_to_s(((self.coef[1] & 0xf) << 8) | self.coef[2], 12)
        self.c00 = _u_to_s((self.coef[3] << 12) | (self.coef[4] << 4) | (self.coef[5] >> 4), 20)
        self.c10 = _u_to_s(((self.coef[5] & 0xf) << 16) | (self.coef[6] << 8) | self.coef[7], 20)
        self.c01 = _u_to_s((self.coef[8] << 8) | self.coef[9], 16)
        self.c11 = _u_to_s((self.coef[10] << 8) | self.coef[11], 16)
        self.c20 = _u_to_s((self.coef[12] << 8) | self.coef[13], 16)
        self.c21 = _u_to_s((self.coef[14] << 8) | self.coef[15], 16)
        self.c30 = _u_to_s((self.coef[16] << 8) | self.coef[17], 16)
        self.c31 = _u_to_s((self.coef[18] << 4) | (self.coef[19] >> 4), 12)
        self.c40 = _u_to_s(((self.coef[19] & 0xf) << 8) | self.coef[20], 12)

    def write_cfg(self, bus):
        bus.write_byte_data(self.__address, _REG_PRS_CFG, 0b00000100)
        bus.write_byte_data(self.__address, _REG_TMP_CFG, 0b00000100)
        bus.write_byte_data(self.__address, _REG_CFG_REG, 0b00111100)

    def __calculate(self, p_raw_sc, t_raw_sc):
        p2_raw_sc = p_raw_sc*p_raw_sc
        p3_raw_sc = p2_raw_sc*p_raw_sc
        p4_raw_sc = p3_raw_sc*p_raw_sc

        p = self.c00 + self.c10*p_raw_sc + self.c20*p2_raw_sc + self.c30*p3_raw_sc + self.c40*p4_raw_sc + \
            t_raw_sc*(self.c01 + self.c11*p_raw_sc + self.c21*p2_raw_sc + self.c31*p3_raw_sc)
        t = self.c0*0.5 + self.c1*t_raw_sc

        return p, t

    def measure_all(self, bus):
        self.__wait_for_sensor(bus, 0b01000000)
        bus.write_byte_data(self.__address, _REG_MEAS_CFG, 0b111)

        while True:
            time.sleep(0.001)

            count = 0
            t_rdy, p_rdy = False, False
            while count < 10000:
                time.sleep(0.001)
                count += 1

                cfg = bus.read_byte(self.__address, _REG_MEAS_CFG)
                print(f'P+T {count}: {self.__str_meas_cfg(cfg)}')
                print(f'FIFO_STS: 0x{bus.read_byte(self.__address, _REG_FIFO_STS):02x}')
                if not p_rdy and (cfg & 0x50 == 0x50):
                    data = bus.read_i2c_block_data(self.__address, _REG_BLOCK_PRS, _REG_BLOCK_SIZE_PRS)
                    # print(f'PRS_BX: {" ".join(f"{x:02x}" for x in data)}')
                    p_raw_u = (data[0] << 16) | (data[1] << 8) | data[2]
                    # print(f'P_raw: 0x{p_raw_u:06x}')
                    p_raw = _u_to_s(p_raw_u, 24)
                    # print(f'P_raw: {p_raw}')
                    if t_rdy:
                        break
                    p_rdy = True

                if not t_rdy and (cfg & 0x60 == 0x60):
                    data = bus.read_i2c_block_data(self.__address, _REG_BLOCK_TMP, _REG_BLOCK_SIZE_TMP)
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

            yield self.__calculate(p_raw_sc, t_raw_sc)

    def __measure_pressure(self, bus, wait):
        self.__wait_for_sensor(bus, 0b01000000)
        bus.write_byte_data(self.__address, _REG_MEAS_CFG, 0b001)

        if wait:
            start = time.monotonic_ns()
            if not wait(0.3):
                raise Error("Timeout while measuring pressure")
            print(f'P ({(time.monotonic_ns() - start)/1e6} ms)\n\t'
                f'MEAS_CFG: {self.__str_meas_cfg(bus.read_byte(self.__address, _REG_MEAS_CFG))}\n\t'
                f'INT_STS:  {self.__str_int_sts(bus.read_byte(self.__address, _REG_INT_STS))}\n\t'
                f'FIFO_STS: {self.__str_fifo_sts(bus.read_byte(self.__address, _REG_FIFO_STS))}')
        else:
            count = 0
            while count < 10:
                time.sleep(0.03)
                count += 1
                cfg = bus.read_byte(self.__address, _REG_MEAS_CFG)
                # print(f'P {count}: {self.__str_meas_cfg(cfg)}')
                if cfg & 0x10:
                    print(f'P ({count})\n\t'
                        f'MEAS_CFG: {self.__str_meas_cfg(cfg)}\n\t'
                        f'INT_STS:  {self.__str_int_sts(bus.read_byte(self.__address, _REG_INT_STS))}\n\t'
                        f'FIFO_STS: {self.__str_fifo_sts(bus.read_byte(self.__address, _REG_FIFO_STS))}')
                    break
            else:
                raise Error("Timeout while measuring pressure")

        data = bus.read_i2c_block_data(self.__address, _REG_BLOCK_PRS, _REG_BLOCK_SIZE_PRS)
        return _u_to_s((data[0] << 16) | (data[1] << 8) | data[2], 24)/253952

    def __measure_temperature(self, bus, wait):
        self.__wait_for_sensor(bus, 0b01000000)
        bus.write_byte_data(self.__address, _REG_MEAS_CFG, 0b010)

        if wait:
            start = time.monotonic_ns()
            if not wait(0.3):
                raise Error("Timeout while measuring pressure")
            print(f'T ({(time.monotonic_ns() - start)/1e6} ms)\n\t'
                f'MEAS_CFG: {self.__str_meas_cfg(bus.read_byte(self.__address, _REG_MEAS_CFG))}\n\t'
                f'INT_STS:  {self.__str_int_sts(bus.read_byte(self.__address, _REG_INT_STS))}\n\t'
                f'FIFO_STS: {self.__str_fifo_sts(bus.read_byte(self.__address, _REG_FIFO_STS))}')
        else:
            count = 0
            while count < 10:
                time.sleep(0.03)
                count += 1

                cfg = bus.read_byte(self.__address, _REG_MEAS_CFG)
                # print(f'T {count}: {self.__str_meas_cfg(cfg)}')
                if cfg & 0x20:
                    s = ''
                    if ps_int is not None:
                        s = f'\n\tINT: {ps_int.is_active}'
                    print(f'T ({count})\n\t'
                        f'MEAS_CFG: {self.__str_meas_cfg(cfg)}\n\t'
                        f'INT_STS:  {self.__str_int_sts(bus.read_byte(self.__address, _REG_INT_STS))}\n\t'
                        f'FIFO_STS: {self.__str_fifo_sts(bus.read_byte(self.__address, _REG_FIFO_STS))}'
                        f'{s}')
                    break
            else:
                raise Error("Timeout while measuring temperature")

        data = bus.read_i2c_block_data(self.__address, _REG_BLOCK_TMP, _REG_BLOCK_SIZE_TMP)
        return _u_to_s((data[0] << 16) | (data[1] << 8) | data[2], 24)/253952

    def measure(self, bus, wait=None):
        return self.__calculate(
                self.__measure_pressure(bus, wait),
                self.__measure_temperature(bus, wait)
            )
