import argparse
import signal
import time

from .signal import Signal
from .bus import i2c
from .spl07_003 import Spl07_003

def make_args_paraser():
    return argparse.ArgumentParser(description='I2C Sensors communication CLI')

def main():
    make_args_paraser().parse_args()

    s = Signal(signal.SIGINT, signal.SIGTERM)

    ps = Spl07_003()
    with i2c() as bus:
        # ps.reset(bus)
        ps.read_coef(bus)

        print('COEF:')
        print(f'\tc0.: {ps.c0} (0x{ps.coef[0]:02x}, 0x{ps.coef[1]:02x})')
        print(f'\tc1.: {ps.c1} (0x{ps.coef[1]:02x}, 0x{ps.coef[2]:02x})')
        print(f'\tc00: 0x{ps.c00:05x} (0x{ps.coef[3]:02x}, 0x{ps.coef[4]:02x}, 0x{ps.coef[5]:02x})')
        print(f'\tc10: 0x{ps.c10:05x} (0x{ps.coef[5]:02x}, 0x{ps.coef[6]:02x}, 0x{ps.coef[7]:02x})')
        print(f'\tc01:  0x{ps.c01:04x} (0x{ps.coef[8]:02x}, 0x{ps.coef[9]:02x})')
        print(f'\tc11:  0x{ps.c11:04x} (0x{ps.coef[10]:02x}, 0x{ps.coef[11]:02x})')
        print(f'\tc20:  0x{ps.c20:04x} (0x{ps.coef[12]:02x}, 0x{ps.coef[13]:02x})')
        print(f'\tc21:  0x{ps.c21:04x} (0x{ps.coef[14]:02x}, 0x{ps.coef[15]:02x})')
        print(f'\tc30:  0x{ps.c30:04x} (0x{ps.coef[16]:02x}, 0x{ps.coef[17]:02x})')
        print(f'\tc31:  0x{ps.c31:04x} (0x{ps.coef[18]:02x}, 0x{ps.coef[19]:02x})')
        print(f'\tc40:   0x{ps.c40:03x} (0x{ps.coef[19]:02x}, 0x{ps.coef[20]:02x})')

        ps.write_cfg(bus)

        # measure = ps.measure_all(bus)
        count = 0
        while not s.is_signaled(): # and count < 3:
            # p, t = next(measure)
            p, t = ps.measure(bus)
            count += 1
            print(f'{count}: Temp: {t:0.4f} C, Pressure: {p:0.2f} Pa')
            time.sleep(1)

    return 0

