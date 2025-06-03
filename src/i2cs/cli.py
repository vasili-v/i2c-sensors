import argparse
import signal
import gpiozero

from .signal import Signal
from .bus import i2c
from .spl07_003 import Spl07_003, ALTERNATE_ADDRESS

def make_args_paraser():
    return argparse.ArgumentParser(description='I2C Sensors communication CLI')

def main():
    make_args_paraser().parse_args()

    s = Signal(signal.SIGINT, signal.SIGTERM)

    ps_int = gpiozero.DigitalInputDevice(20, pull_up=True)
    # ps_int = None
    led = gpiozero.LED(21)

    ps = Spl07_003(ALTERNATE_ADDRESS)
    with i2c() as bus:
        # ps.reset(bus)
        ps.read_coef(bus)

        print('COEF:')
        print(f'\tc0.: {ps.c0} (0x{ps.coef[0]:02x}, 0x{ps.coef[1]:02x})')
        print(f'\tc1.: {ps.c1} (0x{ps.coef[1]:02x}, 0x{ps.coef[2]:02x})')
        print(f'\tc00: {ps.c00} (0x{ps.coef[3]:02x}, 0x{ps.coef[4]:02x}, 0x{ps.coef[5]:02x})')
        print(f'\tc10: {ps.c10} (0x{ps.coef[5]:02x}, 0x{ps.coef[6]:02x}, 0x{ps.coef[7]:02x})')
        print(f'\tc01: {ps.c01} (0x{ps.coef[8]:02x}, 0x{ps.coef[9]:02x})')
        print(f'\tc11: {ps.c11} (0x{ps.coef[10]:02x}, 0x{ps.coef[11]:02x})')
        print(f'\tc20: {ps.c20} (0x{ps.coef[12]:02x}, 0x{ps.coef[13]:02x})')
        print(f'\tc21: {ps.c21} (0x{ps.coef[14]:02x}, 0x{ps.coef[15]:02x})')
        print(f'\tc30: {ps.c30} (0x{ps.coef[16]:02x}, 0x{ps.coef[17]:02x})')
        print(f'\tc31: {ps.c31} (0x{ps.coef[18]:02x}, 0x{ps.coef[19]:02x})')
        print(f'\tc40: {ps.c40} (0x{ps.coef[19]:02x}, 0x{ps.coef[20]:02x})')

        ps.write_cfg(bus)

        # measure = ps.measure_all(bus)
        count = 0
        while True: # and count < 3:
            # p, t = next(measure)
            led.on()
            try:
                p, t = ps.measure(bus, ps_int.wait_for_active)
            finally:
                led.off()
            count += 1
            print(f'{count}: Temp: {t:0.1f} C, Pressure: {p/100:0.1f} mbar')
            if s.wait(1):
                break

    return 0

