""" The submodule provides an entry point for its CLI """

import argparse
import signal
import gpiozero

from .signal import Signal
from .bus import i2c
from .spl07_003 import Spl07003, ALTERNATE_ADDRESS

def make_args_paraser():
    """ Creates an argument parser """
    return argparse.ArgumentParser(description='I2C Sensors communication CLI')

def main():
    """ Entry point for the CLI """
    make_args_paraser().parse_args()

    s = Signal(signal.SIGINT, signal.SIGTERM)

    ps_int = gpiozero.DigitalInputDevice(20, pull_up=True)
    # ps_int = None
    led = gpiozero.LED(21)

    ps = Spl07003(ALTERNATE_ADDRESS)
    with i2c() as bus:
        # ps.reset(bus)
        ps.read_coef(bus)

        print(f'{ps.calibration}')

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
