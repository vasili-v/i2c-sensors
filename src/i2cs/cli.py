""" The submodule provides an entry point for its CLI """

import argparse
import signal
import gpiozero

from .signal import Signal
from .bus import i2c
from .spl07_003 import Spl07003, DEFAULT_ADDRESS

def make_args_paraser():
    """ Creates an argument parser """
    parser = argparse.ArgumentParser(description='I2C Sensors communication CLI')

    parser.add_argument(
            '--pressure', type=int, default=DEFAULT_ADDRESS,
            help='SPL07-003 pressure sensor I2C address (default: 0x77, alternate: 0x76)',
            metavar='ADDR'
        )

    parser.add_argument(
            '--pressure-interrupt', type=int,
            help='GPIO pin number for pressure interrupt (default: poll for data)',
            metavar='PIN'
        )

    return parser

def main():
    """ Entry point for the CLI """
    args = make_args_paraser().parse_args()

    s = Signal(signal.SIGINT, signal.SIGTERM)

    wait = None
    if args.pressure_interrupt is not None:
        wait = gpiozero.DigitalInputDevice(args.pressure_interrupt, pull_up=True).wait_for_active

    led = gpiozero.LED(21)

    with i2c() as bus:
        with Spl07003(bus, args.pressure) as ps:
            ps.reset()

            calibration = ps.read_coef()
            print(f'{calibration}')

            ps.write_cfg()

            count = 0
            while True:
                led.on()
                try:
                    p, t = ps.measure(wait)
                finally:
                    led.off()
                count += 1
                print(f'{count}: Temp: {t:0.1f} C, Pressure: {p/100:0.1f} mbar')
                if s.wait(1):
                    break

    return 0
