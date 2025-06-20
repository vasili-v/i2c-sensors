""" The submodule provides an entry point for its CLI """

import argparse
import signal
import collections

import gpiozero

from .signal import Signal
from .bus import i2c
from .spl07_003 import Spl07003, DEFAULT_ADDRESS, Oversampling

def _based_int(s):
    """ Converts a string to an integer, or raises an exception """
    s = s.strip().upper()
    if s.startswith('0X'):
        return int(s[2:], 16)

    if s.startswith('0B'):
        return int(s[2:], 2)

    if s.startswith('0'):
        try:
            return int(s[1:], 8)
        except ValueError:
            pass

    return int(s)

def make_args_paraser():
    """ Creates an argument parser """
    parser = argparse.ArgumentParser(description='I2C Sensors communication CLI')

    parser.add_argument(
            '--led', type=int,
            help='GPIO pin number for indicating LED (default: don\'t use LED)',
            metavar='PIN'
        )

    parser.add_argument(
            '--pressure-address', type=_based_int, default=DEFAULT_ADDRESS,
            help='SPL07-003 pressure sensor I2C address (default: 0x77, alternate: 0x76)',
            metavar='ADDR'
        )

    parser.add_argument(
            '--pressure-values', type=lambda x: x.upper(), default=_DEFAULT_PRESSURE_VALUES,
            choices=_PRESSURE_VALUES.keys(),
            help= 'Measure only pressure, temperature, or both (default: ' \
                 f'"{_DEFAULT_PRESSURE_VALUES}")',
        )

    parser.add_argument(
            '--pressure-interrupt', type=int,
            help='GPIO pin number for pressure interrupt (default: wait for data)',
            metavar='PIN'
        )

    parser.add_argument(
            '--pressure-prs-oversampling', type=lambda x: Oversampling[x],
            choices=list(Oversampling), default=Oversampling.X16,
            help= 'Oversampling rate for pressure measurement by pressure sensor (default: ' \
                 f'{Oversampling.X16.name})',
            metavar='RATE'
        )

    parser.add_argument(
            '--pressure-tmp-oversampling', type=lambda x: Oversampling[x],
            choices=list(Oversampling), default=Oversampling.X1,
            help= 'Oversampling rate for temperature measurement by pressure sensor (default: ' \
                 f'{Oversampling.X1.name})',
            metavar='RATE'
        )

    parser.add_argument(
            '--pressure-calibration-tmp', type=float,
            help='Temperature for pressure sensor calibration in °C (default: show uncalibrated ' \
                 'value)',
            metavar='TEMP'
        )

    return parser

class _MockLed:
    """ Mock class for LED to when pin is not specified """
    def on(self):
        """ Mock method for turning on the LED """

    def off(self):
        """Mock method for turning off the LED """

def main():
    """ Entry point for the CLI """
    args = make_args_paraser().parse_args()

    s = Signal(signal.SIGINT, signal.SIGTERM)

    wait = None
    if args.pressure_interrupt is not None:
        wait = gpiozero.DigitalInputDevice(args.pressure_interrupt, pull_up=True).wait_for_active

    led = gpiozero.LED(args.led) if args.led is not None else _MockLed()

    start_measurement, print_measurement = _PRESSURE_VALUES[args.pressure_values]
    print_measurement = print_measurement[args.pressure_calibration_tmp is None]

    with i2c() as bus:
        with Spl07003(bus, args.pressure_address) as ps:
            ps.reset()

            calibration = ps.read_coef()
            print(f'{calibration}')

            count = 0
            measurement = start_measurement(ps, args, wait)
            while True:
                led.on()
                try:
                    result = next(measurement)
                finally:
                    led.off()
                count += 1
                print_measurement(count, result)
                if s.wait(1):
                    break

    return 0

def prs_start_measurement(sensor, args, wait):
    """ Starts the pressure measurement """
    return sensor.cmd_mode_measure(
            prs_os_rate=args.pressure_prs_oversampling,
            tmp_os_rate=args.pressure_tmp_oversampling,
            interrupts=wait is not None,
            wait=wait,
        )

def prs_print_measurement(n, result):
    """ Prints a pressure and temperature measurement result """
    print(f'{n}: Temp: {result[1]:0.1f} C, Pressure: {result[0]/100:0.1f} mbar')

def prs_start_prs_measurement(sensor, args, wait):
    """ Starts the pressure measurement """
    sensor.calibration_tmp = args.pressure_calibration_tmp
    return sensor.cmd_mode_measure_prs(
            os_rate=args.pressure_prs_oversampling,
            interrupts=wait is not None,
            wait=wait,
        )

def prs_print_prs_measurement(n, result):
    """ Prints a pressure measurement result """
    print(f'{n}: Pressure: {result:0.6f}')

def prs_print_prs_mbar_measurement(n, result):
    """ Prints a pressure measurement result """
    print(f'{n}: Pressure: {result/100:0.1f} mbar')

def prs_start_tmp_measurement(sensor, args, wait):
    """ Starts the temperature measurement """
    return sensor.cmd_mode_measure_tmp(
            os_rate=args.pressure_tmp_oversampling,
            interrupts=wait is not None,
            wait=wait,
        )

def prs_print_tmp_measurement(n, result):
    """ Prints a temperature measurement result """
    print(f'{n}: Temp: {result:0.1f} C')

_DEFAULT_PRESSURE_VALUES = 'T+P'
_PRESSURE_VALUES_PRS = 'P'

def _constant_factory(value):
    """ Returns a function that always returns the given value """
    return lambda: value

_DEFAULT_PRESSURE_FUNCTIONS = (
    prs_start_measurement,
    collections.defaultdict(_constant_factory(prs_print_measurement)),
)

_PRESSURE_VALUES = {
    _DEFAULT_PRESSURE_VALUES: _DEFAULT_PRESSURE_FUNCTIONS,
    'P+T': _DEFAULT_PRESSURE_FUNCTIONS,
    _PRESSURE_VALUES_PRS: (prs_start_prs_measurement, {
            False: prs_print_prs_mbar_measurement,
            True: prs_print_prs_measurement,
        }),
    'T': (
            prs_start_tmp_measurement,
            collections.defaultdict(_constant_factory(prs_print_tmp_measurement))
        ),
}
