import argparse
import signal

from .signal import Signal

def make_args_paraser():
    return argparse.ArgumentParser(description='I2C Sensors communication CLI')

def main():
    make_args_paraser().parse_args()
    Signal(signal.SIGINT, signal.SIGTERM).wait()
    return 0
