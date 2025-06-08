""" The submodule provides an event-based signal handling mechanism """

import sys
import signal
import threading

class Signal:
    """ The Signal class provides a way to handle signals using threading.Event """
    def __init__(self, *signals):
        self.__signaled = threading.Event()
        for s in signals:
            signal.signal(s, self.__handler)

    def __handler(self, signum, frame):
        msg = f'Got {signal.Signals(signum).name}'
        if frame:
            msg += f' at {frame.f_code.co_qualname}...'
        print(msg, file=sys.stderr)
        self.__signaled.set()

    def is_signaled(self):
        """ Check if one of the signals has been sent"""
        return self.__signaled.is_set()

    def wait(self, timeout=None):
        """ Wait for one of the signals to be sent, with an optional timeout """
        return self.__signaled.wait(timeout)
