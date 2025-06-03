import sys
import signal
import threading

class Signal:
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
        return self.__signaled.is_set()

    def wait(self, timeout=None):
        return self.__signaled.wait(timeout)
