""" The submodule provides a tool to wait for two independed time cycles """

import time

_NANOSECONDS = 1e9

def set_alarm(pri_cycle, pri_offset, sec_cycle, sec_offset):
    """ Returns an iterator which yields when one or both of time cycles end and duration of
        the wait. However, the iterator tries to postpone secondary cycle for duration of
        the primaries' offset if they overlap
    """
    pri_cycle_ns = int(pri_cycle*_NANOSECONDS)
    pri_offset_ns = int(pri_offset*_NANOSECONDS)
    sec_cycle_ns = int(sec_cycle*_NANOSECONDS)
    sec_offset_ns = int(sec_offset*_NANOSECONDS)

    now = time.monotonic_ns()
    start = now
    pri_deadline = now + pri_offset_ns
    sec_deadline = pri_deadline + sec_offset_ns

    while True:
        dt = min(pri_deadline, sec_deadline) - time.monotonic_ns()
        time.sleep(max(dt/_NANOSECONDS, 0))

        now = time.monotonic_ns()
        primary_rdy = pri_deadline <= now
        secondary_rdy = sec_deadline <= now

        if not (primary_rdy or secondary_rdy):
            continue

        if primary_rdy:
            pri_deadline += pri_cycle_ns

        if secondary_rdy:
            sec_deadline += sec_cycle_ns

        if pri_deadline > sec_deadline - sec_offset_ns and \
            sec_deadline > pri_deadline - pri_offset_ns:
            sec_deadline = pri_deadline + sec_offset_ns

        yield primary_rdy, secondary_rdy, (now - start)/_NANOSECONDS
        start = now

def set_alarm_external(wait, timeout=None):
    """ Returns an iterator which yields when given wait function returns True. If the wait
        function returns False, stops the iteration """
    start = time.monotonic_ns()
    while wait(timeout):
        now = time.monotonic_ns()
        yield False, False, (now - start)/_NANOSECONDS
        start = now
