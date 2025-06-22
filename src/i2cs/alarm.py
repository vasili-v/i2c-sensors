""" The submodule provides a tool to wait for two independed time cycles """

import time

_NANOSECONDS = 1e9

def set_alarm(primary_cycle, primary_offset, secondory_cycle, secondary_offset):
    """ Returns an iterator which yields when one or both of time cycles end and duration of
        the wait. However, the iterator tries to postpone secondary cycle for duration of
        the primaries' offset if they overlap
    """
    primary_cycle = int(primary_cycle*_NANOSECONDS)
    primary_offset = int(primary_offset*_NANOSECONDS)
    secondary_cycle = int(secondory_cycle*_NANOSECONDS)
    secondary_offset = int(secondary_offset*_NANOSECONDS)

    now = time.monotonic_ns()
    start = now
    primary_deadline = now + primary_offset
    secondary_deadline = primary_deadline + secondary_offset

    while True:
        time.sleep((min(primary_deadline, secondary_deadline) - now)/_NANOSECONDS)

        now = time.monotonic_ns()
        primary_rdy = primary_deadline <= now
        secondary_rdy = secondary_deadline <= now

        if not (primary_rdy or secondary_rdy):
            continue

        if primary_rdy:
            primary_deadline += primary_cycle

        if secondary_rdy:
            secondary_deadline += secondary_cycle

        if primary_deadline + primary_offset > secondary_deadline and \
            secondary_deadline + secondary_offset > primary_deadline:
            secondary_deadline = primary_deadline + primary_offset

        yield primary_rdy, secondary_rdy, (now - start)/_NANOSECONDS
        start = now
