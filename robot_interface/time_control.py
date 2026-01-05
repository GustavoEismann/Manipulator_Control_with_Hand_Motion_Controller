"""
High-Resolution Timing Functions for Performance Measurement

This module provides microsecond-precision timing functions for both Windows
and Linux platforms. It uses OS-specific low-level performance counters to
achieve timing accuracy suitable for real-time control applications.

Key Features:
    - Microsecond (μs) precision timing
    - Cross-platform support (Windows and Linux)
    - Monotonic time (unaffected by system clock changes)
    - Arduino-style API (micros, millis, delay functions)

Functions:
    micros() -> float:
        Returns current timestamp in microseconds (1 μs = 10^-6 s)
    
    millis() -> float:
        Returns current timestamp in milliseconds (1 ms = 10^-3 s)
    
    delay(ms):
        Blocking delay for specified milliseconds
    
    delayMicroseconds(us):
        Blocking delay for specified microseconds

Platform Implementation:
    Windows: QueryPerformanceCounter (~2 MHz QPC clock)
    Linux: clock_gettime with CLOCK_MONOTONIC_RAW

Typical Usage:
    # Measure loop execution time
    start = micros()
    # ... do work ...
    elapsed = micros() - start
    print(f"Execution time: {elapsed:.2f} μs")
    
    # Fixed-rate loop
    while True:
        loop_start = millis()
        # ... control logic ...
        delay(10)  # Target 100 Hz loop

Author: Gustavo Eismann
Date: 05/jan/2026
"""

import ctypes
import os

VERSION = '0.2.0'

# ========================================================================
# OS-SPECIFIC LOW-LEVEL TIMING IMPLEMENTATIONS
# ========================================================================

if (os.name == 'nt'):  # Windows Implementation
    # Uses Windows QueryPerformanceCounter API for high-resolution timing
    def micros():
        "return a timestamp in microseconds (us)"
        tics = ctypes.c_int64()
        freq = ctypes.c_int64()

        # get ticks on the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics))

        # get the actual freq. of the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq))

        t_us = tics.value * 1e6 / freq.value
        return t_us


    def millis():
        "return a timestamp in milliseconds (ms)"
        tics = ctypes.c_int64()
        freq = ctypes.c_int64()

        # get ticks on the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceCounter(ctypes.byref(tics))

        # get the actual freq. of the internal ~2MHz QPC clock
        ctypes.windll.Kernel32.QueryPerformanceFrequency(ctypes.byref(freq))

        t_ms = tics.value * 1e3 / freq.value
        return t_ms


elif (os.name == 'posix'):  # for Linux:

    # Constants:
    CLOCK_MONOTONIC_RAW = 4  # see <linux/time.h> here: https://github.com/torvalds/linux/blob/master/include/uapi/linux/time.h

    # prepare ctype timespec structure of {long, long}
    class timespec(ctypes.Structure):
        _fields_ =\
            [
                ('tv_sec', ctypes.c_long),
                ('tv_nsec', ctypes.c_long)
            ]

    # Configure Python access to the clock_gettime C library, via ctypes:
    # Documentation:
    # -ctypes.CDLL: https://docs.python.org/3.2/library/ctypes.html
    # -librt.so.1 with clock_gettime: https://docs.oracle.com/cd/E36784_01/html/E36873/librt-3lib.html #-
    # -Linux clock_gettime(): http://linux.die.net/man/3/clock_gettime
    librt = ctypes.CDLL('librt.so.1', use_errno=True)
    clock_gettime = librt.clock_gettime
    # specify input arguments and types to the C clock_gettime() function
    # (int clock_ID, timespec* t)
    clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(timespec)]


    def monotonic_time():
        "return a timestamp in seconds (sec)"
        t = timespec()
        # (Note that clock_gettime() returns 0 for success, or -1 for failure, in
        # which case errno is set appropriately)
        # -see here: http://linux.die.net/man/3/clock_gettime
        if clock_gettime(CLOCK_MONOTONIC_RAW, ctypes.pointer(t)) != 0:
            # if clock_gettime() returns an error
            errno_ = ctypes.get_errno()
            raise OSError(errno_, os.strerror(errno_))
        return t.tv_sec + t.tv_nsec * 1e-9  # sec


    def micros():
        "return a timestamp in microseconds (us)"
        return monotonic_time() * 1e6  # us


    def millis():
        "return a timestamp in milliseconds (ms)"
        return monotonic_time() * 1e3  # ms


# Other timing functions:
def delay(delay_ms):
    "delay for delay_ms milliseconds (ms)"
    t_start = millis()

    while (millis() - t_start < delay_ms):
        pass  # do nothing
    return


def delayMicroseconds(delay_us):
    "delay for delay_us microseconds (us)"
    t_start = micros()

    while (micros() - t_start < delay_us):
        pass  # do nothing
    return
