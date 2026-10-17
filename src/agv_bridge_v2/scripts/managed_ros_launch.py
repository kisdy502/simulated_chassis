#!/usr/bin/env python3
"""Keep a nested launch tied to the bridge process, independently of its threads."""
import ctypes
import os
import select
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    # pidfd refers to the process lifetime, unlike PR_SET_PDEATHSIG, which also
    # fires when the particular C++ thread that called fork exits normally.
    try:
        parent = os.pidfd_open(int(sys.argv[1]))
    except ProcessLookupError:
        return 0
    ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0)  # PR_SET_CHILD_SUBREAPER
    stopping = None

    def interrupted(signum, frame):
        nonlocal stopping
        if stopping is None:
            stopping = time.monotonic()

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    process = subprocess.Popen(sys.argv[2:])  # inherit this owned process group
    stage = 0
    try:
        while True:
            dead, _, _ = select.select([parent], [], [], 0.1)
            if dead and stopping is None:
                os.killpg(os.getpgrp(), signal.SIGINT)
            if dead:
                time.sleep(0.05)  # a readable pidfd stays readable after death
            status = process.poll()
            if status is not None:
                # Adopt and reap grandchildren when ros2 launch exits first.
                try:
                    while os.waitpid(-1, os.WNOHANG)[0] > 0:
                        pass
                except ChildProcessError:
                    return 0 if stopping is not None else status
                if not Path(f'/proc/self/task/{os.getpid()}/children').read_text().strip():
                    return 0 if stopping is not None else status
                if stopping is None:
                    os.killpg(os.getpgrp(), signal.SIGINT)
            if stopping is not None:
                elapsed = time.monotonic() - stopping
                if elapsed >= 5 and stage == 0:
                    os.killpg(os.getpgrp(), signal.SIGTERM)
                    stage = 1
                if elapsed >= 7:
                    os.killpg(os.getpgrp(), signal.SIGKILL)
    finally:
        os.close(parent)


if __name__ == '__main__':
    sys.exit(main())
