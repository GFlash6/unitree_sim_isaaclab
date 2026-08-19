#!/usr/bin/env python3
"""Create one signal-safe process group, then replace this process with a component."""

from __future__ import annotations

import os
import signal
import sys


if len(sys.argv) < 2:
    raise SystemExit("component command is required")

os.setsid()
for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(signum, signal.SIG_DFL)
os.execv(sys.argv[1], sys.argv[1:])
