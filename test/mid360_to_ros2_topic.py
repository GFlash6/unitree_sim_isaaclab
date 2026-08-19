#!/usr/bin/env python3
"""Compatibility entry point; the production bridge lives in holoagent_bridge."""

import sys

from holoagent_bridge.mid360_to_ros2_topic import *  # noqa: F401,F403


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
