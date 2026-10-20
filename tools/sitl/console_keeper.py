#!/usr/bin/env python3
"""Hold SITL SERIAL0 TCP console open so Rover can continue booting.

This script is not an autonomous-navigation test. It sends no commands.
"""
import socket
import sys
import time

port = int(sys.argv[1]) if len(sys.argv) > 1 else 5760
deadline = time.monotonic() + 30
while time.monotonic() < deadline:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=1) as sock:
            print("SITL console TCP connected", flush=True)
            sock.settimeout(1)
            ending = time.monotonic() + 60
            while time.monotonic() < ending:
                try:
                    block = sock.recv(4096)
                    if not block:
                        break
                except socket.timeout:
                    continue
            sys.exit(0)
    except (ConnectionRefusedError, TimeoutError, OSError):
        time.sleep(0.25)
print("SITL SERIAL0 not reachable", file=sys.stderr)
sys.exit(2)
