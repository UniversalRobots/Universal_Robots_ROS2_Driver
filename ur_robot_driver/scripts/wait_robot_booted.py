#!/usr/bin/env python3
# Copyright 2026 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Wait until a UR controller/URSim instance is reachable after boot."""

import argparse
import socket
import sys
import time
import urllib.error
import urllib.request

DASHBOARD_PORT = 29999
ROBOT_API_OPENAPI_PATH = "/universal-robots/robot-api/openapi.json"


def dashboard_reachable(robot_ip: str, timeout: float) -> bool:
    """Return True if the dashboard server accepts a TCP connection."""
    try:
        with socket.create_connection((robot_ip, DASHBOARD_PORT), timeout=timeout):
            return True
    except OSError:
        return False


def robot_api_reachable(robot_ip: str, timeout: float) -> bool:
    """Return True if the PolyScope X Robot API openapi endpoint responds with HTTP 200."""
    url = f"http://{robot_ip}{ROBOT_API_OPENAPI_PATH}"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            return response.status == 200
    except (urllib.error.URLError, TimeoutError, ValueError):
        return False


def probe_robot(robot_ip: str, probe_timeout: float) -> str | None:
    """Probe dashboard and Robot API; return a short description of the first hit."""
    if dashboard_reachable(robot_ip, probe_timeout):
        return f"dashboard server at {robot_ip}:{DASHBOARD_PORT}"
    if robot_api_reachable(robot_ip, probe_timeout):
        return f"Robot API at http://{robot_ip}{ROBOT_API_OPENAPI_PATH}"
    return None


def wait_for_robot(
    robot_ip: str,
    timeout: float,
    interval: float,
    probe_timeout: float,
) -> bool:
    """Poll until the dashboard or Robot API is reachable, or ``timeout`` elapses."""
    deadline = time.monotonic() + timeout
    print(
        f"Waiting for robot at {robot_ip} "
        f"(dashboard :{DASHBOARD_PORT} or Robot API {ROBOT_API_OPENAPI_PATH})...",
        flush=True,
    )
    while time.monotonic() < deadline:
        found = probe_robot(robot_ip, probe_timeout)
        if found is not None:
            print(f"Robot is reachable via {found}.", flush=True)
            return True
        remaining = max(0.0, deadline - time.monotonic())
        print(
            f"Robot not ready yet; retrying in {interval:.1f}s ({remaining:.1f}s left)...",
            flush=True,
        )
        time.sleep(min(interval, remaining) if remaining > 0 else interval)
        if time.monotonic() >= deadline:
            break
    print(
        f"Timed out after {timeout:.1f}s waiting for robot at {robot_ip}.",
        file=sys.stderr,
        flush=True,
    )
    return False


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "robot_ip",
        help="IP address of the robot / URSim instance",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="Overall time to wait in seconds (default: 120)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=3.0,
        help="Delay between connection attempts in seconds (default: 3)",
    )
    parser.add_argument(
        "--probe-timeout",
        type=float,
        default=2.0,
        help="Timeout for a single connection attempt in seconds (default: 2)",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.timeout <= 0 or args.interval <= 0 or args.probe_timeout <= 0:
        print("timeout, interval and probe-timeout must be positive", file=sys.stderr)
        return 2

    ok = wait_for_robot(
        args.robot_ip,
        args.timeout,
        args.interval,
        args.probe_timeout,
    )
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
