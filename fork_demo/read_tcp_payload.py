#!/usr/bin/env python3
"""Read the CONFIGURED TCP offset (and, with --rtde, the payload) from the real
UR10 CB3 controller. READ-ONLY: it opens the controller's broadcast interfaces,
sends NO motion/config command, and exits.

Why this tool: before any fork-pick execution we must confirm a *fork* TCP is set
on the robot (forks reach ~0.427 m along tool Z, 38 mm off tool Y). The default
UR flange TCP is all-zero; a fork TCP is far from zero. This prints the offset so
the captain's "fork TCP is set" claim can be verified from the PC.

Sources:
  * TCP OFFSET + robot/safety mode: the PRIMARY interface (port 30011, 10 Hz
    broadcast). We only READ the ROBOT_STATE stream and parse the CARTESIAN_INFO
    sub-package (which carries the 6-DOF TCP offset on CB3 PolyScope 3.x) and the
    ROBOT_MODE_DATA sub-package. Many clients (incl. the ROS driver) read this
    stream concurrently; a read does not disturb them.
  * PAYLOAD: only with the explicit --rtde flag, via a short RTDE output
    subscription on :30004 (the same port the driver uses; RTDE permits multiple
    output recipes, but we keep this opt-in so the default never touches it).

NB: while the arm is POWER_OFF / in an emergency stop the controller may report a
stale or zeroed offset; re-read when RUNNING / NORMAL for a trustworthy value.
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

PRIMARY_PORT = 30011   # primary interface, 10 Hz (CB3)
RTDE_PORT = 30004

# UR primary-interface ROBOT_STATE sub-package types
PKG_ROBOT_MODE = 0
PKG_CARTESIAN_INFO = 4

ROBOT_MODE_NAMES = {
    0: "DISCONNECTED", 1: "CONFIRM_SAFETY", 2: "BOOTING", 3: "POWER_OFF",
    4: "POWER_ON", 5: "IDLE", 6: "BACKDRIVE", 7: "RUNNING", 8: "UPDATING_FIRMWARE",
}


def _read_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("primary interface closed mid-message")
        buf += chunk
    return buf


def read_primary(robot_ip: str, timeout_s: float = 8.0) -> dict:
    """Read one ROBOT_STATE and return parsed TCP offset + robot mode.

    Returns {} if nothing parseable arrived in time.
    """
    out: dict = {}
    deadline = time.time() + timeout_s
    with socket.create_connection((robot_ip, PRIMARY_PORT), timeout=timeout_s) as s:
        s.settimeout(timeout_s)
        while time.time() < deadline and not ("tcp_offset" in out and "robot_mode" in out):
            header = _read_exact(s, 5)
            msg_len = struct.unpack(">i", header[:4])[0]
            msg_type = header[4]
            body = _read_exact(s, msg_len - 5)
            if msg_type != 16:  # not ROBOT_STATE
                continue
            # Walk sub-packages.
            off = 0
            while off + 5 <= len(body):
                sub_len = struct.unpack(">i", body[off:off + 4])[0]
                sub_type = body[off + 4]
                sub = body[off + 5:off + sub_len]
                if sub_type == PKG_CARTESIAN_INFO:
                    # CB3: 6 doubles TCP-in-base, then 6 doubles TCP OFFSET.
                    if len(sub) >= 12 * 8:
                        vals = struct.unpack(">12d", sub[:12 * 8])
                        out["tcp_in_base"] = vals[0:6]
                        out["tcp_offset"] = vals[6:12]
                    elif len(sub) >= 6 * 8:
                        out["tcp_in_base"] = struct.unpack(">6d", sub[:6 * 8])
                elif sub_type == PKG_ROBOT_MODE:
                    # First field after timestamp (uint64) is a set of bools; the
                    # robotMode byte sits at a fixed offset on CB3. Parse defensively.
                    try:
                        # layout: uint64 timestamp, 6 bools, then uint8 robotMode ...
                        robot_mode = sub[8 + 6]
                        out["robot_mode"] = ROBOT_MODE_NAMES.get(robot_mode, f"code{robot_mode}")
                    except IndexError:
                        pass
                if sub_len <= 0:
                    break
                off += sub_len
    return out


def read_payload_rtde(robot_ip: str, timeout_s: float = 6.0) -> dict:
    """Opt-in RTDE read of payload mass + CoG. Returns {} on any failure."""
    RTDE_REQUEST_PROTOCOL_VERSION = 86   # 'V'
    RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS = 79  # 'O'
    RTDE_CONTROL_PACKAGE_START = 83      # 'S'
    RTDE_DATA_PACKAGE = 85               # 'U'

    def send(sock, ptype, payload=b""):
        pkg = struct.pack(">HB", 3 + len(payload), ptype) + payload
        sock.sendall(pkg)

    def recv_pkg(sock):
        head = _read_exact(sock, 3)
        size, ptype = struct.unpack(">HB", head)
        return ptype, _read_exact(sock, size - 3)

    try:
        with socket.create_connection((robot_ip, RTDE_PORT), timeout=timeout_s) as s:
            s.settimeout(timeout_s)
            send(s, RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))
            recv_pkg(s)
            variables = b"payload,payload_cog"
            send(s, RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, struct.pack(">d", 10.0) + variables)
            ptype, body = recv_pkg(s)
            recipe_id = body[0]
            types = body[1:].decode(errors="replace")
            if "NOT_FOUND" in types:
                return {"error": f"RTDE types: {types}"}
            send(s, RTDE_CONTROL_PACKAGE_START, b"")
            recv_pkg(s)
            # Read a data package.
            for _ in range(20):
                ptype, body = recv_pkg(s)
                if ptype == RTDE_DATA_PACKAGE and body and body[0] == recipe_id:
                    data = body[1:]
                    payload_mass = struct.unpack(">d", data[0:8])[0]
                    cog = struct.unpack(">3d", data[8:32]) if len(data) >= 32 else None
                    return {"payload_kg": payload_mass, "payload_cog": cog}
            return {"error": "no RTDE data package"}
    except (OSError, struct.error, ConnectionError) as e:
        return {"error": f"{type(e).__name__}: {e}"}


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--robot-ip", default="192.168.0.1")
    p.add_argument("--rtde", action="store_true",
                   help="also read payload via a short RTDE subscription on :30004 (opt-in)")
    args = p.parse_args(argv)

    print(f"Reading configured TCP from UR primary interface {args.robot_ip}:{PRIMARY_PORT} ...")
    try:
        info = read_primary(args.robot_ip)
    except (OSError, ConnectionError, struct.error) as e:
        print(f"PRIMARY READ FAILED: {type(e).__name__}: {e}", file=sys.stderr)
        return 2

    mode = info.get("robot_mode", "UNKNOWN")
    print(f"robot_mode (primary): {mode}")
    off = info.get("tcp_offset")
    if off:
        x, y, z, rx, ry, rz = off
        print("TCP OFFSET (tool0 -> TCP), metres / rad (axis-angle):")
        print(f"  x={x:+.5f}  y={y:+.5f}  z={z:+.5f}   rx={rx:+.5f} ry={ry:+.5f} rz={rz:+.5f}")
        mag = (x * x + y * y + z * z) ** 0.5
        is_fork = z > 0.30 and abs(y) > 0.01
        print(f"  |translation| = {mag:.4f} m  -> looks like a FORK TCP: {is_fork}")
        print("  (fork kit expects ~ x=0.000 y=-0.038 z=0.427; all-zero = default flange TCP)")
    else:
        print("TCP OFFSET: not reported (no CARTESIAN_INFO parsed).")
    if "tcp_in_base" in info:
        print(f"TCP-in-base now: {tuple(round(v, 4) for v in info['tcp_in_base'])}")

    if args.rtde:
        print(f"\nReading payload via RTDE {args.robot_ip}:{RTDE_PORT} ...")
        pl = read_payload_rtde(args.robot_ip)
        if "error" in pl:
            print(f"PAYLOAD READ FAILED: {pl['error']}", file=sys.stderr)
        else:
            print(f"payload mass: {pl['payload_kg']:.3f} kg   cog: {pl['payload_cog']}")
    else:
        print("\n(payload not read; pass --rtde to read it via a short RTDE subscription)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
