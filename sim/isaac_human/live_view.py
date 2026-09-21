#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Pure helpers for the "Watch it live" WebRTC mode of the human-detection sim.

No Isaac / ROS imports, so this is importable in the offline test env (system
python3) and unit-tested there. It carries:

  * the fixed port facts of the stream (signalling 49100, the client web page),
  * the forbidden-port set the brief pins and a free-port picker,
  * the SLOW / STOP floor-ring radii read from the SAME safety config the
    detector uses (never hard-coded here) + their colours,
  * the live-zone beacon colours keyed off /human_safety/state, and
  * the argument parser the scene (``isaac_scene.py``) builds, so argument
    parsing is testable without importing Isaac.

The heavy lifting (drawing the rings, enabling the livestream extension,
subscribing to the state topic) lives in ``isaac_scene.py``; this module only
holds the parts that are pure and worth testing offline.
"""
from __future__ import annotations

import argparse
import socket
import sys
from dataclasses import dataclass
from typing import Callable, List, Tuple

# --- fixed ports ----------------------------------------------------------- #
# omni.kit.livestream.webrtc signalling (TCP). The owner's shelfbot tooling
# streams on exactly this port; the browser client defaults to it.
LIVESTREAM_PORT = 49100
# Preferred port for the static browser client (python3 -m http.server).
DEFAULT_CLIENT_PORT = 8080

# Ports the brief forbids the client server from ever binding (other apps /
# reserved dev ports on this workstation).
FORBIDDEN_PORTS = frozenset(
    set(range(4173, 4179))     # 4173-4178
    | set(range(8443, 8450))   # 8443-8449
    | {8765, 54321}
)


def is_forbidden_port(port: int) -> bool:
    return port in FORBIDDEN_PORTS


def port_is_free(port: int, host: str) -> bool:
    """True if nothing is bound to (host, port) right now. A real bind test —
    deliberately WITHOUT SO_REUSEADDR, so a live listener reads as busy."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind((host, port))
            return True
        except OSError:
            return False


def pick_client_port(preferred: int, is_free: Callable[[int], bool],
                     forbidden=FORBIDDEN_PORTS, max_scan: int = 200) -> int:
    """First free, non-forbidden TCP port at or above ``preferred``.

    ``is_free(port) -> bool`` is injected so the search is unit-testable without
    binding sockets. Scans upward past forbidden/busy ports; raises if none of
    the first ``max_scan`` candidates is usable.
    """
    port = preferred
    for _ in range(max_scan):
        if 1 <= port <= 65535 and port not in forbidden and is_free(port):
            return port
        port += 1
    raise RuntimeError(
        f"no free, non-forbidden port found in "
        f"[{preferred}, {preferred + max_scan})")


# --- zone rings (read from the safety config, never hard-coded) ------------ #
@dataclass(frozen=True)
class RingSpec:
    """One floor ring: name, world radius (m) about base_link, RGBA in 0..1."""
    name: str
    radius: float
    rgba: Tuple[float, float, float, float]


def zone_ring_specs(reach_radius: float, stop_radius: float, slow_radius: float,
                    alpha: float = 0.7) -> List[RingSpec]:
    """The SLOW and STOP rings the detector uses, as concentric floor circles.

    Radii and colours match how the production fusion node draws them
    (cams/human_safety/nodes/fusion_node.py): the ring radius is the ENVELOPE
    boundary — ``reach_radius`` (the arm's cylinder) plus the zone's clearance —
    so STOP is reach+stop (2.0 m here) and SLOW is reach+slow (3.3 m here).
    SLOW is amber, STOP is red.
    """
    return [
        RingSpec("SLOW", reach_radius + slow_radius, (1.0, 0.8, 0.0, alpha)),
        RingSpec("STOP", reach_radius + stop_radius, (1.0, 0.1, 0.1, alpha)),
    ]


def read_zone_radii(config_path: str) -> Tuple[float, float, float]:
    """(reach_radius, stop_radius, slow_radius) from a human_safety YAML config.

    Reads the SAME ``safety:`` block the detector loads, so the drawn rings can
    never drift from the logic. Raises if the keys are missing.
    """
    import yaml  # local import: keeps the module import pure for the arg tests
    with open(config_path) as f:
        raw = yaml.safe_load(f) or {}
    s = raw.get("safety") or {}
    return (float(s["reach_radius"]), float(s["stop_radius"]),
            float(s["slow_radius"]))


# --- live-zone beacon colours (keyed off /human_safety/state "zone") ------- #
ZONE_COLORS = {
    "CLEAR": (0.10, 0.80, 0.20),   # green
    "SLOW":  (1.00, 0.70, 0.00),   # amber
    "STOP":  (0.90, 0.10, 0.10),   # red
}
# No state yet / unknown zone: neutral grey (fail-visible, not green).
UNKNOWN_ZONE_COLOR = (0.45, 0.45, 0.45)


def zone_color(zone) -> Tuple[float, float, float]:
    return ZONE_COLORS.get(zone, UNKNOWN_ZONE_COLOR)


# --- the scene argument parser (shared so parsing is testable) ------------- #
def build_scene_arg_parser() -> argparse.ArgumentParser:
    """The parser ``isaac_scene.py`` uses. Kept here so argument parsing can be
    tested without importing Isaac."""
    ap = argparse.ArgumentParser()
    ap.add_argument("--cams", default="wrist,shelf,table")
    ap.add_argument("--gt-log", required=True, help="ground-truth JSONL output")
    ap.add_argument("--max-seconds", type=float, default=180.0,
                    help="hard wall-clock cap for the whole run")
    ap.add_argument("--settle-seconds", type=float, default=4.0,
                    help="render warm-up before the person starts moving")
    ap.add_argument("--sim-hz", type=float, default=5.0,
                    help="cap Isaac's update/publish rate (Hz)")
    # -- live-view additions ------------------------------------------------ #
    ap.add_argument("--livestream", action="store_true",
                    help="enable the omni.kit.livestream.webrtc viewport stream")
    ap.add_argument("--public-ip", default=None,
                    help="publicEndpointAddress for WebRTC ICE (the tailnet IP)")
    ap.add_argument("--livestream-port", type=int, default=LIVESTREAM_PORT,
                    help="livestream signalling port (default 49100)")
    ap.add_argument("--loop-walk", action="store_true",
                    help="repeat the person's walk until --max-seconds so there "
                         "is always something to watch")
    ap.add_argument("--config", default=None,
                    help="human_safety sim config; the SLOW/STOP ring radii are "
                         "read from its safety block (never hard-coded)")
    ap.add_argument("--zone-beacon", action="store_true",
                    help="best-effort: colour a beacon at the robot green/amber/"
                         "red from /human_safety/state")
    # -- record additions (offline video, NO WebRTC) ----------------------- #
    ap.add_argument("--record", action="store_true",
                    help="capture the fixed OBSERVER camera (cell + rings + "
                         "beacon) to JPEG frames for the offline video composer; "
                         "draws the rings + beacon like the live view but with NO "
                         "WebRTC stream (streaming only adds GPU load)")
    ap.add_argument("--observer-frames-dir", default=None,
                    help="directory for the recorded observer JPEG frames")
    ap.add_argument("--observer-index", default=None,
                    help="JSONL index (one {path,t} per saved observer frame)")
    ap.add_argument("--observer-size", default="1280x720",
                    help="observer capture size WxH (even dims; default 1280x720)")
    ap.add_argument("--observer-every", type=int, default=1,
                    help="capture every Nth sim step (throttle observer render)")
    return ap


# --- tiny CLI the bash runner calls for port selection --------------------- #
def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="live-view helpers")
    sub = ap.add_subparsers(dest="cmd", required=True)
    pp = sub.add_parser("pick-port", help="print a free, non-forbidden port")
    pp.add_argument("--preferred", type=int, default=DEFAULT_CLIENT_PORT)
    pp.add_argument("--bind", required=True, help="host/IP to test the bind on")
    args = ap.parse_args(argv)
    if args.cmd == "pick-port":
        port = pick_client_port(args.preferred,
                                lambda p: port_is_free(p, args.bind))
        print(port)
        return 0
    return 2


if __name__ == "__main__":
    sys.exit(_main())
