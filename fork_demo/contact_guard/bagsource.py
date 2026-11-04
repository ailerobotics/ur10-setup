"""Read a recorded teach/replay mcap bag + teach metadata into leg time series.

This is the only module that touches an mcap file; ``mcap`` is imported lazily
so the rest of the package stays importable without it. No rclpy.

A teach-002 leg is located in the continuous bag by its dwell window: the motion
for leg ``start_label -> end_label`` is the interval
``[dwell(start).t_end, dwell(end).t_start]``. Within that interval the bag's
``/joint_states`` clock is the master; wrench, TCP pose and speed are interpolated
onto it, the wrench is shifted to the fork tip, TCP speed is differenced, and
normalised progress is the arc-length projection of each joint config onto the
leg's taught ``joint_path`` (from the per-leg plan yaml).
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

from . import signals as sig
from .params import ContactGuardParams
from .signals import LegTimeSeries

TEACH_DIR_DEFAULT = (
    "/home/manishsaini/firstmate/data/ur10-fork-pick-real-v1/teach-002"
)

TOPICS = [
    "/joint_states",
    "/force_torque_sensor_broadcaster/wrench",
    "/tcp_pose_broadcaster/pose",
    "/speed_scaling_state_broadcaster/speed_scaling",
    "/io_and_status_controller/robot_program_running",
]


@dataclass
class LegDef:
    index: int
    name: str
    contact: bool
    start_label: str
    end_label: str
    joint_path: np.ndarray          # (M, 6) in JOINT_ORDER
    goal: np.ndarray                # (6,)


@dataclass
class BagStreams:
    """Raw decoded streams, each a (t, data) pair with absolute-epoch seconds."""
    js_t: np.ndarray
    js_pos: np.ndarray              # (T, 6) in JOINT_ORDER
    js_vel: np.ndarray
    js_eff: np.ndarray
    w_t: np.ndarray
    w_force: np.ndarray             # (W, 3) tool0 frame
    w_torque: np.ndarray            # (W, 3)
    tcp_t: np.ndarray
    tcp_pos: np.ndarray             # (P, 3) base frame
    spd_t: np.ndarray
    spd: np.ndarray                 # percent (0..100)
    prog_t: np.ndarray
    prog: np.ndarray                # bool
    t0: float                       # bag start epoch

    def rel(self, t: np.ndarray) -> np.ndarray:
        return t - self.t0


# --------------------------------------------------------------------------- #
# metadata
# --------------------------------------------------------------------------- #
def load_leg_defs(run_dir: str = TEACH_DIR_DEFAULT) -> List[LegDef]:
    with open(os.path.join(run_dir, "replay_plan.json")) as fh:
        rp = json.load(fh)
    legs: List[LegDef] = []
    for s in rp["segments"]:
        plan_path = os.path.join(run_dir, s["plan_yaml"])
        with open(plan_path) as fh:
            plan = yaml.safe_load(fh)
        jp = np.asarray(plan["joint_path"], dtype=float)
        goal = np.asarray(plan.get("goal_joints", jp[-1]), dtype=float)
        legs.append(LegDef(
            index=int(s["index"]), name=str(s["name"]),
            contact=bool(s["contact"]), start_label=str(s["start"]),
            end_label=str(s["end"]), joint_path=jp, goal=goal,
        ))
    return legs


def load_dwell_windows(run_dir: str = TEACH_DIR_DEFAULT
                       ) -> Dict[str, Tuple[float, float]]:
    """Map a waypoint label -> (dwell.t_start, dwell.t_end) absolute epoch."""
    with open(os.path.join(run_dir, "teach_dwells.json")) as fh:
        dwells = json.load(fh)["dwells"]
    with open(os.path.join(run_dir, "labelled_waypoints.json")) as fh:
        wps = json.load(fh)["waypoints"]
    out: Dict[str, Tuple[float, float]] = {}
    for w in wps:
        di = w.get("dwell")
        if di is None or di >= len(dwells):
            continue
        d = dwells[di]
        out[w["label"]] = (float(d["t_start"]), float(d["t_end"]))
    return out


def leg_interval(leg: LegDef, windows: Dict[str, Tuple[float, float]],
                 ordered_labels: Sequence[str]) -> Optional[Tuple[float, float]]:
    """Motion interval [t0, t1] for a leg from dwell windows.

    Start = end of the start-label dwell; end = start of the end-label dwell.
    A synthetic start label (e.g. ``__carry_via__``) falls back to the previous
    labelled waypoint's dwell end.
    """
    end = windows.get(leg.end_label)
    if end is None:
        return None
    start = windows.get(leg.start_label)
    if start is None:
        # fall back to the label just before end_label in teach order
        try:
            ei = ordered_labels.index(leg.end_label)
        except ValueError:
            return None
        start = None
        for j in range(ei - 1, -1, -1):
            if ordered_labels[j] in windows:
                start = windows[ordered_labels[j]]
                break
        if start is None:
            return None
    t0, t1 = start[1], end[0]
    if t1 <= t0:
        return None
    return (t0, t1)


# --------------------------------------------------------------------------- #
# bag decode (lazy mcap)
# --------------------------------------------------------------------------- #
def read_bag(bag_path: str, topics: Sequence[str] = TOPICS) -> BagStreams:
    from mcap_ros2.reader import read_ros2_messages  # lazy

    js_t, js_names = [], None
    js_pos, js_vel, js_eff = [], [], []
    w_t, w_f, w_m = [], [], []
    tcp_t, tcp_p = [], []
    spd_t, spd = [], []
    prog_t, prog = [], []
    t0 = None
    for m in read_ros2_messages(bag_path, topics=list(topics)):
        t = m.log_time_ns / 1e9
        if t0 is None:
            t0 = t
        topic = m.channel.topic
        msg = m.ros_msg
        if topic == "/joint_states":
            if js_names is None:
                js_names = list(msg.name)
            js_t.append(t)
            js_pos.append(sig.reorder_joints(msg.name, msg.position))
            js_vel.append(sig.reorder_joints(msg.name, msg.velocity)
                          if len(msg.velocity) == len(msg.name)
                          else np.zeros(6))
            js_eff.append(sig.reorder_joints(msg.name, msg.effort)
                          if len(msg.effort) == len(msg.name)
                          else np.zeros(6))
        elif topic.endswith("/wrench"):
            w = msg.wrench
            w_t.append(t)
            w_f.append([w.force.x, w.force.y, w.force.z])
            w_m.append([w.torque.x, w.torque.y, w.torque.z])
        elif topic.endswith("/pose"):
            p = msg.pose.position
            tcp_t.append(t)
            tcp_p.append([p.x, p.y, p.z])
        elif topic.endswith("/speed_scaling"):
            spd_t.append(t)
            spd.append(float(msg.data))
        elif topic.endswith("/robot_program_running"):
            prog_t.append(t)
            prog.append(bool(msg.data))
    arr = lambda x, w=1: (np.asarray(x, dtype=float).reshape(-1, w) if w > 1
                          else np.asarray(x, dtype=float))
    return BagStreams(
        js_t=arr(js_t), js_pos=(np.asarray(js_pos) if js_pos else np.zeros((0, 6))),
        js_vel=(np.asarray(js_vel) if js_vel else np.zeros((0, 6))),
        js_eff=(np.asarray(js_eff) if js_eff else np.zeros((0, 6))),
        w_t=arr(w_t), w_force=(np.asarray(w_f) if w_f else np.zeros((0, 3))),
        w_torque=(np.asarray(w_m) if w_m else np.zeros((0, 3))),
        tcp_t=arr(tcp_t), tcp_pos=(np.asarray(tcp_p) if tcp_p else np.zeros((0, 3))),
        spd_t=arr(spd_t), spd=arr(spd),
        prog_t=arr(prog_t), prog=np.asarray(prog, dtype=bool),
        t0=float(t0 if t0 is not None else 0.0),
    )


def _interp_rows(t_query: np.ndarray, t_src: np.ndarray, data: np.ndarray
                 ) -> np.ndarray:
    if t_src.size == 0:
        return np.zeros((t_query.size, data.shape[1] if data.ndim == 2 else 1))
    if data.ndim == 1:
        return np.interp(t_query, t_src, data)
    cols = [np.interp(t_query, t_src, data[:, c]) for c in range(data.shape[1])]
    return np.stack(cols, axis=1)


def leg_timeseries(leg: LegDef, streams: BagStreams, interval: Tuple[float, float]
                   ) -> LegTimeSeries:
    """Build a resampled, fork-frame :class:`LegTimeSeries` for one leg."""
    t0, t1 = interval
    sel = (streams.js_t >= t0) & (streams.js_t <= t1)
    jt = streams.js_t[sel]
    jp = streams.js_pos[sel]
    je = streams.js_eff[sel]
    if jt.size == 0:
        return LegTimeSeries(leg=leg.name, t=np.zeros(0), progress=np.zeros(0))
    # interpolate wrench + tcp onto the joint clock
    wf = _interp_rows(jt, streams.w_t, streams.w_force)
    wm = _interp_rows(jt, streams.w_t, streams.w_torque)
    tcp = _interp_rows(jt, streams.tcp_t, streams.tcp_pos)
    # TCP speed (central difference), m/s
    tcp_vel = np.zeros(jt.size)
    if jt.size >= 2:
        dt = np.gradient(jt)
        dt[dt == 0] = 1e-6
        d = np.gradient(tcp, axis=0)
        tcp_vel = np.linalg.norm(d, axis=1) / dt
    # progress via arc-length projection on the taught joint_path
    cum = sig.path_arclengths(leg.joint_path)
    progress = np.array([sig.progress_of(q, leg.joint_path, cum) for q in jp])
    # per-sample fork-frame features
    feats: Dict[str, list] = {s: [] for s in sig.BAG_SIGNALS}
    for i in range(jt.size):
        ff, mf = sig.wrench_to_fork(wf[i], wm[i])
        f = sig.sample_features(ff, mf, je[i], float(tcp_vel[i]))
        for s in sig.BAG_SIGNALS:
            feats[s].append(f[s])
    ts = LegTimeSeries(leg=leg.name, t=jt - jt[0], progress=progress)
    ts.feats = {s: np.asarray(v, dtype=float) for s, v in feats.items()}
    # stash the TCP path length for mm-at-trip conversion
    ts.feats["_tcp_travel_m"] = np.asarray(
        [float(np.sum(np.linalg.norm(np.diff(tcp, axis=0), axis=1)))
         if tcp.shape[0] >= 2 else 0.0])
    return ts


def ordered_labels(run_dir: str = TEACH_DIR_DEFAULT) -> List[str]:
    with open(os.path.join(run_dir, "labelled_waypoints.json")) as fh:
        wps = json.load(fh)["waypoints"]
    return [w["label"] for w in wps]
