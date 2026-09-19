# SPDX-License-Identifier: Apache-2.0
"""Exact forward kinematics for the cell's CALIBRATED UR10, in base_link.

Pure numpy + the URDF's own joint origins (no DH assumptions, no base_link/base
offset guesswork): it chains the exact <origin> transforms of ur10_calibrated.urdf
so the TCP it returns is in the SAME base_link frame nvblox maps and cuMotion
plans in. Used by the route proof to (a) choose a start/goal sweep the fixed synth
camera can see and (b) place a synthetic obstacle exactly on the planned TCP path.

FK only — this never commands anything.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from typing import Dict, List, Sequence

import numpy as np

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def _rpy(r, p, y) -> np.ndarray:
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _origin(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _rot_axis(axis, q) -> np.ndarray:
    a = np.asarray(axis, dtype=float)
    a /= np.linalg.norm(a)
    x, y, z = a
    c, s, C = np.cos(q), np.sin(q), 1 - np.cos(q)
    R = np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C]])
    T = np.eye(4)
    T[:3, :3] = R
    return T


class UR10FK:
    """Parses the URDF once; FK for tool0 (and any link) from a 6-joint config."""

    def __init__(self, urdf_path: str, base: str = "base_link", tip: str = "tool0"):
        root = ET.parse(urdf_path).getroot()
        self.joints = {}
        self.children = {}          # parent_link -> list of joint names
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0").split())] if o is not None else [0, 0, 0]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0").split())] if o is not None else [0, 0, 0]
            ax = j.find("axis")
            axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1]
            rec = {"name": j.get("name"), "type": j.get("type"),
                   "parent": j.find("parent").get("link"),
                   "child": j.find("child").get("link"),
                   "xyz": xyz, "rpy": rpy, "axis": axis}
            self.joints[rec["name"]] = rec
            self.children.setdefault(rec["parent"], []).append(rec["name"])
        self.base = base
        self.tip = tip
        self._chain = self._find_chain(base, tip)

    def _find_chain(self, base: str, tip: str) -> List[str]:
        # BFS from base to tip, returning the ordered joint names.
        stack = [(base, [])]
        seen = set()
        while stack:
            link, path = stack.pop()
            if link == tip:
                return path
            if link in seen:
                continue
            seen.add(link)
            for jn in self.children.get(link, []):
                stack.append((self.joints[jn]["child"], path + [jn]))
        raise ValueError(f"no chain {base} -> {tip}")

    def fk(self, q: Sequence[float], tip: str = None) -> np.ndarray:
        """4x4 base->tip for joint config q (order = JOINTS)."""
        qd = dict(zip(JOINTS, q))
        chain = self._chain if (tip is None or tip == self.tip) else self._find_chain(self.base, tip)
        T = np.eye(4)
        for jn in chain:
            j = self.joints[jn]
            T = T @ _origin(j["xyz"], j["rpy"])
            if j["type"] in ("revolute", "continuous"):
                T = T @ _rot_axis(j["axis"], qd[jn])
        return T

    def tcp(self, q: Sequence[float]) -> np.ndarray:
        """TCP (tool0) position in base_link."""
        return self.fk(q)[:3, 3]

    def tcp_path(self, qs: Sequence[Sequence[float]]) -> np.ndarray:
        return np.array([self.tcp(q) for q in qs])
