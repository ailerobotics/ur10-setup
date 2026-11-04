"""UR10 forward kinematics from the cell's *measured* (calibrated) URDF.

Pure numpy, no ROS. The chain base_link -> tool0 is read straight from
``cumotion/ur10_measured.urdf`` so the kinematics match the calibrated real
arm, not the nominal UR10 DH table. Validated offline against the teach-002
telemetry: FK(pre_grip joints) @ fork_TCP reproduces the recorded base_link
fork pose to < 0.1 mm (see fork_demo/align_check/README.md "FK validation").

The only thing align_check needs FK for is the *table* side: a tag seen by the
wrist at the pre-place pause must be carried base_link <- tool0 <- optical. The
*shelf* side needs no FK at all -- the tines and the wrist camera are both
rigidly bolted to the flange, so the tine image positions are a constant of
hand-eye + intrinsics, independent of arm pose.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np

# Joint order as published on /joint_states / stored in teach logs.
JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

# Fixed + revolute joints, in order, that compose base_link -> tool0 in the
# UR10 measured URDF. The revolute ones additionally rotate about their axis.
_CHAIN = (
    "base_link-base_link_inertia",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "wrist_3-flange",
    "flange-tool0",
)

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_URDF = os.path.join(_REPO, "cumotion", "ur10_measured.urdf")


def rpy_to_matrix(rpy) -> np.ndarray:
    """ZYX intrinsic (URDF rpy) -> 3x3 rotation, R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def make_transform(xyz, rpy) -> np.ndarray:
    m = np.eye(4)
    m[:3, :3] = rpy_to_matrix(rpy)
    m[:3, 3] = xyz
    return m


def _rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    m = np.eye(4)
    m[0, 0] = c
    m[0, 1] = -s
    m[1, 0] = s
    m[1, 1] = c
    return m


@dataclass
class UR10Kinematics:
    """base_link -> tool0 forward kinematics from a URDF's fixed joint origins."""

    origins: dict  # joint name -> (xyz, rpy)

    @classmethod
    def from_urdf(cls, path: str = DEFAULT_URDF) -> "UR10Kinematics":
        root = ET.parse(path).getroot()
        origins = {}
        for joint in root.findall("joint"):
            if joint.find("parent") is None or joint.find("child") is None:
                continue
            o = joint.find("origin")
            xyz = [float(v) for v in (o.get("xyz") if o is not None and o.get("xyz") else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy") if o is not None and o.get("rpy") else "0 0 0").split()]
            origins[joint.get("name")] = (xyz, rpy)
        missing = [j for j in _CHAIN if j not in origins]
        if missing:
            raise ValueError(f"URDF {path} missing chain joints: {missing}")
        return cls(origins=origins)

    def fk_tool0(self, joints) -> np.ndarray:
        """4x4 base_link <- tool0 for the six joint angles (JOINT_ORDER, rad)."""
        q = dict(zip(JOINT_ORDER, joints))
        m = np.eye(4)
        for name in _CHAIN:
            xyz, rpy = self.origins[name]
            m = m @ make_transform(xyz, rpy)
            if name in q:
                m = m @ _rot_z(q[name])
        return m
