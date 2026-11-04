import numpy as np

from fork_demo.contact_guard import signals as sig
from fork_demo.contact_guard.params import FORK_OFFSET


def test_reorder_joints():
    names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    vals = [3.0, 2.0, 1.0, 4.0, 5.0, 6.0]
    out = sig.reorder_joints(names, vals)
    # canonical order is pan, lift, elbow, w1, w2, w3
    assert list(out) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def test_wrench_to_fork_shifts_torque_not_force():
    f = [10.0, 0.0, 0.0]
    m = [0.0, 0.0, 0.0]
    ff, mf = sig.wrench_to_fork(f, m, FORK_OFFSET)
    assert np.allclose(ff, f)                       # force unchanged
    # M_fork = -offset x F ; offset=(0,-0.038,0.427), F=(10,0,0)
    expect = -np.cross(FORK_OFFSET, f)
    assert np.allclose(mf, expect)
    assert not np.allclose(mf, m)                   # torque DID shift


def test_progress_monotonic_along_path():
    path = np.array([[0, 0, 0, 0, 0, 0],
                     [1, 0, 0, 0, 0, 0],
                     [2, 0, 0, 0, 0, 0]], dtype=float)
    cum = sig.path_arclengths(path)
    assert sig.progress_of(path[0], path, cum) == 0.0
    assert abs(sig.progress_of(path[1], path, cum) - 0.5) < 1e-6
    assert abs(sig.progress_of(path[-1], path, cum) - 1.0) < 1e-6


def test_sample_features_components_present():
    f = sig.sample_features([3.0, 4.0, 5.0], [1.0, 2.0, 0.5], [0, 0, 0, 0, 0, 0],
                            0.1, track_err=0.02)
    assert abs(f["f_lat"] - 5.0) < 1e-9             # hypot(3,4)
    assert f["f_ins"] == 5.0
    assert f["t_pry"] == 2.0
    for k in ("fx", "fy", "fz", "mx", "my", "mz"):
        assert k in f
    assert f["track_err"] == 0.02
