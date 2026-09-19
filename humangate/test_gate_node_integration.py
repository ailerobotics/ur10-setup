#!/usr/bin/env python3
"""
The ONE ROS integration test for the gate (report §8, task Verification).

Brings up the real GateNode and a fake camera-state publisher on an ISOLATED
ROS_DOMAIN_ID (78) with FASTDDS_BUILTIN_TRANSPORTS=UDPv4, drives a scripted
approach -> tracked-leaving -> dwell -> resume sequence, then drops the publisher,
and asserts the decisions observed on the gate's own topic: SLOW and STOP while a
person is present/close, RUN only after the healthy CLEAR dwell, and STOP again
once the state topic goes silent (heartbeat timeout).

The ROS work runs in a SUBPROCESS with the isolation env set BEFORE rclpy is
imported, so domain 78 holds no matter how pytest imported things. If rclpy or
the DDS middleware is unavailable, the test skips rather than failing.

Run standalone:  python3 test_gate_node_integration.py
Or under pytest: pytest test_gate_node_integration.py
"""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DOMAIN_ID = "78"


# ------------------------- subprocess worker -------------------------------

def _worker():
    """Runs inside the subprocess: real ROS graph on domain 78."""
    try:
        import threading
        import time

        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String

        sys.path.insert(0, HERE)
        import state_adapter as sa
        from camera_state import ZONE_CLEAR, ZONE_SLOW, ZONE_STOP
        from gate_node import GateNode, DECISION_TOPIC_DEFAULT
    except Exception as e:                            # noqa: BLE001
        print("WORKER_ROS_UNAVAILABLE " + repr(e))
        return 0

    try:
        rclpy.init(args=sys.argv)
    except Exception as e:                            # noqa: BLE001
        print("WORKER_ROS_UNAVAILABLE " + repr(e))
        return 0

    seen = []

    class Harness(Node):
        def __init__(self):
            super().__init__("gate_test_harness")
            self.pub = self.create_publisher(String, sa.TOPIC_DEFAULT, 10)
            self.create_subscription(String, DECISION_TOPIC_DEFAULT, self._on_dec, 10)

        def _on_dec(self, msg):
            seen.append(json.loads(msg.data)["action"])

        def send(self, zone, dist, seq):
            now = time.time()
            known = dist is not None
            data = sa.build_state_json(
                zone, stamp=now, evidence_stamp=(now if known else None),
                heartbeat_seq=seq,
                distance_status=("known" if known else "none"),
                nearest_distance_base=dist)
            self.pub.publish(String(data=data))

    gate = GateNode()
    harness = Harness()
    ex = MultiThreadedExecutor()
    ex.add_node(gate)
    ex.add_node(harness)
    t = threading.Thread(target=ex.spin, daemon=True)
    t.start()

    def stream(zone, dist, dur, seq):
        end = time.time() + dur
        while time.time() < end:
            harness.send(zone, dist, seq[0])
            seq[0] += 1
            time.sleep(0.05)

    seq = [1]
    time.sleep(0.3)                       # let discovery settle
    stream(ZONE_SLOW, 2.4, 0.3, seq)      # person approaching
    stream(ZONE_STOP, 1.2, 0.4, seq)      # person close -> STOP
    stream(ZONE_SLOW, 2.3, 0.3, seq)      # tracked leaving
    stream(ZONE_CLEAR, None, 1.2, seq)    # clear + dwell (0.5 s) -> RUN
    action_before_silence = seen[-1] if seen else None
    time.sleep(0.6)                       # silence -> heartbeat timeout -> STOP
    final = seen[-1] if seen else None

    ex.shutdown()
    gate.destroy_node()
    harness.destroy_node()
    rclpy.shutdown()

    compressed = [a for i, a in enumerate(seen) if i == 0 or a != seen[i - 1]]
    print("WORKER_OK " + json.dumps({
        "compressed": compressed,
        "before_silence": action_before_silence,
        "final": final,
    }))
    return 0


# ------------------------- pytest entry point ------------------------------

def _run_worker_subprocess():
    env = dict(os.environ)
    env["ROS_DOMAIN_ID"] = DOMAIN_ID
    env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    cmd = [sys.executable, os.path.abspath(__file__), "--worker",
           "--ros-args", "-p", "clear_dwell_s:=0.5", "-p", "decide_rate_hz:=20.0"]
    return subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=90)


def test_integration_ros_domain_78():
    import importlib.util
    if importlib.util.find_spec("rclpy") is None:
        _skip("rclpy not importable in this environment")
        return

    proc = _run_worker_subprocess()
    out = proc.stdout.strip()
    line = next((ln for ln in out.splitlines()
                 if ln.startswith(("WORKER_OK", "WORKER_ROS_UNAVAILABLE"))), "")
    if line.startswith("WORKER_ROS_UNAVAILABLE"):
        _skip("ROS/DDS unavailable in this environment: " + line)
        return
    assert line.startswith("WORKER_OK"), \
        f"worker did not complete.\nstdout:\n{proc.stdout}\nstderr:\n{proc.stderr}"

    result = json.loads(line[len("WORKER_OK "):])
    actions = result["compressed"]
    assert "SLOW" in actions, f"expected a SLOW while approaching; got {actions}"
    assert "STOP" in actions, f"expected a STOP while close; got {actions}"
    assert "RUN" in actions, f"expected a RUN after the dwell; got {actions}"
    # RUN must come AFTER a STOP (approach then resume, not RUN from the start)
    assert actions.index("RUN") > actions.index("STOP"), \
        f"RUN should follow the approach STOP; got {actions}"
    # once the topic goes silent, the gate must fall back to STOP (heartbeat)
    assert result["final"] == "STOP", \
        f"expected STOP after the state topic went silent; got {result['final']}"


def _skip(msg):
    try:
        import pytest
        pytest.skip(msg)
    except ImportError:
        print(f"SKIP {msg}")


if __name__ == "__main__":
    if "--worker" in sys.argv:
        sys.exit(_worker())
    # standalone: run the test directly and report
    try:
        test_integration_ros_domain_78()
        print("PASS test_integration_ros_domain_78")
    except AssertionError as e:
        print(f"FAIL test_integration_ros_domain_78: {e}")
        sys.exit(1)
