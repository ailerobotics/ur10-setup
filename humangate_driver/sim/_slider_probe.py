#!/usr/bin/env python3
"""Helper for measure_slider_ursim.sh: send a long trajectory and sweep the speed
slider while sampling applied scaling and actual joint motion. Not a standalone
tool — invoked by the shell wrapper once the driver is up."""
import math
import time

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ur_msgs.srv import SetSpeedSliderFraction

J = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
     'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


def main():
    rclpy.init()
    n = rclpy.create_node('slider_probe')
    js, scaling = {}, [None]
    n.create_subscription(JointState, '/joint_states',
                          lambda m: js.update(zip(m.name, m.position)), 20)
    n.create_subscription(Float64, '/speed_scaling_state_broadcaster/speed_scaling',
                          lambda m: scaling.__setitem__(0, m.data), 10)
    sl = n.create_client(SetSpeedSliderFraction, '/io_and_status_controller/set_speed_slider')
    fjt = ActionClient(n, FollowJointTrajectory,
                       '/scaled_joint_trajectory_controller/follow_joint_trajectory')

    def spin(t):
        e = time.time() + t
        while time.time() < e:
            rclpy.spin_once(n, timeout_sec=0.02)

    for _ in range(30):
        spin(1.0)
        if len(js) >= 6 and scaling[0] is not None:
            break
    print(f"discovered: njoints={len(js)} scaling={scaling[0]}")
    if len(js) < 6:
        print("ABORT: no joint states"); n.destroy_node(); rclpy.shutdown(); return
    sl.wait_for_service(timeout_sec=5); fjt.wait_for_server(timeout_sec=10)
    q0 = [js.get(j, 0.0) for j in J]

    def setslider(f):
        r = SetSpeedSliderFraction.Request(); r.speed_slider_fraction = float(f)
        fut = sl.call_async(r); rclpy.spin_until_future_complete(n, fut, timeout_sec=3)
        spin(0.3)

    tr = JointTrajectory(); tr.joint_names = J; npts = 120
    for k in range(npts + 1):
        fr = k / npts; s = 0.5 * (1 - math.cos(2 * math.pi * fr))
        p = list(q0); p[0] = q0[0] + 0.8 * s
        pt = JointTrajectoryPoint(); pt.positions = [float(v) for v in p]
        t = fr * 60
        pt.time_from_start.sec = int(t); pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
        tr.points.append(pt)
    tr.points[0].velocities = [0.0] * 6
    tr.points[-1].velocities = [0.0] * 6; tr.points[-1].accelerations = [0.0] * 6
    tr.header.stamp.sec = 0

    setslider(1.0)
    g = FollowJointTrajectory.Goal(); g.trajectory = tr
    f = fjt.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    gh = f.result()
    print(f"goal accepted: {gh.accepted if gh else None}")

    def sample(label):
        p0 = js.get(J[0]); spin(2.5); p1 = js.get(J[0])
        print(f"{label:16s} scaling={scaling[0]!s:>7}  pan={p1:.4f}  moved={abs(p1 - p0):.5f} rad/2.5s")
        return p1

    print("--- slider sweep during motion ---")
    sample("slider=1.0")
    setslider(0.5); sample("slider=0.5")
    setslider(0.1); sample("slider=0.1")
    setslider(0.0); sample("slider=0.0 (a)"); sample("slider=0.0 (b)")
    setslider(1.0); sample("slider=1.0 back")
    if gh and gh.accepted:
        gh.cancel_goal_async(); spin(1.0)
    n.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    main()
