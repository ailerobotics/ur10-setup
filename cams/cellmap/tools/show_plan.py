#!/usr/bin/env python3
"""
Publish saved plans as RViz markers on /cellmap/plan_path (latched-ish).

The captain wants to WATCH the plan against the map before anything moves.
MoveIt's own trajectory display needs the MoveIt RViz plugins and a live
move_group session; a plain MarkerArray works in the nvblox container's RViz,
which is the one that can draw the map at all.

Draws, per plan: the tool0 polyline, a green ball at the start, a red ball at
the goal, and a label. Publishes until Ctrl-C. Commands nothing.

  python3 show_plan.py cams/cellmap/plans/mt-tag0.yaml cams/cellmap/plans/mt-tag1.yaml
"""
import sys
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

COLOURS = [(0.05, 0.65, 0.65), (0.75, 0.20, 0.15), (0.20, 0.45, 0.80)]


def main():
    paths = sys.argv[1:]
    if not paths:
        sys.exit('usage: show_plan.py <plan.yaml> [...]')
    rclpy.init()
    n = Node('show_plan')
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL   # late RViz still sees it
    pub = n.create_publisher(MarkerArray, '/cellmap/plan_path', qos)

    ma = MarkerArray()
    mid = 0
    for i, p in enumerate(paths):
        d = yaml.safe_load(open(p))
        tp = d.get('tool_path')
        if not tp:
            print(f'{p}: no tool_path (plan failed?) — skipped')
            continue
        r, g, b = COLOURS[i % len(COLOURS)]
        line = Marker()
        line.header.frame_id = 'base_link'
        line.ns, line.id, line.type, line.action = 'plan', mid, Marker.LINE_STRIP, Marker.ADD
        line.scale.x = 0.012
        line.color = ColorRGBA(r=r, g=g, b=b, a=1.0)
        line.pose.orientation.w = 1.0
        line.points = [Point(x=float(q[0]), y=float(q[1]), z=float(q[2])) for q in tp]
        ma.markers.append(line); mid += 1
        for pt, (cr, cg, cb), tag in ((tp[0], (0.1, 0.8, 0.1), 'start'),
                                      (tp[-1], (0.9, 0.1, 0.1), 'goal')):
            s = Marker()
            s.header.frame_id = 'base_link'
            s.ns, s.id, s.type, s.action = 'plan', mid, Marker.SPHERE, Marker.ADD
            s.scale.x = s.scale.y = s.scale.z = 0.045
            s.color = ColorRGBA(r=cr, g=cg, b=cb, a=0.95)
            s.pose.position = Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
            s.pose.orientation.w = 1.0
            ma.markers.append(s); mid += 1
        t = Marker()
        t.header.frame_id = 'base_link'
        t.ns, t.id = 'plan', mid
        t.type, t.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        t.scale.z = 0.055
        t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.95)
        t.pose.position = Point(x=float(tp[-1][0]), y=float(tp[-1][1]),
                                z=float(tp[-1][2]) + 0.09)
        t.pose.orientation.w = 1.0
        t.text = f"{d.get('label', p)}  {d.get('tool_path_m', 0):.2f} m"
        ma.markers.append(t); mid += 1
        print(f'{p}: {len(tp)} points')

    print(f'publishing {len(ma.markers)} markers on /cellmap/plan_path — Ctrl-C to stop')
    timer_state = {'n': 0}

    def tick():
        pub.publish(ma)
        timer_state['n'] += 1
    n.create_timer(1.0, tick)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
