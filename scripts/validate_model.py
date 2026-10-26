#!/usr/bin/env python3
"""Bounded motion validation for the MiR100 simulation model.

Sends a fixed cmd_vel profile and logs commanded velocity, wheel
odometry (/odom), and Gazebo ground truth (/world/empty/pose/info via
gz transport) at sim time. Prints per-trial odometry vs ground-truth
distance and heading change. Simulation self-consistency only.

Prerequisites (see docs/MODEL_VALIDATION.txt for the full session):
  1. ros2 launch mir_description gazebo.launch.py gui:=false
  2. ros2 run ros_gz_bridge parameter_bridge \
       /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
       /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
       /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry
Usage: python3 scripts/validate_model.py --mode straight --trials 3 --out /tmp/valid.csv
"""
import argparse, csv, math, sys, threading, time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import gz.transport13 as gzt
from gz.msgs10.pose_v_pb2 import Pose_V

WHEEL_RADIUS = 0.0625        # mir100_ref.urdf.xacro
WHEEL_SEPARATION = 0.445208  # 2 * mir_100_act_wheel_dy

state = {'odom': None, 'gt': None, 'lock': threading.Lock()}

def gz_cb(msg):
    for p in msg.pose:
        if p.name == 'mir100':
            with state['lock']:
                state['gt'] = (p.position.x, p.position.y,
                               p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

def yawq(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['straight', 'turn'], required=True)
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--out', default='/tmp/validate_model_log.csv')
    ap.add_argument('--lin', type=float, default=0.2, help='straight: m/s')
    ap.add_argument('--ang', type=float, default=0.5, help='turn: rad/s')
    ap.add_argument('--duration', type=float, default=None,
                    help='sim seconds per trial (default 10 straight / 6.2832 turn)')
    args = ap.parse_args()
    lin = args.lin if args.mode == 'straight' else 0.0
    ang = args.ang if args.mode == 'turn' else 0.0
    T = args.duration or (10.0 if args.mode == 'straight' else 6.2832)

    gznode = gzt.Node()
    gznode.subscribe(Pose_V, '/world/empty/pose/info', gz_cb)
    rclpy.init()
    n = rclpy.create_node('validate_model')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    n.create_subscription(Odometry, '/odom',
                          lambda m: state.__setitem__('odom', m), 10)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    out = csv.writer(open(args.out, 'w', newline=''))
    out.writerow(['t_sim', 'trial', 'phase', 'cmd_lin', 'cmd_ang',
                  'odom_x', 'odom_y', 'odom_yaw_deg', 'gt_x', 'gt_y', 'gt_yaw_deg'])

    simnow = lambda: n.get_clock().now().nanoseconds / 1e9

    def snap():
        with state['lock']:
            o = state['odom'].pose.pose
            g = state['gt']
        return dict(ox=o.position.x, oy=o.position.y,
                    oyaw=yawq(o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w),
                    gx=g[0], gy=g[1], gyaw=yawq(g[2], g[3], g[4], g[5]))

    t0 = time.time()
    while (state['odom'] is None or state['gt'] is None) and time.time() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.2)
    if state['odom'] is None or state['gt'] is None:
        print('streams missing: odom=%s gt=%s' % (state['odom'] is not None, state['gt'] is not None))
        sys.exit(2)
    # Baseline: let clock/odom/GT streaming settle before the timed profile,
    # so trial measurements start from a stable baseline.
    t_base = simnow()
    while simnow() - t_base < 2.0:
        rclpy.spin_once(n, timeout_sec=0.05)

    for ti in range(args.trials):
        s0 = snap()
        ts0 = simnow()
        tw = Twist(); tw.linear.x = lin; tw.angular.z = ang
        while simnow() - ts0 < T:
            pub.publish(tw)
            rclpy.spin_once(n, timeout_sec=0.02)
            s = snap()
            out.writerow([round(simnow(), 3), ti, 'drive', lin, ang,
                          s['ox'], s['oy'], math.degrees(s['oyaw']),
                          s['gx'], s['gy'], math.degrees(s['gyaw'])])
        pub.publish(Twist())
        ts1 = simnow()
        while simnow() - ts1 < 3.0:  # settle
            rclpy.spin_once(n, timeout_sec=0.05)
            s = snap()
            out.writerow([round(simnow(), 3), ti, 'settle', 0, 0,
                          s['ox'], s['oy'], math.degrees(s['oyaw']),
                          s['gx'], s['gy'], math.degrees(s['gyaw'])])
        s1 = snap()
        dod = math.hypot(s1['ox'] - s0['ox'], s1['oy'] - s0['oy'])
        dgt = math.hypot(s1['gx'] - s0['gx'], s1['gy'] - s0['gy'])
        dyod = math.degrees((s1['oyaw'] - s0['oyaw'] + math.pi) % (2 * math.pi) - math.pi)
        dygt = math.degrees((s1['gyaw'] - s0['gyaw'] + math.pi) % (2 * math.pi) - math.pi)
        print('TRIAL %d: cmd lin=%.2f ang=%.2f T=%.2fs | odom d=%.3f dyaw=%.1f | GT d=%.3f dyaw=%.1f'
              % (ti, lin, ang, T, dod, dyod, dgt, dygt), flush=True)
    print('log: %s' % args.out, flush=True)

if __name__ == '__main__':
    main()
