#!/usr/bin/env python3
"""nav_driver5.py - portable sequential-waypoint Nav2 driver for the RECORDED
hard-world validation run.

Label: WORLD-ORACLE waypoint guidance (route file authored from known world
geometry). Not autonomous exploration; a completed run is not a swept-path
safety proof (clearance comes from check_clearance.py against gz ground truth).

Changes vs v4 (nav_driver4_final.py):
  - full sim-time goal stamps (sec + nanosec from /clock; v4 floored to seconds)
  - per-leg timeout with goal cancel (exit 5) and global route timeout (exit 6)
  - JSONL machine-checkable event log; waypoints/log paths are CLI args (no /tmp hardcode)
  - map-growth wait bounded in SIM time (v4 used wall time)
Unchanged sequencing: nearest-waypoint resume, unconditional advance on status 4.
Exit codes: 0 complete | 2 TF never ready | 4 leg status!=4 | 5 leg timeout | 6 global timeout | 7 map growth timeout | 8 start-index mismatch (would have skipped legs)
"""
import argparse, json, math, sys, time
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--waypoints', required=True, help='JSON [[x,y],...] in map frame, last entry = final goal')
    ap.add_argument('--log', required=True, help='JSONL event log path')
    ap.add_argument('--leg-timeout', type=float, default=120.0, help='sim-seconds per waypoint')
    ap.add_argument('--global-timeout', type=float, default=900.0, help='sim-seconds for the whole route')
    ap.add_argument('--map-margin', type=float, default=0.5, help='goal must be this far inside current SLAM map bounds')
    ap.add_argument('--map-wait', type=float, default=60.0, help='sim-seconds to wait for map growth per waypoint')
    ap.add_argument('--expect-start-index', type=int, default=1,
                    help='first leg index that MUST run; mismatch aborts loudly (exit 8) instead of silently skipping legs')
    a = ap.parse_args()
    wps = [tuple(map(float, p)) for p in json.load(open(a.waypoints))]
    logf = open(a.log, 'a', buffering=1)

    rclpy.init()
    n = rclpy.create_node('recorded_route_driver',
                          parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    def sim_now():
        t = n.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9
    def rec(ev, **kw):
        kw.update(ev=ev, t_sim=round(sim_now(), 3), t_wall=round(time.time(), 3))
        logf.write(json.dumps(kw) + '\n')
    def pose():
        try:
            t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time(),
                                     timeout=rclpy.duration.Duration(seconds=1.0))
            s = t.header.stamp
            return (t.transform.translation.x, t.transform.translation.y, s.sec + s.nanosec * 1e-9)
        except Exception:
            return None

    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    ac = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    mapinfo = {}
    def mcb(m):
        mapinfo['o'] = (m.info.origin.position.x, m.info.origin.position.y)
        mapinfo['w'] = m.info.width * m.info.resolution
        mapinfo['h'] = m.info.height * m.info.resolution
    n.create_subscription(OccupancyGrid, '/map', mcb, 10)
    def inb(x, y):
        if 'o' not in mapinfo: return False
        ox, oy = mapinfo['o']; m = a.map_margin
        return ox + m <= x <= ox + mapinfo['w'] - m and oy + m <= y <= oy + mapinfo['h'] - m

    rec('start', waypoints=a.waypoints, n_wp=len(wps), leg_timeout=a.leg_timeout,
        global_timeout=a.global_timeout, argv=sys.argv)

    # TF gate: require an advancing map->base_footprint stamp before any goal
    t0 = time.time(); last = -1.0
    while time.time() - t0 < 180:
        rclpy.spin_once(n, timeout_sec=0.5)
        p = pose()
        if p and p[2] > last >= 0:
            rec('tf_ready', stamp=p[2], robot=[round(p[0], 3), round(p[1], 3)])
            break
        if p: last = p[2]
        time.sleep(1.0)
    else:
        rec('tf_timeout'); print('TF TIMEOUT', flush=True); sys.exit(2)
    ac.wait_for_server()

    t_global0 = sim_now()
    p0 = None
    while p0 is None:
        rclpy.spin_once(n, timeout_sec=1.0); p0 = pose()
    i = min(range(len(wps)), key=lambda k: math.hypot(wps[k][0] - p0[0], wps[k][1] - p0[1]))
    rec('resume_at', index=i, robot=[round(p0[0], 3), round(p0[1], 3)])
    i += 1  # nearest waypoint is where we ARE; target the NEXT
    if i != a.expect_start_index:
        rec('start_index_mismatch', found=i, expected=a.expect_start_index)
        print(f'START INDEX MISMATCH: would start at {i}, expected {a.expect_start_index} - aborting rather than skipping legs', flush=True)
        sys.exit(8)

    while i < len(wps):
        if sim_now() - t_global0 > a.global_timeout:
            rec('global_timeout', index=i); print('GLOBAL TIMEOUT', flush=True); sys.exit(6)
        tgt = wps[i]
        if not inb(*tgt):
            rec('map_wait', index=i, tgt=list(tgt), bounds=mapinfo.get('o'))
            t1 = sim_now()
            while sim_now() - t1 < a.map_wait and not inb(*tgt):
                rclpy.spin_once(n, timeout_sec=1.0)
            if not inb(*tgt):
                rec('map_wait_timeout', index=i, tgt=list(tgt))
                print('MAP GROWTH TIMEOUT', flush=True); sys.exit(7)
        final = (i == len(wps) - 1)
        g = NavigateToPose.Goal(); g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = n.get_clock().now().to_msg()  # FULL sec+nanosec sim stamp
        g.pose.pose.position.x = float(tgt[0]); g.pose.pose.position.y = float(tgt[1])
        g.pose.pose.orientation.w = 1.0
        p_send = pose()
        rec('goal', index=i, tgt=list(tgt), final=final,
            stamp=[g.pose.header.stamp.sec, g.pose.header.stamp.nanosec],
            robot_at_send=[round(p_send[0], 3), round(p_send[1], 3)] if p_send else None)
        fut = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, fut)
        h = fut.result()
        if not h.accepted:
            rec('rejected', index=i); time.sleep(3.0); continue
        rf = h.get_result_async()
        t_leg0 = sim_now(); status = None; lastlog = sim_now()
        while rclpy.ok():
            rclpy.spin_once(n, timeout_sec=0.5)
            if rf.done():
                status = rf.result().status; break
            now = sim_now()
            if now - t_leg0 > a.leg_timeout:
                rec('leg_timeout', index=i, tgt=list(tgt))
                h.cancel_goal_async()
                print('LEG TIMEOUT', flush=True); sys.exit(5)
            if now - t_global0 > a.global_timeout:
                rec('global_timeout', index=i); h.cancel_goal_async()
                print('GLOBAL TIMEOUT', flush=True); sys.exit(6)
            if now - lastlog > 5.0:
                p2 = pose()
                if p2:
                    rec('progress', index=i, robot=[round(p2[0], 3), round(p2[1], 3)], tf_stamp=round(p2[2], 3))
                lastlog = now
        p_end = pose()
        rec('result', index=i, tgt=list(tgt), status=status, dur_sim=round(sim_now() - t_leg0, 2),
            robot_at_result=[round(p_end[0], 3), round(p_end[1], 3)] if p_end else None,
            tf_stamp=round(p_end[2], 3) if p_end else None)
        print(f'LEG {i} status={status}', flush=True)
        if status != 4:
            rec('leg_failed', index=i, status=status)
            print('LEG FAILED, stopping', flush=True); sys.exit(4)
        i += 1  # Nav2 confirmed arrival in its consistent frame; do not re-send (v3 loop bug)

    rec('route_complete', dur_sim=round(sim_now() - t_global0, 2))
    print('ROUTE COMPLETE - FINAL GOAL REACHED', flush=True)
    sys.exit(0)

if __name__ == '__main__':
    main()
