#!/usr/bin/env python3
"""run_recorder5.py - continuous evidence recorder for the recorded validation run.

One JSONL out file with interleaved, fully-stamped records:
  ev=tf       map->odom, odom->base_footprint, map->base_footprint @ 10 Hz (header sec+nanosec);
              TF lookup FAILURES are recorded too (ev=tf_error) - gaps are evidence, not silence.
  ev=odom     /odom nav_msgs/Odometry, full rate
  ev=clock    /clock decimated to 1 Hz (wall<->sim alignment)
  ev=gt       gz ground truth of the robot model from /world/<world>/pose/info (wall-stamped
              at receive; gz Pose_V carries no header - align to sim via ev=clock)
  ev=contact  contact topic blocks IF gz exposes one (checked at startup; usually none ->
              ev=note; collision evidence then comes from post-hoc clearance vs world geometry)
  ev=note     recorder lifecycle / limitations
Also writes run_meta.json (argv, ROS_DOMAIN_ID/GZ_PARTITION, sha256 of files passed via --sha-file)
and optionally a rosbag2 of /tf /odom /clock via --bag-dir.
"""
import argparse, hashlib, json, os, re, subprocess, sys, threading, time
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

NUM = r'[-+0-9.eE]+'

class GzPoseReader(threading.Thread):
    """Parse `gz topic -e -t /world/<w>/pose/info` text stream; emit completed pose blocks."""
    def __init__(self, world, model, cb):
        super().__init__(daemon=True)
        self.cmd = ['gz', 'topic', '-e', '-t', f'/world/{world}/pose/info']
        self.model, self.cb, self.proc = model, cb, None
    def run(self):
        try:
            self.proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except Exception as e:
            self.cb({'ev': 'note', 'text': f'gz pose/info reader failed to start: {e}'}); return
        depth = 0; block = None
        for line in self.proc.stdout:
            opens = line.count('{'); closes = line.count('}')
            if depth == 0 and line.strip().startswith('pose {'):
                block = [line]; depth = opens - closes
                if depth == 0: self._emit(block); block = None
                continue
            if block is not None:
                block.append(line); depth += opens - closes
                if depth <= 0:
                    self._emit(block); block = None; depth = 0
    def _emit(self, lines):
        txt = ''.join(lines)
        m = re.search(r'name: "([^"]+)"', txt)
        if not m or m.group(1) != self.model: return
        def f(field):
            r = re.search(field + r':\s*(' + NUM + ')', txt)
            return float(r.group(1)) if r else None
        pos = re.search(r'position\s*{(.*?)}', txt, re.S)
        ori = re.search(r'orientation\s*{(.*?)}', txt, re.S)
        if not pos: return
        px = re.search(r'x:\s*(' + NUM + ')', pos.group(1)); py = re.search(r'y:\s*(' + NUM + ')', pos.group(1))
        rec = {'ev': 'gt', 'x': float(px.group(1)), 'y': float(py.group(1))}
        if ori:
            for k in 'xyzw':
                r = re.search(k + r':\s*(' + NUM + ')', ori.group(1))
                rec['q' + k] = float(r.group(1)) if r else None
        self.cb(rec)
    def stop(self):
        if self.proc: self.proc.terminate()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', default='hard_world')
    ap.add_argument('--model', default='mir100')
    ap.add_argument('--out', required=True)
    ap.add_argument('--bag-dir', default=None)
    ap.add_argument('--sha-file', action='append', default=[])
    a = ap.parse_args()
    out = open(a.out, 'a', buffering=1)
    lock = threading.Lock()
    def rec(d):
        d.setdefault('t_wall', round(time.time(), 3))
        with lock: out.write(json.dumps(d) + '\n')

    meta = {'argv': sys.argv, 'ROS_DOMAIN_ID': os.environ.get('ROS_DOMAIN_ID'),
            'GZ_PARTITION': os.environ.get('GZ_PARTITION'), 't_wall_start': time.time(),
            'sha256': {p: hashlib.sha256(open(p, 'rb').read()).hexdigest() for p in a.sha_file if os.path.exists(p)}}
    with open(a.out + '.run_meta.json', 'w') as f: json.dump(meta, f, indent=2)

    rclpy.init()
    n = rclpy.create_node('run_recorder', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
    last_clock = {'s': -1}
    def clk_cb(m):
        if m.clock.sec != last_clock['s']:
            last_clock['s'] = m.clock.sec
            rec({'ev': 'clock', 'sec': m.clock.sec, 'nanosec': m.clock.nanosec})
    n.create_subscription(Clock, '/clock', clk_cb, 10)
    def odom_cb(m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        rec({'ev': 'odom', 'stamp': [m.header.stamp.sec, m.header.stamp.nanosec],
             'x': round(p.x, 4), 'y': round(p.y, 4), 'qz': round(q.z, 5), 'qw': round(q.w, 5),
             'vx': round(m.twist.twist.linear.x, 4), 'wz': round(m.twist.twist.angular.z, 4)})
    n.create_subscription(Odometry, '/odom', odom_cb, 200)
    FRAMES = [('map', 'odom'), ('odom', 'base_footprint'), ('map', 'base_footprint')]
    def tf_tick():
        for tgt, src in FRAMES:
            try:
                t = buf.lookup_transform(tgt, src, rclpy.time.Time())
                tr, q, s = t.transform.translation, t.transform.rotation, t.header.stamp
                rec({'ev': 'tf', 'frame': f'{tgt}->{src}', 'stamp': [s.sec, s.nanosec],
                     'x': round(tr.x, 4), 'y': round(tr.y, 4), 'qz': round(q.z, 5), 'qw': round(q.w, 5)})
            except Exception as e:
                rec({'ev': 'tf_error', 'frame': f'{tgt}->{src}', 'err': str(e)[:200]})
    n.create_timer(0.1, tf_tick)

    # contact topics if gz exposes any (usually none without a ContactSensor -> post-hoc clearance instead)
    try:
        topics = subprocess.run(['gz', 'topic', '-l'], capture_output=True, text=True, timeout=20).stdout
        contacts = [t for t in topics.splitlines() if 'contact' in t.lower()]
    except Exception:
        contacts = []
    rec({'ev': 'note', 'text': f'contact topics: {contacts or "none exposed; clearance is computed post-hoc from GT vs world geometry"}'})

    gzreader = GzPoseReader(a.world, a.model, rec)
    gzreader.start()
    bag = None
    if a.bag_dir:
        try:
            bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', a.bag_dir, '/tf', '/odom', '/clock'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            rec({'ev': 'note', 'text': f'rosbag recording to {a.bag_dir}'})
        except Exception as e:
            rec({'ev': 'note', 'text': f'rosbag failed to start: {e}'})
    rec({'ev': 'note', 'text': 'recorder started'})
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        rec({'ev': 'note', 'text': 'recorder stopping'})
        gzreader.stop()
        if bag: bag.terminate()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
