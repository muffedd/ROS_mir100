#!/usr/bin/env python3
"""check_clearance.py - post-run verifier for the recorded validation run.

Checks the pre-registered criteria against RECORDED evidence (not TF belief):
  C2 per-leg physical arrival: GT distance to each driver goal at its result time
  C3 final GT distance to true goal
  C4 continuous clearance: min distance from the ACTUAL footprint polygon (placed at each
     GT pose) to every wall box parsed from the world SDF; contact = clearance <= 0
Inputs: recorder JSONL (GT), driver JSONL (goal/result events), world SDF, footprint polygon.
SDF assumption: static walls are <model> with pose "x y z r p yaw" and box collisions whose
<link>/<collision> poses are identity (true for hard_world.sdf). Composition is yaw-only.
"""
import argparse, json, math, sys
import xml.etree.ElementTree as ET

def seg_dist(p1, p2, q1, q2):
    def sub(a, b): return (a[0]-b[0], a[1]-b[1])
    def dot(a, b): return a[0]*b[0]+a[1]*b[1]
    def pt_seg(p, a, b):
        ab = sub(b, a); t = 0.0 if dot(ab, ab) == 0 else max(0.0, min(1.0, dot(sub(p, a), ab)/dot(ab, ab)))
        proj = (a[0]+t*ab[0], a[1]+t*ab[1])
        return math.hypot(p[0]-proj[0], p[1]-proj[1])
    if segments_intersect(p1, p2, q1, q2): return 0.0
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))

def segments_intersect(p1, p2, q1, q2):
    def o(a, b, c): return (b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])
    d1, d2 = o(q1, q2, p1), o(q1, q2, p2)
    d3, d4 = o(p1, p2, q1), o(p1, p2, q2)
    return ((d1 > 0) != (d2 > 0)) and ((d3 > 0) != (d4 > 0))

def point_in_poly(p, poly):
    inside = False; j = len(poly)-1
    for i in range(len(poly)):
        xi, yi = poly[i]; xj, yj = poly[j]
        if (yi > p[1]) != (yj > p[1]) and p[0] < (xj-xi)*(p[1]-yi)/(yj-yi)+xi:
            inside = not inside
        j = i
    return inside

def hull(points):
    P = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    if len(P) < 3: return P
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lo = []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    up = []
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]

def edges(poly):
    return [(poly[i], poly[(i+1) % len(poly)]) for i in range(len(poly))]

def poly_dist(A, B):
    if any(point_in_poly(v, B) for v in A) or any(point_in_poly(v, A) for v in B): return 0.0
    return min(seg_dist(a1, a2, b1, b2) for a1, a2 in edges(A) for b1, b2 in edges(B))

def parse_walls(sdf_path):
    root = ET.parse(sdf_path).getroot()
    walls = []
    for model in root.iter('model'):
        mp = (model.findtext('pose') or '0 0 0 0 0 0').split()
        mx, my, myaw = float(mp[0]), float(mp[1]), float(mp[5] if len(mp) > 5 else 0)
        for col in model.iter('collision'):
            box = col.find('./geometry/box/size')
            if box is None: continue
            sx, sy = [float(v) for v in box.text.split()[:2]]
            if sx > 5 and sy > 5: continue  # ground plane, not a wall
            cp = (col.findtext('pose') or '0 0 0 0 0 0').split()
            cx, cy = float(cp[0]), float(cp[1])
            c, s = math.cos(myaw), math.sin(myaw)
            wx, wy = mx + c*cx - s*cy, my + s*cx + c*cy
            c2, s2 = math.cos(myaw), math.sin(myaw)
            corners = [(wx + c2*px - s2*py, wy + s2*px + c2*py)
                       for px in (sx/2, -sx/2) for py in (sy/2, -sy/2)]
            walls.append({'name': model.get('name'), 'poly': corners})
    return walls

def yaw_of(rec):
    qz, qw = rec.get('qz'), rec.get('qw')
    if qz is None or qw is None: return 0.0
    return 2*math.atan2(qz, qw)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--recorder', required=True)
    ap.add_argument('--driver', required=True)
    ap.add_argument('--footprint', required=True, help='JSON polygon string or @file')
    ap.add_argument('--goal', nargs=2, type=float, required=True, help='true goal x y (world frame)')
    ap.add_argument('--map-origin', nargs=2, type=float, default=[-4.31, -2.81],
                    help='world coords of SLAM map origin (spawn), used to convert driver goals')
    ap.add_argument('--arrival-tol', type=float, default=0.35)
    ap.add_argument('--final-tol', type=float, default=0.50)
    ap.add_argument('--summary', default=None)
    ap.add_argument('--n-legs', type=int, default=15, help='required leg count: result indices must be exactly 1..N')
    a = ap.parse_args()
    fp = json.loads(open(a.footprint[1:]).read() if a.footprint.startswith('@') else a.footprint)
    walls = parse_walls(a.world)
    gts, driver = [], []
    for line in open(a.recorder):
        try: r = json.loads(line)
        except Exception: continue
        if r.get('ev') == 'gt': gts.append(r)
    for line in open(a.driver):
        try: r = json.loads(line)
        except Exception: continue
        if r.get('ev') in ('goal', 'result', 'leg_failed', 'route_complete'): driver.append(r)
    gts.sort(key=lambda r: r['t_wall'])
    def gt_at(tw):
        return min(gts, key=lambda r: abs(r['t_wall']-tw), default=None)

    # SWEPT clearance: hull of footprint at sample k AND k+1 encloses the translation sweep
    # between samples (rotation sweep conservative for small dtheta; max step reported below).
    def placed(r):
        yw = yaw_of(r); c, s = math.cos(yw), math.sin(yw)
        return [(r['x'] + c*px - s*py, r['y'] + s*px + c*py) for px, py in fp]
    clearance = []
    max_step = 0.0
    for k in range(len(gts)):
        poly = placed(gts[k])
        if k + 1 < len(gts):
            poly2 = placed(gts[k + 1])
            step = max(math.hypot(poly2[j][0] - poly[j][0], poly2[j][1] - poly[j][1]) for j in range(len(poly)))
            max_step = max(max_step, step)
            poly = hull(list(poly) + list(poly2))
        dmin = min((poly_dist(poly, w['poly']), w['name']) for w in walls)[0]
        clearance.append({'t_wall': gts[k]['t_wall'], 'x': gts[k]['x'], 'y': gts[k]['y'], 'clearance': round(dmin, 4)})
    cmin = min(c['clearance'] for c in clearance) if clearance else None
    cmin_at = min(clearance, key=lambda c: c['clearance']) if clearance else None

    legs = []
    mo = a.map_origin
    for r in driver:
        if r.get('ev') == 'result':
            gx, gy = r['tgt'][0] + mo[0], r['tgt'][1] + mo[1]  # map frame -> world
            g = gt_at(r['t_wall'])
            d = math.hypot(g['x']-gx, g['y']-gy) if g else None
            legs.append({'index': r['index'], 'status': r['status'],
                         'gt_dist_to_goal': round(d, 3) if d is not None else None,
                         'ok_c2': (d is not None and d <= a.arrival_tol and r['status'] == 4)})
    final_gt = gts[-1] if gts else None
    final_d = math.hypot(final_gt['x']-a.goal[0], final_gt['y']-a.goal[1]) if final_gt else None
    idxs = sorted(l['index'] for l in legs)
    summary = {
        'n_gt_samples': len(gts), 'n_walls': len(walls),
        'c1_ok': idxs == list(range(1, a.n_legs + 1)) and all(l['status'] == 4 for l in legs),
        'c1_leg_indices': idxs, 'c1_required': f'1..{a.n_legs}',
        'c4_max_sample_step_m': round(max_step, 4),
        'c2_all_legs_arrived': all(l['ok_c2'] for l in legs) and len(legs) > 0, 'legs': legs,
        'c3_final_dist': round(final_d, 3) if final_d is not None else None,
        'c3_ok': final_d is not None and final_d <= a.final_tol,
        'c4_min_clearance': cmin, 'c4_ok': cmin is not None and cmin > 0.0,
        'c4_min_at': cmin_at,
        'c4_warn_band_samples(0-0.05m)': sum(1 for c in clearance if 0 < c['clearance'] < 0.05),
    }
    txt = json.dumps(summary, indent=2)
    print(txt)
    if a.summary: open(a.summary, 'w').write(txt)

if __name__ == '__main__':
    main()
