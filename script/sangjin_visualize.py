# -*- coding: utf-8 -*-
# ===============================================================
# rosbag 스냅샷 + TF 정규화 + 경로 누적 플롯 (feasible/raw/best, corridors, robot/target/dynamics)
# - 동적장애물: /bpmp_simulator/obstacle_state_list (ObjectStateList)
# - feasible/raw/best: visualization_msgs/MarkerArray (1번 코드 정렬 로직 유지)
# - corridors: decomp_ros_msgs/PolyhedronArray (2번 코드 모양 정확도 유지: H-rep half-plane clipping)
# ===============================================================

from collections import defaultdict
from typing import List, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

# -------------------- 설정 --------------------
MODE = "ros1"  # "ros1" 또는 "ros2"
BAG_PATH = "/home/seungwoo/alchemist_ws/src/BPMP-Tracker/script/airsim_results/top_down.bag"
TARGET_FRAME = "map"
TF_TOPICS = ["/tf", "/tf_static"]

# 토픽들
TOPIC_OBS_LIST = "/bpmp_simulator/obstacle_state_list"
FEAS_TOPICS = ["/bpmp_tracker/feasible_primitives"]
RAW_TOPICS  = ["/bpmp_tracker/raw_primitive"]
BEST_TOPICS = ["/bpmp_tracker/best_primitives"]
CORRIDOR_T  = "/bpmp_predictor/corridor"
CORRIDOR_R  = "/bpmp_tracker/corridor"
ROBOT_POSE_TOPICS  = ["/airsim/gtpose"]
TARGET_POSE_TOPICS = ["/airsim/object/target/pose", "/target_pose", "/bpmp_simulator/target_state"]
TARGET_BEST_TOPIC = ["/bpmp_predictor/TargetBestPrimitive"]

# 시간 설정
USE_ABS_TIME = False
T_OFFSET_SEC = 15.9
ABS_TIME_SEC = 0.0
PATH_WINDOW_SEC = 10.0
CYL_RADIUS = 0.5
SLICE_Z = 0.3
SLICE_EPS = 0.5

# -------------------- 유틸 --------------------
def pick_closest(msg_list: List[Tuple[float, object]], t_query: float):
    if not msg_list: return None
    idx = int(np.argmin([abs(t - t_query) for t, _ in msg_list]))
    return msg_list[idx]

def pose_xyz_from_any(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose") and hasattr(msg.pose.pose, "position"):
        p = msg.pose.pose.position
        return np.array([float(p.x), float(p.y), float(p.z)], dtype=float), getattr(msg, 'header', None)
    if hasattr(msg, "pose") and hasattr(msg.pose, "position"):
        p = msg.pose.position
        return np.array([float(p.x), float(p.y), float(p.z)], dtype=float), getattr(msg, 'header', None)
    if hasattr(msg, "px") and hasattr(msg, "py") and hasattr(msg, "pz"):
        return np.array([float(msg.px), float(msg.py), float(msg.pz)], dtype=float), None
    return None, None

def hull2d_monotone_chain(points_xy: np.ndarray):
    pts = np.unique(points_xy, axis=0)
    if len(pts) <= 1: return pts
    pts = pts[pts[:,0].argsort(kind='mergesort')]
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0: lower.pop()
        lower.append(tuple(p))
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0: upper.pop()
        upper.append(tuple(p))
    return np.array(lower[:-1] + upper[:-1], dtype=float)

def quat_to_R(x,y,z,w):
    xx, yy, zz = x*x, y*y, z*z; xy, xz, yz = x*y, x*z, y*z; wx, wy, wz = w*x, w*y, w*z
    return np.array([[1-2*(yy+zz), 2*(xy-wz),   2*(xz+wy)],
                     [2*(xy+wz),   1-2*(xx+zz), 2*(yz-wx)],
                     [2*(xz-wy),   2*(yz+wx),   1-2*(xx+yy)]], dtype=float)

def build_tf_store(storage):
    from collections import defaultdict as _dd
    tf_store = _dd(list)
    for tf_topic in TF_TOPICS:
        for ts, msg in storage.get(tf_topic, []):
            for tr in getattr(msg, 'transforms', []):
                parent = getattr(getattr(tr, 'header', None), 'frame_id', '')
                child  = getattr(tr, 'child_frame_id', '')
                t = getattr(tr, 'transform', None)
                if not parent or not child or t is None: continue
                trans = getattr(t, 'translation', None); rot = getattr(t, 'rotation', None)
                if trans is None or rot is None: continue
                tf_store[(parent, child)].append((ts,
                    np.array([float(trans.x), float(trans.y), float(trans.z)], dtype=float),
                    (float(rot.x), float(rot.y), float(rot.z), float(rot.w))))
    for k in tf_store: tf_store[k].sort(key=lambda x: x[0])
    return tf_store

def lookup_tf(tf_store, source, target, t_query):
    if source == target or not source or not target: return np.eye(3), np.zeros(3)
    direct = (target, source); inverse = (source, target)
    def pick_nearest(lst):
        idx = int(np.argmin([abs(ts - t_query) for ts,_,_ in lst])); return lst[idx]
    if direct in tf_store and tf_store[direct]:
        _, t, q = pick_nearest(tf_store[direct]); R = quat_to_R(*q[:3], q[3]); return R, t
    if inverse in tf_store and tf_store[inverse]:
        _, t, q = pick_nearest(tf_store[inverse]); R = quat_to_R(*q[:3], q[3]); Rinv = R.T; return Rinv, -Rinv@t
    return np.eye(3), np.zeros(3)

def transform_points(P, R, t):
    if P.size == 0: return P
    return (R @ P.T).T + t

def markerarray_extract_lines_xy_in_target(marker_array_msg, tf_store, t_query):
    segs = []
    for mk in getattr(marker_array_msg, 'markers', []):
        src = getattr(getattr(mk, 'header', None), 'frame_id', TARGET_FRAME)
        pts = getattr(mk, 'points', [])
        if pts:
            P = np.array([[pt.x, pt.y, pt.z] for pt in pts], dtype=float)
            R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t_query)
            Pm = transform_points(P, R, tt); segs.append(Pm[:, :2])
        elif hasattr(mk, 'pose'):
            p = getattr(mk, 'position', None) if hasattr(mk, 'position') else getattr(mk.pose, 'position', None)
            if p is not None:
                P = np.array([[p.x, p.y, p.z]], dtype=float)
                R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t_query)
                Pm = transform_points(P, R, tt); segs.append(Pm[:, :2])
    return segs

def marker_or_array_extract_lines_xy_in_target(msg, tf_store, t_query):
    """MarkerArray 또는 단일 Marker 모두 지원"""
    if hasattr(msg, 'markers'):
        return markerarray_extract_lines_xy_in_target(msg, tf_store, t_query)
    # 단일 Marker
    segs = []
    src = getattr(getattr(msg, 'header', None), 'frame_id', TARGET_FRAME)
    pts = getattr(msg, 'points', [])
    if pts:
        P = np.array([[pt.x, pt.y, pt.z] for pt in pts], dtype=float)
        R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t_query)
        Pm = transform_points(P, R, tt); segs.append(Pm[:, :2])
    else:
        p = getattr(msg, 'position', None) if hasattr(msg, 'position') else getattr(getattr(msg, 'pose', None), 'position', None)
        if p is not None:
            P = np.array([[p.x, p.y, p.z]], dtype=float)
            R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t_query)
            Pm = transform_points(P, R, tt); segs.append(Pm[:, :2])
    return segs

# -------------------- (2번 방식) 코리더: H-rep 기반 Half-plane clipping --------------------
def polyhedronarray_slice_xy_in_target(polyarr_msg, tf_store, t_query):
    polys_xy = []
    src = getattr(getattr(polyarr_msg, 'header', None), 'frame_id', TARGET_FRAME)
    list_attr = None
    for cand in ["polyhedra","polyhedrons","polyhedron","polys"]:
        if hasattr(polyarr_msg, cand):
            list_attr = getattr(polyarr_msg, cand)
            break
    if list_attr is None:
        return polys_xy

    def clip_with_halfplane(poly_xy, a, b, c):
        if poly_xy is None or len(poly_xy) == 0:
            return []
        out = []
        n = len(poly_xy)
        for i in range(n):
            P = poly_xy[i]
            Q = poly_xy[(i+1) % n]
            wP = a*P[0] + b*P[1] - c
            wQ = a*Q[0] + b*Q[1] - c
            insideP = (wP <= 1e-9)
            insideQ = (wQ <= 1e-9)
            if insideP and insideQ:
                out.append(Q)
            elif insideP and not insideQ:
                t = wP / (wP - wQ)
                I = P + t*(Q - P)
                out.append(I)
            elif (not insideP) and insideQ:
                t = wP / (wP - wQ)
                I = P + t*(Q - P)
                out.append(I)
                out.append(Q)
        return out

    R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t_query)
    for poly in list_attr:
        pts = getattr(poly, 'points', [])
        nms = getattr(poly, 'normals', [])
        if not pts:
            continue
        P0 = np.array([[p.x, p.y, p.z] for p in pts], dtype=float)
        Pm = transform_points(P0, R, tt)

        if not nms or len(nms) != len(pts):
            # 포인트만 있을 때는 가까운 z-버텍스 + 볼록껍질(안전망)
            mask = np.abs(Pm[:,2] - SLICE_Z) <= SLICE_EPS
            xy = Pm[mask, :2]
            if xy.shape[0] >= 3:
                hull = hull2d_monotone_chain(xy)
                if hull is not None and hull.shape[0] >= 3:
                    polys_xy.append(hull)
            continue

        N0 = np.array([[n.x, n.y, n.z] for n in nms], dtype=float)
        Nm = (R @ N0.T).T  # normals → TARGET_FRAME

        # 큰 바운딩 박스에서 시작해서 각 면의 2D 반평면으로 클리핑
        xmin = float(np.min(Pm[:,0]) - 50.0); xmax = float(np.max(Pm[:,0]) + 50.0)
        ymin = float(np.min(Pm[:,1]) - 50.0); ymax = float(np.max(Pm[:,1]) + 50.0)
        poly_xy = [np.array([xmin,ymin]), np.array([xmax,ymin]), np.array([xmax,ymax]), np.array([xmin,ymax])]

        # 면식: n·x <= n·p0  (z=SLICE_Z에 투영) → nx x + ny y <= (n·p0 - nz*SLICE_Z)
        for k in range(Pm.shape[0]):
            nx, ny, nz = Nm[k]
            d3 = float(np.dot(Nm[k], Pm[k]))
            c2d = d3 - nz*SLICE_Z
            poly_xy = clip_with_halfplane(poly_xy, nx, ny, c2d)
            if len(poly_xy) < 3:
                break
        if len(poly_xy) >= 3:
            polys_xy.append(np.array(poly_xy, dtype=float))
    return polys_xy

# -------------------- 로딩 --------------------
storage = defaultdict(list)
t_query = None

if MODE.lower() == 'ros1':
    import rosbag
    bag = rosbag.Bag(BAG_PATH, 'r')
    t_start = bag.get_start_time(); t_end = bag.get_end_time()
    t_query = ABS_TIME_SEC if USE_ABS_TIME else min(t_start + T_OFFSET_SEC, t_end)
    topics = [TOPIC_OBS_LIST, CORRIDOR_T, CORRIDOR_R] + TF_TOPICS
    topics += FEAS_TOPICS + RAW_TOPICS + BEST_TOPICS + TARGET_BEST_TOPIC + ROBOT_POSE_TOPICS + TARGET_POSE_TOPICS
    for topic, msg, t in bag.read_messages(topics=list(set(topics))):
        storage[topic].append((t.to_sec(), msg))
    bag.close()
else:
    from rosbags.highlevel import AnyReader
    with AnyReader([BAG_PATH]) as reader:
        times = []
        for conn in reader.connections:
            for _, ts, _ in reader.messages(connections=[conn]): times.append(ts); break
        if not times: raise RuntimeError('빈 rosbag2 입니다.')
        t_start = min(times)*1e-9; t_end = max(times)*1e-9
        t_query = ABS_TIME_SEC if USE_ABS_TIME else min(t_start + T_OFFSET_SEC, t_end)
        wanted = [TOPIC_OBS_LIST, CORRIDOR_T, CORRIDOR_R] + TF_TOPICS + FEAS_TOPICS + RAW_TOPICS + BEST_TOPICS + TARGET_BEST_TOPIC + ROBOT_POSE_TOPICS + TARGET_POSE_TOPICS
        conns = [c for c in reader.connections if c.topic in set(wanted)]
        for conn in conns:
            for _, ts, raw in reader.messages(connections=[conn]):
                msg = reader.deserialize(raw, conn.msgtype)
                storage[conn.topic].append((ts*1e-9, msg))

# -------------------- TF & 스냅샷 파싱 --------------------
tf_store = build_tf_store(storage)

# 장애물
obstacles_xy = np.empty((0,2), dtype=float)
if storage.get(TOPIC_OBS_LIST):
    closest = pick_closest(storage[TOPIC_OBS_LIST], t_query)
    if closest is not None:
        _, m = closest
        pts = []
        for s in getattr(m, 'object_state_list', []):
            if hasattr(s, 'px') and hasattr(s, 'py'): pts.append([float(s.px), float(s.py)])
        obstacles_xy = np.array(pts, dtype=float) if pts else np.empty((0,2), dtype=float)

# Feasible/Raw/Best
feas_segs_xy = []
for tp in FEAS_TOPICS:
    if storage.get(tp):
        closest = pick_closest(storage[tp], t_query)
        if closest is not None:
            _, m = closest
            feas_segs_xy = markerarray_extract_lines_xy_in_target(m, tf_store, t_query)
            break
raw_segs_xy = []
for tp in RAW_TOPICS:
    if storage.get(tp):
        closest = pick_closest(storage[tp], t_query)
        if closest is not None:
            _, m = closest
            raw_segs_xy = markerarray_extract_lines_xy_in_target(m, tf_store, t_query)
            break
best_segs_xy = []
for tp in BEST_TOPICS:
    if storage.get(tp):
        closest = pick_closest(storage[tp], t_query)
        if closest is not None:
            _, m = closest
            best_segs_xy = markerarray_extract_lines_xy_in_target(m, tf_store, t_query)
            break
# 추가: Target best trajectory
target_best_segs_xy = []
for tp in TARGET_BEST_TOPIC:
    if storage.get(tp):
        closest = pick_closest(storage[tp], t_query)
        if closest is not None:
            _, m = closest
            target_best_segs_xy = marker_or_array_extract_lines_xy_in_target(m, tf_store, t_query)
            break

# -------- 하이브리드 스위치: 코리더는 H-rep 클리핑 함수 사용 --------
corridorT_polys = []
if storage.get(CORRIDOR_T):
    _, m = pick_closest(storage[CORRIDOR_T], t_query)
    corridorT_polys = polyhedronarray_slice_xy_in_target(m, tf_store, t_query)

corridorR_polys = []
if storage.get(CORRIDOR_R):
    _, m = pick_closest(storage[CORRIDOR_R], t_query)
    corridorR_polys = polyhedronarray_slice_xy_in_target(m, tf_store, t_query)

# 로봇/타겟 현재 포즈
robot_pose_xy = None
for tp in ROBOT_POSE_TOPICS:
    if storage.get(tp):
        closest = pick_closest(storage[tp], t_query)
        if closest is not None:
            t_r, m = closest
            p, hdr = pose_xyz_from_any(m)
            if p is not None:
                src = getattr(hdr, 'frame_id', TARGET_FRAME) if hdr is not None else TARGET_FRAME
                R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t_r)
                robot_pose_xy = (R @ p.reshape(3,1)).reshape(3,)[:2] + tt[:2]
                break

target_pose_xy = None
for tp in TARGET_POSE_TOPICS:
    if storage.get(tp):
        closest = pick_closest(storage[tp], t_query)
        if closest is not None:
            t_t, m = closest
            p, hdr = pose_xyz_from_any(m)
            if p is not None:
                src = getattr(hdr, 'frame_id', TARGET_FRAME) if hdr is not None else TARGET_FRAME
                R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t_t)
                target_pose_xy = (R @ p.reshape(3,1)).reshape(3,)[:2] + tt[:2]
                break

# 최근 PATH_WINDOW_SEC 경로
def within_window(seq, t0, dt):
    return [(t, m) for (t, m) in seq if (t0 - dt) <= t <= t0]

robot_path_xy = []
for tp in ROBOT_POSE_TOPICS:
    if storage.get(tp):
        hist = within_window(storage[tp], t_query, PATH_WINDOW_SEC)
        for t, m in hist:
            p, hdr = pose_xyz_from_any(m)
            if p is None: continue
            src = getattr(hdr, 'frame_id', TARGET_FRAME) if hdr is not None else TARGET_FRAME
            R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t)
            pm = (R @ p.reshape(3,1)).reshape(3,) + tt
            robot_path_xy.append(pm[:2])
        if robot_path_xy: break

target_path_xy = []
for tp in TARGET_POSE_TOPICS:
    if storage.get(tp):
        hist = within_window(storage[tp], t_query, PATH_WINDOW_SEC)
        for t, m in hist:
            p, hdr = pose_xyz_from_any(m)
            if p is None: continue
            src = getattr(hdr, 'frame_id', TARGET_FRAME) if hdr is not None else TARGET_FRAME
            R, tt = lookup_tf(tf_store, src, TARGET_FRAME, t)
            pm = (R @ p.reshape(3,1)).reshape(3,) + tt
            target_path_xy.append(pm[:2])
        if target_path_xy: break

dyn_paths_xy = [[] for _ in range(10)]
if storage.get(TOPIC_OBS_LIST):
    hist = within_window(storage[TOPIC_OBS_LIST], t_query, PATH_WINDOW_SEC)
    for t, m in hist:
        lst = getattr(m, 'object_state_list', [])
        for i, s in enumerate(lst[:10]):
            dyn_paths_xy[i].append([float(s.px), float(s.py)])

# -------------------- 플로팅 --------------------
fig = plt.figure(figsize=(10,10))
ax = plt.gca(); ax.set_aspect('equal', adjustable='box')
ax.set_title(f"Snapshot + last {PATH_WINDOW_SEC:.1f}s paths at t={t_query:.2f}s ({TARGET_FRAME})")

# raw (얇은 회색)
for seg in raw_segs_xy or []:
    if seg is not None and len(seg) > 0:
        ax.plot(seg[:,0], seg[:,1], linewidth=0.6, alpha=0.6, color='#808080' , zorder=10)

# feasible (얇은 하늘색)
for seg in feas_segs_xy or []:
    if seg is not None and len(seg) > 0:
        ax.plot(seg[:,0], seg[:,1], linewidth=0.8, alpha=0.7, color='#87CEFA', zorder=12)

# best (진한 파랑)
used = False
for seg in best_segs_xy or []:
    if seg is not None and len(seg) > 0:
        ax.plot(seg[:,0], seg[:,1], linewidth=3.0, alpha=0.95, color='navy', linestyle=(0,(1.0,1.0)), zorder=14, label=("Best trajectory" if not used else None))
        used = True
# target best (진한 빨강)
t_used = False
for seg in target_best_segs_xy or []:
    if seg is not None and len(seg) > 0:
        ax.plot(seg[:,0], seg[:,1], linewidth=3.0, alpha=0.95, color='red', linestyle=(0,(1.0,1.0)), zorder=14, label=("Target best trajectory" if not t_used else None))
        t_used = True

# corridors (면 제약 반영된 정확한 폴리곤)
def draw_polys(ax, polys, label=None, face=None, edge='k', alpha=0.25):
    used = False
    for poly in polys:
        poly = np.asarray(poly)
        if poly.ndim != 2 or poly.shape[1] != 2 or len(poly) < 3:
            continue
        xs = list(poly[:,0]) + [poly[0,0]]
        ys = list(poly[:,1]) + [poly[0,1]]
        ax.fill(xs, ys, facecolor=face, edgecolor=edge, alpha=alpha,
                linewidth=1.5, label=(label if not used else None))
        used = True

draw_polys(ax, corridorT_polys, label="Target corridor",
           face=(1.0, 0.4, 0.4), edge=(0.6, 0.1, 0.1), alpha=0.25)

draw_polys(ax, corridorR_polys, label="Robot corridor",
           face=(0.2, 0.4, 1.0), edge=(0.1, 0.2, 0.5), alpha=0.25)

# 동적 장애물
for i in range(10):
    pts = np.asarray(dyn_paths_xy[i], dtype=float)
    if pts.size > 0:
        ax.plot(pts[:,0], pts[:,1], color='green', linewidth=2.0, alpha=0.9, label=('Dyn path' if i==0 else None))

if obstacles_xy is not None and obstacles_xy.size > 0:
    for x, y in obstacles_xy:
        ax.add_patch(Circle((x,y), CYL_RADIUS, facecolor=(0.0,1.0,0.0,0.35), edgecolor='green', linewidth=2.0))

# 로봇/타겟 경로 + 현재 위치
if robot_path_xy:
    P = np.asarray(robot_path_xy, dtype=float); ax.plot(P[:,0], P[:,1], color='blue', linewidth=3.0, alpha=0.95, label='Robot path')
if target_path_xy:
    P = np.asarray(target_path_xy, dtype=float); ax.plot(P[:,0], P[:,1], color='red', linewidth=3.0, alpha=0.95, label='Target path')

if 'robot_pose_xy' in locals() and robot_pose_xy is not None:
    ax.scatter([robot_pose_xy[0]],[robot_pose_xy[1]], s=90, marker='o', color='blue', edgecolors='k', zorder=5, label='Robot')
if 'target_pose_xy' in locals() and target_pose_xy is not None:
    ax.scatter([target_pose_xy[0]],[target_pose_xy[1]], s=110, marker='*', color='red', edgecolors='k', zorder=5, label='Target')

# ---- Info prints ----
_cnt = lambda segs: sum(1 for s in (segs or []) if s is not None and len(s) > 0)
print(f"[INFO] feasible segments: {_cnt(feas_segs_xy)}")
print(f"[INFO] raw segments: {_cnt(raw_segs_xy)}")
print(f"[INFO] best segments: {_cnt(best_segs_xy)}")
print(f"[INFO] target_best segments: {_cnt(target_best_segs_xy)}")
print(f"[INFO] corridorT polygons: {len(corridorT_polys)} | corridorR polygons: {len(corridorR_polys)}")
print(f"[INFO] obstacles_now (shape): {obstacles_xy.shape if obstacles_xy is not None else (0,0)}")
print(f"[INFO] robot_path_pts(last {PATH_WINDOW_SEC}s): {len(robot_path_xy)} | target_path_pts(last {PATH_WINDOW_SEC}s): {len(target_path_xy)}")
print(f"[INFO] dyn_paths total pts(last {PATH_WINDOW_SEC}s): {sum(len(p) for p in dyn_paths_xy)}")
ax.legend(loc='best'); ax.grid(True, linestyle=':')
plt.xlabel('X [m]'); plt.ylabel('Y [m]')
plt.tight_layout(); plt.show()

# 디버그: 비었을 때 로그
if not feas_segs_xy:
    print("[WARN] feasible segs empty - check topic/time")
if not raw_segs_xy:
    print("[WARN] raw segs empty - check topic/time")
if not best_segs_xy:
    print("[WARN] robot best segs empty - check topic/time")
if not target_best_segs_xy:
    print("[WARN] target best segs empty - check topic/time")
