#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_bags_extended.py  —  ROS 2 (Humble) rosbag2 analyzer (sqlite3)

Patch aplicado:
- Selección de plan "robusto": en vez de usar SIEMPRE el último /plan,
  se elige el plan MÁS LARGO publicado hasta t0+Δ (por defecto Δ=2s). Esto evita
  el sesgo de planes "mini" al final del recorrido y repara métricas como
  global_plan_len_m, efficiency_pos y tracking_*.

Nuevas métricas:
- Control effort (proxies de consumo): integrales L1/L2 de cmd_vel y de aceleración odométrica:
  * cmd_lin_L1, cmd_lin_L2, cmd_ang_L1, cmd_ang_L2
  * acc_lin_L1, acc_lin_L2
- Energía real (si disponible): integra P=V*I desde /diagnostics (o /a200_0000/diagnostics)
  * energy_Wh_pos (hasta llegada), avg_power_W_pos
  Si no encuentra V/I en diagnostics, deja NaN.
- Perfil de velocidad: fracción de tiempo en bins (v<0.05), (0.05..0.20], (>0.20) m/s

Mantiene:
- Llegada por posición (y opcional yaw).
- Alineación ODOM->MAP por Procrustes SE(2) contra el plan más largo cercano a t0.
- Métricas de tracking, eficiencia, suavidad (p95), stops, oscilaciones, clearance, costmap, cmd jitter.

Uso típico:
  source /opt/ros/humble/setup.bash
  python3 analyze_bags_extended.py \
    --results-dir  ~/ros2_ws/src/kevin_nav2_experiments/results \
    --out-csv     ~/ros2_ws/src/kevin_nav2_experiments/results2/metrics_full.csv \
    --out-summary ~/ros2_ws/src/kevin_nav2_experiments/results2/summary_full.csv \
    --fig-dir     ~/ros2_ws/src/kevin_nav2_experiments/results2/figs_metrics \
    --traj-dir    ~/ros2_ws/src/kevin_nav2_experiments/results2/figs_traj \
    --pos-radius 0.50 --pos-hold 0.30 --yaw-deg 15 --yaw-hold 0.20 \
    --stop-speed 0.03 --clearance-safe 0.35 --high-cost 200 \
    --plan-select-window-s 2.0 \
    --controllers DWB RPP MPPI --direction-filter all --debug
"""

import os, re, math, argparse, re as _re
import numpy as np
import pandas as pd
from pathlib import Path

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message

# --- Tópicos esperados (ajustables si cambian) ---
NAMESPACE = "/a200_0000"
TOPICS = {
    "odom":       f"{NAMESPACE}/platform/odom/filtered",
    "plan":       f"{NAMESPACE}/plan",
    "recv_plan":  f"{NAMESPACE}/received_global_plan",
    "costmap":    f"{NAMESPACE}/local_costmap/costmap",
    "scan":       f"{NAMESPACE}/sensors/lidar2d_0/scan",
    "amcl":       f"{NAMESPACE}/amcl_pose",
    "goal":       f"{NAMESPACE}/goal_pose",
    "cmd":        f"{NAMESPACE}/cmd_vel",
    "clock":      "/clock",
    "diag_ns":    f"{NAMESPACE}/diagnostics",
    "diag_root":  "/diagnostics",
}

# --------- helpers geom/arrays ----------
def yaw_from_quat(q):
    x,y,z,w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0*(w*z + x*y)
    cosy_cosp = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(siny_cosp, cosy_cosp)

def ang_diff(a, b):
    e = (a - b + math.pi) % (2*math.pi) - math.pi
    return e

def dist2d(p, q): return math.hypot(p[0]-q[0], p[1]-q[1])

def path_length(points):
    if len(points) < 2: return np.nan
    return float(sum(dist2d(points[i], points[i-1]) for i in range(1, len(points))))

def project_point_to_polyline(poly, p):
    if len(poly) < 2: return (None, -1, 0.0, float("inf"))
    best = (None, -1, 0.0, float("inf"))
    px, py = p
    for i in range(1, len(poly)):
        ax, ay = poly[i-1]; bx, by = poly[i]
        vx, vy = bx-ax, by-ay
        wx, wy = px-ax, py-ay
        v2 = vx*vx + vy*vy
        t = 0.0 if v2 == 0 else max(0.0, min(1.0, (wx*vx + wy*vy)/v2))
        qx, qy = ax + t*vx, ay + t*vy
        d = math.hypot(px-qx, py-qy)
        if d < best[3]:
            best = ((qx, qy), i-1, t, d)
    return best

def polyline_dist_point(poly, p):
    _,_,_,d = project_point_to_polyline(poly, p)
    return d

def unwrap_angle(a): return np.unwrap(a)

def deriv_local(x, t):
    if len(x) < 2: return np.array([])
    dt = np.diff(t).astype(float); dt[dt==0] = 1e-6
    return np.diff(x)/dt

# --------- costmap sampling ----------
def occupancy_at(costmap_msg, x, y):
    info = costmap_msg.info
    yaw = yaw_from_quat(info.origin.orientation)
    if abs(yaw) > 1e-3: return np.nan
    res = info.resolution
    mx = int((x - info.origin.position.x) / res)
    my = int((y - info.origin.position.y) / res)
    if mx<0 or my<0 or mx>=info.width or my>=info.height: return np.nan
    try:
        return int(costmap_msg.data[my*info.width + mx])
    except Exception:
        return np.nan

# --------- lectura bag ----------
def read_bag_messages(bag_path_or_dir, wanted_topics):
    bag_uri = str(bag_path_or_dir)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_uri, storage_id='sqlite3'), ConverterOptions('', ''))
    ttypes = {x.name: x.type for x in reader.get_all_topics_and_types()}
    mcls = {}
    for t in wanted_topics:
        if t in ttypes:
            try: mcls[t] = get_message(ttypes[t])
            except: mcls[t] = None
    out = {t: [] for t in wanted_topics}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in out or topic not in mcls or mcls[topic] is None:
            continue
        try:
            msg = deserialize_message(data, mcls[topic])
            out[topic].append((t*1e-9, msg))
        except Exception:
            continue
    for k in out: out[k].sort(key=lambda x: x[0])
    return out

# --------- llegada (posición / yaw) ----------
def arrival_time_position(times, XY, goal_xy, t0, radius, hold):
    if times.size == 0: return np.nan, np.nan
    i0 = np.searchsorted(times, t0, side='left')
    win, last_t, dmin = 0.0, None, float("inf")
    for i in range(i0, len(times)):
        x,y = XY[i]; d = dist2d((x,y), goal_xy); dmin = min(dmin, d)
        ti = times[i]
        if last_t is None: last_t = ti
        dt = max(0.0, ti - last_t); last_t = ti
        if d < radius:
            win += dt
            if win >= hold:
                return ti - t0 - (win - hold), dmin
        else:
            win = 0.0
    return np.nan, dmin

def arrival_time_yaw(times, yaw_arr, goal_yaw, t0, yaw_radius_rad, hold):
    if times.size == 0 or yaw_arr.size == 0: return np.nan
    i0 = np.searchsorted(times, t0, side='left')
    win, last_t = 0.0, None
    for i in range(i0, len(times)):
        e = abs(ang_diff(yaw_arr[i], goal_yaw))
        ti = times[i]
        if last_t is None: last_t = ti
        dt = max(0.0, ti - last_t); last_t = ti
        if e < yaw_radius_rad:
            win += dt
            if win >= hold:
                return ti - t0 - (win - hold)
        else:
            win = 0.0
    return np.nan

# --------- SE(2) alignment ODOM->MAP contra plan ----------
def estimate_se2_odom_to_map(odom_xy, odom_t, global_plan, t_fit=3.0, d_prune=0.75):
    if len(odom_xy) < 3 or len(global_plan) < 2:
        return (np.eye(2), np.zeros(2), 'identity', 0)
    t0 = odom_t[0]
    mask = odom_t <= (t0 + t_fit)
    A = odom_xy[mask]
    if len(A) < 3:
        A = odom_xy[:min(50, len(odom_xy))]
    if len(A) < 3:
        return (np.eye(2), np.zeros(2), 'identity', 0)
    B_list, A_list = [], []
    for p in A:
        q, _, _, d = project_point_to_polyline(global_plan, tuple(p))
        if q is None: continue
        if d <= d_prune:
            A_list.append(p); B_list.append(q)
    if len(A_list) < 3:
        R = np.eye(2)
        t = np.array(global_plan[0]) - np.array(odom_xy[0])
        return (R, t, 'trans', len(A_list))
    A = np.array(A_list); B = np.array(B_list)
    muA = A.mean(axis=0); muB = B.mean(axis=0)
    A0 = A - muA; B0 = B - muB
    H = A0.T @ B0
    U, S, Vt = np.linalg.svd(H)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = U @ Vt
    t = muB - R @ muA
    return (R, t, 'se2', len(A_list))

def apply_se2(R, t, pts):
    return (R @ pts.T).T + t

# --------- selección de plan (patch) ----------
def extract_plan_xy(plan_msgs, recv_plan_msgs, t0, window_s=2.0):
    """
    Elige el plan más largo publicado hasta t0+window_s.
    Considera tanto /plan como /received_global_plan.
    """
    cands = []
    for arr in (plan_msgs, recv_plan_msgs):
        for ts, msg in arr:
            if not np.isnan(t0) and ts > t0 + window_s:
                continue
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            L = path_length(pts)
            if L == L:  # not NaN
                cands.append((L, pts))
    if not cands:
        # como fallback, usa el más largo global
        for arr in (plan_msgs, recv_plan_msgs):
            for ts, msg in arr:
                pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
                L = path_length(pts)
                if L == L:
                    cands.append((L, pts))
    if not cands:
        return []
    cands.sort(key=lambda x: x[0], reverse=True)
    return cands[0][1]

# --------- parseo de diagnostics para energía ----------
def parse_voltage_current_from_diag(diag_msg):
    """
    Intenta extraer (V, A) desde un DiagnosticArray.
    Busca en las key-values de cada status patrones como 'volt', 'current'.
    Devuelve (V, A) como floats o NaN si no se encuentran.
    """
    volt = np.nan; amp = np.nan
    try:
        for st in diag_msg.status:
            for kv in st.values:
                k = kv.key.lower()
                v = kv.value
                # extraer número de '12.3 V', '3.2A', etc.
                m = _re.search(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", v)
                if not m: 
                    continue
                val = float(m.group(0))
                if "volt" in k or k.endswith("(v)") or k.endswith(" v") or k == "v":
                    volt = val
                if "curr" in k or "amp" in k or k.endswith("(a)") or k.endswith(" a") or k == "a":
                    amp = val
    except Exception:
        pass
    return volt, amp

# --------- análisis por bag ----------
def compute_metrics_for_bag(bag_path, cfg):
    wanted = [TOPICS[k] for k in ["odom","plan","recv_plan","costmap","scan","amcl","goal","cmd","clock","diag_ns","diag_root"]]
    msgs = read_bag_messages(bag_path, wanted)

    odom = msgs.get(TOPICS["odom"], [])
    amcl = msgs.get(TOPICS["amcl"], [])
    plan_msgs = msgs.get(TOPICS["plan"], [])
    recv_plan_msgs = msgs.get(TOPICS["recv_plan"], [])
    costmap_msgs = msgs.get(TOPICS["costmap"], [])
    scan_msgs = msgs.get(TOPICS["scan"], [])
    goal_msgs = msgs.get(TOPICS["goal"], [])
    cmd_msgs = msgs.get(TOPICS["cmd"], [])
    clock_msgs = msgs.get(TOPICS["clock"], [])
    diag_msgs = msgs.get(TOPICS["diag_ns"], []) + msgs.get(TOPICS["diag_root"], [])

    # timeline
    all_ts = []
    for arr in [odom, amcl, plan_msgs, recv_plan_msgs, costmap_msgs, scan_msgs, goal_msgs, cmd_msgs, clock_msgs, diag_msgs]:
        if arr: all_ts.extend([t for (t, _) in arr])
    if not all_ts:
        return None
    t_min, t_max = min(all_ts), max(all_ts)
    duration_total = t_max - t_min if t_max >= t_min else np.nan

    # goal (map)
    gx = gy = gyaw = np.nan
    t_goal = np.nan
    if goal_msgs:
        t_goal = goal_msgs[0][0]
        gpose = goal_msgs[0][1].pose
        gx, gy = gpose.position.x, gpose.position.y
        gyaw = yaw_from_quat(gpose.orientation)

    # odometría
    od_t = np.array([t for (t, _) in odom]) if odom else np.array([])
    od_x = np.array([m.pose.pose.position.x for (_, m) in odom]) if odom else np.array([])
    od_y = np.array([m.pose.pose.position.y for (_, m) in odom]) if odom else np.array([])
    od_xy = np.stack([od_x, od_y], axis=1) if od_x.size else np.empty((0,2))
    od_yaw = np.array([yaw_from_quat(m.pose.pose.orientation) for (_, m) in odom]) if odom else np.array([])
    od_vx  = np.array([m.twist.twist.linear.x for (_, m) in odom]) if odom else np.array([])
    od_wz  = np.array([m.twist.twist.angular.z for (_, m) in odom]) if odom else np.array([])

    # origen temporal preferido
    t0 = t_goal if not np.isnan(t_goal) else (od_t[0] if od_t.size else t_min)

    # plan global (map) con patch
    global_plan = extract_plan_xy(plan_msgs, recv_plan_msgs, t0, window_s=cfg["plan_select_window_s"])

    # Fuente para detectar llegada (preferir AMCL)
    pos_source = "unknown"
    times_det = np.array([]); XY_det = np.empty((0,2)); yaw_det = np.array([])
    if amcl:
        pos_source = "amcl"
        amcl_t = np.array([t for (t, _) in amcl])
        amcl_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for (_, m) in amcl])
        amcl_yaw = np.array([yaw_from_quat(m.pose.pose.orientation) for (_, m) in amcl])
        times_det, XY_det, yaw_det = amcl_t, amcl_xy, amcl_yaw
    elif len(global_plan) >= 2 and od_t.size >= 3:
        R, t, mode, n_pairs = estimate_se2_odom_to_map(od_xy, od_t, global_plan, t_fit=cfg["align_fit_s"], d_prune=cfg["align_prune_m"])
        XY_map_est = apply_se2(R, t, od_xy)
        times_det, XY_det = od_t, XY_map_est
        yaw_det = od_yaw.copy()
        pos_source = "odom_aligned" if mode == 'se2' else ("odom_trans" if mode == 'trans' else "odom")
    elif od_t.size:
        times_det, XY_det = od_t, od_xy
        yaw_det = od_yaw.copy()
        pos_source = "odom"

    # Llegada
    goal_xy = (gx, gy)
    t_pos_s, dmin = arrival_time_position(times_det, XY_det, goal_xy, t0, cfg["pos_radius"], cfg["pos_hold"])
    success_pos = int(not np.isnan(t_pos_s))
    yaw_radius_rad = math.radians(cfg["yaw_deg"])
    t_yaw_s = arrival_time_yaw(times_det, yaw_det, gyaw, t0, yaw_radius_rad, cfg["yaw_hold"]) if yaw_det.size else np.nan
    success_yaw = int(not np.isnan(t_yaw_s))
    success_posyaw = 1 if (success_pos and success_yaw) else 0
    t_arrival_used = (t_pos_s if not cfg["require_yaw"] else (max(t_pos_s, t_yaw_s) if success_posyaw else np.nan))
    success_used = success_pos if not cfg["require_yaw"] else success_posyaw

    # Cortes temporales
    t_cut = (t0 + t_pos_s) if success_pos else (od_t[-1] if od_t.size else t_max)

    # Cortar señales hasta llegada
    def cut(sig_t, sig, t_end):
        if sig_t.size == 0: return sig_t, sig
        idx = sig_t <= t_end
        return sig_t[idx], sig[idx] if hasattr(sig, "__len__") else sig

    od_t_pos, od_xy_pos = cut(od_t, od_xy, t_cut)
    od_vx_pos = od_vx[od_t <= t_cut] if od_vx.size else np.array([])
    od_wz_pos = od_wz[od_t <= t_cut] if od_wz.size else np.array([])

    # Longitud recorrida hasta pos
    odom_path_len_pos = path_length(od_xy_pos.tolist()) if len(od_xy_pos) >= 2 else 0.0

    # Plan tracking/eficiencia (con plan patch)
    global_plan_len = path_length(global_plan) if global_plan else np.nan
    efficiency_pos = tracking_rms_pos = tracking_p95_pos = tracking_max_pos = outside_tol_area = np.nan
    if global_plan and times_det.size:
        idx_det = times_det <= t_cut
        XY_det_pos = XY_det[idx_det]
        if len(XY_det_pos) >= 2:
            eff_len = path_length(XY_det_pos.tolist())
            efficiency_pos = (eff_len / global_plan_len) if (global_plan_len and global_plan_len > 0) else np.nan
            dists = np.array([polyline_dist_point(global_plan, (x, y)) for (x, y) in XY_det_pos.tolist()], dtype=float)
            tracking_rms_pos = float(np.sqrt(np.nanmean(np.square(dists))))
            tracking_p95_pos = float(np.nanpercentile(dists, 95))
            tracking_max_pos = float(np.nanmax(dists))
            tol = cfg["track_tol_m"]
            if not np.isnan(tol) and tol > 0:
                ts = times_det[idx_det]
                excess = np.maximum(0.0, dists - tol)
                dt = np.diff(ts)
                if dt.size and excess.size >= 2:
                    excess_mid = 0.5*(excess[:-1] + excess[1:])
                    outside_tol_area = float(np.sum(excess_mid * dt))

    # Stops/osc
    stops_time_pos = 0.0; stops_count = 0
    if od_vx_pos.size > 1:
        speed = np.abs(od_vx_pos)
        stopped = speed < cfg["stop_speed"]
        if stopped.size > 1:
            dt = np.diff(od_t_pos); dt = np.maximum(dt, 0.0)
            stops_time_pos = float(np.sum(dt * stopped[1:]))
            shifted = np.concatenate([[False], stopped[:-1]])
            starts = np.logical_and(stopped, ~shifted)
            stops_count = int(np.sum(starts))

    osc_count_pos = 0
    if od_wz_pos.size > 1:
        s = np.sign(od_wz_pos)
        osc_count_pos = int(np.sum((s[1:]*s[:-1]) < 0))

    # Aceleración y jerk
    a_lin_pos = deriv_local(od_vx_pos, od_t_pos)
    j_lin_pos = deriv_local(a_lin_pos, od_t_pos[1:]) if len(od_t_pos) >= 3 else np.array([])

    # Estadísticas robustas (p95, rms)
    def safe_stats(arr):
        if arr.size == 0: return (np.nan, np.nan, np.nan)
        return (float(np.nanmax(np.abs(arr))), float(np.nanpercentile(np.abs(arr), 95)), float(np.sqrt(np.nanmean(np.square(arr)))))
    v_lin_max_pos, v_lin_p95_pos, v_lin_rms_pos = safe_stats(od_vx_pos)
    v_ang_max_pos, v_ang_p95_pos, v_ang_rms_pos = safe_stats(od_wz_pos)
    acc_lin_max_pos, acc_lin_p95_pos, acc_lin_rms_pos = safe_stats(a_lin_pos)
    jerk_lin_max_pos, jerk_lin_p95_pos, jerk_lin_rms_pos = safe_stats(j_lin_pos)

    # Reversa y convergencia
    backwards_ratio = float(np.mean(od_vx_pos < 0.0)) if od_vx_pos.size else np.nan
    dist_goal_t = np.array([dist2d(p, goal_xy) for p in XY_det]) if times_det.size else np.array([])
    re_away_count = 0
    if dist_goal_t.size > 2:
        trend = np.diff(dist_goal_t)
        re_away_count = int(np.sum((trend[1:] > 0) & (trend[:-1] < 0)))
    final_dist = float(dist_goal_t[-1]) if dist_goal_t.size else np.nan
    min_dist = float(np.nanmin(dist_goal_t)) if dist_goal_t.size else np.nan

    # Clearance LIDAR hasta t_cut
    min_scan_pos = p05_scan_pos = time_under_safe = np.nan
    if scan_msgs:
        mins = []; ts_scan = []
        for ts, s in scan_msgs:
            if ts > t_cut: break
            arr = np.array(s.ranges, dtype=float); arr = arr[np.isfinite(arr)]
            if arr.size:
                mins.append(np.nanmin(arr)); ts_scan.append(ts)
        if mins:
            mins = np.array(mins, dtype=float)
            min_scan_pos = float(np.nanmin(mins))
            p05_scan_pos = float(np.nanpercentile(mins, 5))
            if len(ts_scan) > 1:
                dt = np.diff(ts_scan); dt = np.maximum(dt, 0.0)
                under = mins[1:] < cfg["clearance_safe"]
                time_under_safe = float(np.sum(dt * under))

    # Costmap bajo el robot hasta t_cut
    cost_min = cost_mean = cost_max = time_high_cost = np.nan
    if costmap_msgs and od_xy_pos.shape[0] > 0:
        cm_by_t = [t for (t, _) in costmap_msgs]; cm_msgs = [m for (_, m) in costmap_msgs]
        costs = []; j = 0
        for ti, (x, y) in zip(od_t_pos, od_xy_pos):
            while j+1 < len(cm_by_t) and cm_by_t[j+1] <= ti: j += 1
            val = occupancy_at(cm_msgs[j], float(x), float(y))
            if not math.isnan(val): costs.append(val)
        if costs:
            costs = np.array(costs, dtype=float)
            cost_min = float(np.min(costs)); cost_mean = float(np.mean(costs)); cost_max = float(np.max(costs))
            time_high_cost = float(np.mean(costs >= cfg["high_cost"])) * (od_t_pos[-1]-od_t_pos[0]) if len(od_t_pos)>=2 else np.nan

    # cmd_vel métricas + control effort
    cmd_rate_hz = cmd_jitter_p95 = dv_p95 = dw_p95 = np.nan
    cmd_lin_L1 = cmd_lin_L2 = cmd_ang_L1 = cmd_ang_L2 = np.nan
    if cmd_msgs:
        tc = np.array([t for (t, _) in cmd_msgs])
        vx = np.array([m.linear.x for (_, m) in cmd_msgs], dtype=float)
        wz = np.array([m.angular.z for (_, m) in cmd_msgs], dtype=float)
        # recortar a t_cut
        idxc = tc <= t_cut
        tc = tc[idxc]; vx = vx[idxc]; wz = wz[idxc]
        if tc.size >= 2:
            dt = np.diff(tc)
            mean_dt = np.mean(dt) if np.all(dt>0) else np.nan
            cmd_rate_hz = (1.0/mean_dt) if mean_dt and mean_dt>0 else (len(tc)/(tc[-1]-tc[0]) if (tc[-1]>tc[0]) else np.nan)
            cmd_jitter_p95 = float(np.nanpercentile(np.abs(dt - np.nanmedian(dt)), 95))
            dv = np.diff(vx)/np.maximum(dt, 1e-6)
            dw = np.diff(wz)/np.maximum(dt, 1e-6)
            dv_p95 = float(np.nanpercentile(np.abs(dv), 95))
            dw_p95 = float(np.nanpercentile(np.abs(dw), 95))
            # esfuerzos L1/L2 (proxies energía de control)
            cmd_lin_L1 = float(np.sum(0.5*(np.abs(vx[:-1])+np.abs(vx[1:]))*dt))
            cmd_ang_L1 = float(np.sum(0.5*(np.abs(wz[:-1])+np.abs(wz[1:]))*dt))
            cmd_lin_L2 = float(np.sum(0.5*((vx[:-1]**2)+(vx[1:]**2))*dt))
            cmd_ang_L2 = float(np.sum(0.5*((wz[:-1]**2)+(wz[1:]**2))*dt))

    # Esfuerzo por aceleración (odom)
    acc_lin_L1 = acc_lin_L2 = np.nan
    if a_lin_pos.size > 0 and od_t_pos.size > 1:
        dt = np.diff(od_t_pos)
        if a_lin_pos.size == dt.size:
            acc_lin_L1 = float(np.sum(np.abs(a_lin_pos)*dt))
            acc_lin_L2 = float(np.sum((a_lin_pos**2)*dt))
        else:
            n = min(len(a_lin_pos), len(dt))
            acc_lin_L1 = float(np.sum(np.abs(a_lin_pos[:n])*dt[:n]))
            acc_lin_L2 = float(np.sum((a_lin_pos[:n]**2)*dt[:n]))

    # Perfil de velocidad (bins)
    v_time_slow = v_time_mid = v_time_fast = np.nan
    if od_vx_pos.size > 1:
        dt = np.diff(od_t_pos); dt = np.maximum(dt, 0.0)
        v_mid = 0.5*(od_vx_pos[:-1]+od_vx_pos[1:])
        abs_v = np.abs(v_mid)
        v_time_slow = float(np.sum(dt[abs_v < 0.05]))
        v_time_mid  = float(np.sum(dt[(abs_v >= 0.05) & (abs_v <= 0.20)]))
        v_time_fast = float(np.sum(dt[abs_v > 0.20]))

    # Energía desde diagnostics (V*I)
    energy_Wh_pos = avg_power_W_pos = np.nan
    if diag_msgs:
        ts = []; P = []
        for t, d in diag_msgs:
            if t > t_cut: break
            V, A = parse_voltage_current_from_diag(d)
            if V==V and A==A:
                ts.append(t); P.append(V*A)
        if len(ts) > 1:
            ts = np.array(ts); P = np.array(P, dtype=float)
            dt = np.diff(ts); dt = np.maximum(dt, 0.0)
            Pmid = 0.5*(P[:-1]+P[1:])
            E_J = float(np.sum(Pmid*dt))
            energy_Wh_pos = E_J / 3600.0
            avg_power_W_pos = float(np.mean(P))

    row = {
        "bag_path": str(bag_path),
        "pos_source": pos_source,
        "duration_total_s": float(duration_total),
        "t0": float(t0) if not np.isnan(t0) else np.nan,
        "goal_x": gx, "goal_y": gy, "goal_yaw": gyaw,
        "success_pos": int(success_pos),
        "t_pos_s": float(t_pos_s) if success_pos else np.nan,
        "success_yaw": int(success_yaw),
        "t_yaw_s": float(t_yaw_s) if success_yaw else np.nan,
        "require_yaw": int(cfg["require_yaw"]),
        "success_used": int(success_used),
        "t_arrival_used_s": float(t_arrival_used) if success_used else np.nan,
        "odom_path_len_pos_m": float(odom_path_len_pos),
        "global_plan_len_m": float(global_plan_len) if global_plan_len==global_plan_len else np.nan,
        "efficiency_pos": float(efficiency_pos) if efficiency_pos==efficiency_pos else np.nan,
        "tracking_rms_pos_m": float(tracking_rms_pos) if tracking_rms_pos==tracking_rms_pos else np.nan,
        "tracking_p95_pos_m": float(tracking_p95_pos) if tracking_p95_pos==tracking_p95_pos else np.nan,
        "tracking_max_pos_m": float(tracking_max_pos) if tracking_max_pos==tracking_max_pos else np.nan,
        "outside_tol_area_m_s": float(outside_tol_area) if outside_tol_area==outside_tol_area else np.nan,
        "stops_time_pos_s": float(stops_time_pos),
        "stops_count": int(stops_count),
        "osc_count_pos": int(osc_count_pos),
        "v_lin_max_pos": float(v_lin_max_pos), "v_lin_p95_pos": float(v_lin_p95_pos), "v_lin_rms_pos": float(v_lin_rms_pos),
        "v_ang_max_pos": float(v_ang_max_pos), "v_ang_p95_pos": float(v_ang_p95_pos), "v_ang_rms_pos": float(v_ang_rms_pos),
        "acc_lin_max_pos": float(acc_lin_max_pos), "acc_lin_p95_pos": float(acc_lin_p95_pos), "acc_lin_rms_pos": float(acc_lin_rms_pos),
        "jerk_lin_max_pos": float(jerk_lin_max_pos), "jerk_lin_p95_pos": float(jerk_lin_p95_pos), "jerk_lin_rms_pos": float(jerk_lin_rms_pos),
        "backwards_ratio": float(backwards_ratio) if backwards_ratio==backwards_ratio else np.nan,
        "re_away_count": int(re_away_count), "final_dist_m": float(final_dist), "min_dist_m": float(min_dist),
        "min_scan_pos_m": float(min_scan_pos) if min_scan_pos==min_scan_pos else np.nan,
        "p05_scan_pos_m": float(p05_scan_pos) if p05_scan_pos==p05_scan_pos else np.nan,
        "time_under_clearance_s": float(time_under_safe) if time_under_safe==time_under_safe else np.nan,
        "costmap_cost_min_pos": float(cost_min) if cost_min==cost_min else np.nan,
        "costmap_cost_mean_pos": float(cost_mean) if cost_mean==cost_mean else np.nan,
        "costmap_cost_max_pos": float(cost_max) if cost_max==cost_max else np.nan,
        "time_high_cost_s": float(time_high_cost) if time_high_cost==time_high_cost else np.nan,
        "cmd_rate_hz": float(cmd_rate_hz) if cmd_rate_hz==cmd_rate_hz else np.nan,
        "cmd_jitter_p95_s": float(cmd_jitter_p95) if cmd_jitter_p95==cmd_jitter_p95 else np.nan,
        "cmd_dv_p95_mps2": float(dv_p95) if dv_p95==dv_p95 else np.nan,
        "cmd_dw_p95_radps2": float(dw_p95) if dw_p95==dw_p95 else np.nan,
        # Proxies de esfuerzo/energía
        "cmd_lin_L1": cmd_lin_L1, "cmd_lin_L2": cmd_lin_L2,
        "cmd_ang_L1": cmd_ang_L1, "cmd_ang_L2": cmd_ang_L2,
        "acc_lin_L1": acc_lin_L1, "acc_lin_L2": acc_lin_L2,
        "v_time_slow_s": v_time_slow, "v_time_mid_s": v_time_mid, "v_time_fast_s": v_time_fast,
        "energy_Wh_pos": energy_Wh_pos, "avg_power_W_pos": avg_power_W_pos,
    }
    return row

# --------- utilidades ----------
def parse_bag_meta(path_like: Path):
    name = path_like.name.lower()
    m = re.match(r"([a-z0-9]+)_(forward|return)_(\d+)_", name)
    if m: return m.group(1).upper(), m.group(2), int(m.group(3))
    if path_like.parent and path_like.parent != path_like:
        return parse_bag_meta(path_like.parent)
    return None, None, None

def find_bag_paths(results_dir: Path):
    out = []
    for ctrl_dir in results_dir.iterdir():
        if not ctrl_dir.is_dir(): continue
        for sub in ctrl_dir.iterdir():
            if sub.is_dir() and ("_forward_" in sub.name or "_return_" in sub.name):
                db3s = list(sub.glob("*.db3"))
                if db3s: out.append(sub)
    return sorted(out, key=lambda p: p.name)

def summarize_group(df, by_cols):
    if df.empty: return pd.DataFrame()
    def iqr(x): return np.nanpercentile(x, 75) - np.nanpercentile(x, 25)
    agg = {
        "success_used": ["mean","sum","count"],
        "t_arrival_used_s": ["median","mean","std","min","max", iqr],
        "t_pos_s": ["median","mean","std", iqr],
        "odom_path_len_pos_m": ["median","mean","std", iqr],
        "global_plan_len_m": ["median","mean","std", iqr],
        "efficiency_pos": ["median","mean","std", iqr],
        "tracking_rms_pos_m": ["median","mean","std", iqr],
        "tracking_p95_pos_m": ["median","mean","std", iqr],
        "stops_time_pos_s": ["median","mean","std", iqr],
        "stops_count": ["median","mean","std", iqr],
        "osc_count_pos": ["median","mean","std", iqr],
        "v_lin_p95_pos": ["median","mean","std", iqr],
        "v_ang_p95_pos": ["median","mean","std", iqr],
        "acc_lin_p95_pos": ["median","mean","std", iqr],
        "jerk_lin_p95_pos": ["median","mean","std", iqr],
        "p05_scan_pos_m": ["median","mean","std", iqr],
        "time_under_clearance_s": ["median","mean","std", iqr],
        "costmap_cost_mean_pos": ["median","mean","std", iqr],
        "time_high_cost_s": ["median","mean","std", iqr],
        "backwards_ratio": ["median","mean","std", iqr],
        "re_away_count": ["median","mean","std", iqr],
        "cmd_rate_hz": ["median","mean","std", iqr],
        "cmd_jitter_p95_s": ["median","mean","std", iqr],
        # nuevos
        "cmd_lin_L1": ["median","mean","std", iqr],
        "cmd_lin_L2": ["median","mean","std", iqr],
        "cmd_ang_L1": ["median","mean","std", iqr],
        "cmd_ang_L2": ["median","mean","std", iqr],
        "acc_lin_L1": ["median","mean","std", iqr],
        "acc_lin_L2": ["median","mean","std", iqr],
        "v_time_slow_s": ["median","mean","std", iqr],
        "v_time_mid_s": ["median","mean","std", iqr],
        "v_time_fast_s": ["median","mean","std", iqr],
        "energy_Wh_pos": ["median","mean","std", iqr],
        "avg_power_W_pos": ["median","mean","std", iqr],
    }
    present = {k:v for k,v in agg.items() if k in df.columns}
    g = df.groupby(by_cols).agg(present)
    g.columns = ["%s_%s"%(c[0], c[1] if isinstance(c, tuple) else "") for c in g.columns.values]
    g = g.reset_index()
    if "success_used_mean" in g.columns:
        g = g.rename(columns={"success_used_mean": "success_rate"})
    return g

# --------- plots ----------
def plot_bar_with_points(df, metric, ylabel, figpath):
    if df.empty or metric not in df.columns: return
    d = df.dropna(subset=[metric])
    if d.empty: return
    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as _np
    except Exception as e:
        print(f"[WARN] matplotlib ausente para {figpath.name}: {e}")
        return
    ctrls = sorted(d['controller'].unique())
    meds = [_np.nanmedian(d.loc[d.controller==c, metric]) for c in ctrls]
    fig, ax = plt.subplots(figsize=(8,5))
    x = _np.arange(len(ctrls)); ax.bar(x, meds)
    for i,c in enumerate(ctrls):
        ys = d.loc[d.controller==c, metric].values
        xs = _np.random.uniform(i-0.15, i+0.15, size=len(ys))
        ax.scatter(xs, ys, s=18, alpha=0.9)
    ax.set_xticks(x); ax.set_xticklabels(ctrls)
    ax.set_ylabel(ylabel); ax.set_title(f"{metric} por controlador (mediana + puntos)")
    ax.grid(True, axis='y', alpha=0.3); fig.tight_layout(); fig.savefig(figpath, dpi=150); plt.close(fig)

def make_all_plots(df, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    items = [
        ("t_arrival_used_s",       "Tiempo a llegada usada (s)"),
        ("odom_path_len_pos_m",    "Longitud odom hasta pos (m)"),
        ("global_plan_len_m",      "Longitud plan (m)"),
        ("efficiency_pos",         "Eficiencia (odom/plan)"),
        ("tracking_rms_pos_m",     "RMS seguimiento plan (m)"),
        ("stops_time_pos_s",       "Tiempo detenido (s)"),
        ("osc_count_pos",          "Oscilaciones (ω)"),
        ("v_lin_p95_pos",          "Velocidad lin p95 (m/s)"),
        ("acc_lin_p95_pos",        "Aceleración lin p95 (m/s²)"),
        ("jerk_lin_p95_pos",       "Jerk lin p95 (m/s³)"),
        ("p05_scan_pos_m",         "Clearance LIDAR p05 (m)"),
        ("costmap_cost_mean_pos",  "Costmap medio (0..255)"),
        ("time_high_cost_s",       "Tiempo en alto costo (s)"),
        # proxies energía/control
        ("cmd_lin_L2",             "Esfuerzo cmd lin (∫v²dt)"),
        ("cmd_ang_L2",             "Esfuerzo cmd ang (∫ω²dt)"),
        ("acc_lin_L2",             "Esfuerzo acc lin (∫a²dt)"),
        ("energy_Wh_pos",          "Energía (Wh) hasta llegada"),
    ]
    for metric, ylabel in items:
        if metric in df.columns:
            plot_bar_with_points(df, metric, ylabel, out_dir / f"{metric}.png")

# --------- trayectorias ----------
def extract_traj_for_plot(bag_path: Path, cfg):
    controller, direction, trial = parse_bag_meta(bag_path)
    wanted = [TOPICS[k] for k in ["odom","amcl","plan","recv_plan","goal"]]
    msgs = read_bag_messages(bag_path, wanted)
    odom = msgs.get(TOPICS["odom"], [])
    amcl = msgs.get(TOPICS["amcl"], [])
    plan_msgs = msgs.get(TOPICS["plan"], [])
    recv_plan_msgs = msgs.get(TOPICS["recv_plan"], [])
    goal_msgs = msgs.get(TOPICS["goal"], [])

    goal_xy = None
    if goal_msgs:
        gpose = goal_msgs[0][1].pose
        goal_xy = (gpose.position.x, gpose.position.y)

    # usar la misma lógica de plan robusto
    t0 = goal_msgs[0][0] if goal_msgs else (odom[0][0] if odom else np.nan)
    plan_xy = extract_plan_xy(plan_msgs, recv_plan_msgs, t0, window_s=cfg["plan_select_window_s"])

    if amcl:
        src = "amcl"
        amcl_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for (_, m) in amcl])
        path_xy = amcl_xy
    else:
        od_t = np.array([t for (t, _) in odom]) if odom else np.array([])
        od_xy = np.array([[m.pose.pose.position.x, m.pose.pose.position.y] for (_, m) in odom]) if odom else np.empty((0,2))
        if len(plan_xy) >= 2 and od_t.size >= 3 and od_xy.shape[0] >= 3:
            R, t, mode, _ = estimate_se2_odom_to_map(od_xy, od_t, plan_xy, t_fit=cfg["align_fit_s"], d_prune=cfg["align_prune_m"])
            path_xy = apply_se2(R, t, od_xy)
            src = "odom_aligned" if mode == 'se2' else ("odom_trans" if mode == 'trans' else "odom")
        else:
            path_xy = od_xy
            src = "odom"
    return {
        "controller": controller, "direction": direction, "trial": trial,
        "src": src, "goal_xy": goal_xy, "plan_xy": plan_xy, "path_xy": path_xy
    }

def make_trajectory_plots(bag_paths, out_dir: Path, cfg, max_per_group=5):
    out_dir.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[WARN] matplotlib ausente para trayectorias: {e}")
        return
    grouped = {}
    for bag in bag_paths:
        info = extract_traj_for_plot(bag, cfg)
        key = (info["controller"], info["direction"])
        grouped.setdefault(key, []).append(info)
    for (controller, direction), lst in grouped.items():
        lst = sorted(lst, key=lambda d: d["trial"])[:max_per_group]
        if not lst: continue
        fig, ax = plt.subplots(figsize=(7,7))
        plan_plotted = False; goal_xy_to_show = None
        for info in lst:
            path = info["path_xy"]
            if path is None or len(path) == 0: continue
            xs, ys = path[:,0], path[:,1]
            ax.plot(xs, ys, linewidth=1.5, alpha=0.9, label=f"trial {info['trial']} ({info['src']})")
            ax.scatter([xs[0]], [ys[0]], s=25)
            if (not plan_plotted) and info["plan_xy"]:
                px, py = zip(*info["plan_xy"])
                ax.plot(px, py, linewidth=1.0, alpha=0.4, linestyle="--", label="global plan")
                plan_plotted = True
            if info["goal_xy"] and goal_xy_to_show is None:
                goal_xy_to_show = info["goal_xy"]
        if goal_xy_to_show is not None:
            ax.scatter([goal_xy_to_show[0]], [goal_xy_to_show[1]], marker='x', s=70, label="goal")
        ax.set_title(f"Trayectorias: {controller} - {direction}")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.axis('equal'); ax.grid(True, alpha=0.3); ax.legend(loc="best", fontsize=8)
        fig.tight_layout()
        out_path = out_dir / f"traj_{controller}_{direction}.png"
        fig.savefig(out_path, dpi=150); plt.close(fig)
        print(f"[OK] Trayectoria guardada: {out_path}")

# --------- main ----------
def main():
    p = argparse.ArgumentParser()
    p.add_argument("--results-dir", help="Directorio con subcarpetas {controller}_{forward|return}_{trial}_... que contienen .db3")
    p.add_argument("--bags", nargs="+", help="Opcional: rutas a .db3 o carpetas de bag (mezclable con --results-dir)")
    p.add_argument("--out-csv", required=True)
    p.add_argument("--out-summary", required=True)
    p.add_argument("--fig-dir", default=None)
    p.add_argument("--traj-dir", default=None)
    p.add_argument("--traj-max-per-group", type=int, default=5)
    p.add_argument("--direction-filter", default="all", choices=["forward","return","all"])
    # Llegada y alineación
    p.add_argument("--pos-radius", type=float, default=0.50)
    p.add_argument("--pos-hold", type=float, default=0.30)
    p.add_argument("--yaw-deg", type=float, default=15.0)
    p.add_argument("--yaw-hold", type=float, default=0.20)
    p.add_argument("--require_yaw", action="store_true", help="Éxito exige posición+yaw")
    p.add_argument("--align-fit-s", type=float, default=3.0)
    p.add_argument("--align-prune-m", type=float, default=0.75)
    p.add_argument("--track-tol-m", type=float, default=0.20, help="Tolerancia lateral (m) para integral de exceso")
    p.add_argument("--plan-select-window-s", type=float, default=2.0, help="Ventana para escoger plan máximo (<= t0+Δ)")
    # Otros umbrales
    p.add_argument("--stop-speed", type=float, default=0.03)
    p.add_argument("--clearance-safe", type=float, default=0.35)
    p.add_argument("--high-cost", type=int, default=200)
    # Filtrado por controlador (opcional)
    p.add_argument("--controllers", nargs="*", default=None, help="Ej: DWB RPP MPPI (por defecto todos)")
    p.add_argument("--debug", action="store_true")
    args = p.parse_args()

    bag_paths = []
    if args.results_dir:
        bag_paths.extend(find_bag_paths(Path(os.path.expanduser(args.results_dir))))
    if args.bags:
        for b in args.bags:
            pth = Path(os.path.expanduser(b))
            if pth.is_dir() or pth.suffix==".db3":
                bag_paths.append(pth)
    if not bag_paths:
        print("[ERROR] No se encontraron bags"); return

    # Filtrar por meta y dirección
    filtered = []
    for bp in bag_paths:
        ctrl, direc, trial = parse_bag_meta(bp)
        if args.controllers and ctrl and ctrl not in [c.upper() for c in args.controllers]:
            continue
        if args.direction_filter != "all" and direc and direc != args.direction_filter:
            continue
        filtered.append(bp)
    bag_paths = filtered if filtered else bag_paths

    cfg = {
        "pos_radius": args.pos_radius, "pos_hold": args.pos_hold,
        "yaw_deg": args.yaw_deg, "yaw_hold": args.yaw_hold,
        "require_yaw": bool(args.require_yaw),
        "align_fit_s": args.align_fit_s, "align_prune_m": args.align_prune_m,
        "track_tol_m": args.track_tol_m,
        "stop_speed": args.stop_speed, "clearance_safe": args.clearance_safe,
        "high_cost": args.high_cost,
        "plan_select_window_s": args.plan_select_window_s,
    }

    rclpy.init()
    rows = []
    for bag in bag_paths:
        ctrl, direc, trial = parse_bag_meta(bag)
        if args.debug:
            print(f"[INFO] Analizando: {bag}  meta=({ctrl},{direc},{trial})")
        try:
            m = compute_metrics_for_bag(bag, cfg)
            if m is None:
                print(f"[WARN] Bag vacío o sin mensajes: {bag}")
                continue
            m.update({"controller": ctrl or "UNKNOWN", "direction": direc or "unknown", "trial": trial or -1})
            rows.append(m)
        except Exception as e:
            print(f"[ERROR] {bag}: {e}")

    if not rows:
        print("[ERROR] No se generaron métricas"); return

    df = pd.DataFrame(rows).sort_values(by=["controller","trial","direction"])
    out_csv = Path(os.path.expanduser(args.out_csv)); out_csv.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(out_csv, index=False); print(f"[OK] CSV por trial: {out_csv}")

    # Resumen por (controller) y por (controller,direction)
    sum_ctrl = summarize_group(df, ["controller"])
    sum_ctrl_dir = summarize_group(df, ["controller","direction"])

    out_summary = Path(os.path.expanduser(args.out_summary)); out_summary.parent.mkdir(parents=True, exist_ok=True)
    base = out_summary.with_suffix("")
    sum_ctrl.to_csv(out_summary, index=False); print(f"[OK] Resumen por controlador: {out_summary}")
    out_summary2 = Path(str(base) + "_by_direction.csv")
    sum_ctrl_dir.to_csv(out_summary2, index=False); print(f"[OK] Resumen por controlador y dirección: {out_summary2}")

    # Figuras
    if args.fig_dir:
        try:
            make_all_plots(df, Path(os.path.expanduser(args.fig_dir)))
            print(f"[OK] Figuras en: {args.fig_dir}")
        except Exception as e:
            print(f"[WARN] No pude generar figuras (métricas): {e}")

    # Trayectorias
    if args.traj_dir:
        try:
            make_trajectory_plots(bag_paths, Path(os.path.expanduser(args.traj_dir)), cfg, max_per_group=args.traj_max_per_group)
        except Exception as e:
            print(f"[WARN] No pude generar trayectorias: {e}")

if __name__ == "__main__":
    main()
