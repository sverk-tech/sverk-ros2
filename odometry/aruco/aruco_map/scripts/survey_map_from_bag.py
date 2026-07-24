#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
survey_map_from_bag.py — уточнение карты ArUco-маркеров по записанному бэгу.

Зачем: markers.txt обычно набивается по номинальной сетке, а физически метки
лежат с отклонениями в 1–2.5 см и разворотами ~1°. При съёмке с ~1 м каждый
сантиметр ошибки карты — это ~10 px репроекции: RANSAC-локализация начинает
сериями браковать кадры (см. разбор полётов 2026-07-19: yield 27–47% на
номинальной карте против 100% на уточнённой, те же самые бэги).

Что делает: берёт углы маркеров из /aruco/det/markers (детекция уже сделана
бортом — картинки не нужны), восстанавливает взаимные положения меток через
пер-маркерные IPPE-позы, затем 2–4 итерации чередующегося уточнения
(позы кадров ↔ позы меток) и пишет markers.txt с фактическими x, y, z, rot_z.

Запуск НЕ требует ROS — только python3 + numpy + opencv + scipy (+ pyyaml):
бэг читается напрямую из sqlite3, CDR-сообщения разбираются вручную.

Примеры:
  # минимум: бэг(и) + калибровка камеры; якорь выбирается автоматически
  python3 survey_map_from_bag.py --bag flight1 --bag flight2 \
      --calib imx219_1280x960.active.yaml --out markers_surveyed.txt

  # сохранить мировую систему прежней карты (совместить по общим id):
  python3 survey_map_from_bag.py --bag flight1 --calib cam.yaml \
      --align-to config/markers.txt --out markers_surveyed.txt

Рекомендации по съёмке: 1–3 минуты полёта/проноса камеры над полем так, чтобы
каждая пара соседних меток побывала в одном кадре десятки раз. После
перекалибровки камеры survey нужно повторить (координаты частично впитывают
ошибки калибровки).
"""

import argparse
import math
import sqlite3
import struct
import sys
from collections import defaultdict

import numpy as np
import cv2
from scipy.optimize import least_squares


# ─────────────────────────────────────────────────────────────────────────────
# Чтение rosbag2 (sqlite3) и ручной разбор CDR — без зависимостей от ROS
# ─────────────────────────────────────────────────────────────────────────────
class CdrReader:
    def __init__(self, buf):
        self.buf = buf
        self.pos = 4  # 4 байта encapsulation header

    def _align(self, n):
        rel = self.pos - 4
        self.pos += (-rel) % n

    def u32(self):
        self._align(4)
        v = struct.unpack_from('<I', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def i32(self):
        self._align(4)
        v = struct.unpack_from('<i', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def f32(self):
        self._align(4)
        v = struct.unpack_from('<f', self.buf, self.pos)[0]
        self.pos += 4
        return v

    def f64(self):
        self._align(8)
        v = struct.unpack_from('<d', self.buf, self.pos)[0]
        self.pos += 8
        return v

    def string(self):
        n = self.u32()
        s = self.buf[self.pos:self.pos + n - 1].decode('utf-8', 'replace')
        self.pos += n
        return s


def parse_marker_array(buf):
    """aruco_det_loc/MarkerArray -> [(id, size_m, corners_px 4x2)]."""
    r = CdrReader(buf)
    r.i32(); r.u32(); r.string()          # header
    out = []
    for _ in range(r.u32()):
        mid = r.u32()
        size = r.f32()
        for _ in range(7):                # geometry_msgs/Pose (не используется)
            r.f64()
        corners = [(r.f32(), r.f32(), r.f32()) for _ in range(4)]
        out.append((mid, size, np.array([(x, y) for x, y, _ in corners])))
    return out


def load_bag_frames(bag_dir, topic):
    import glob
    import os
    db3 = sorted(glob.glob(os.path.join(bag_dir, '*.db3')))
    if not db3:
        sys.exit(f"нет .db3 в {bag_dir}")
    frames = []
    for db in db3:
        con = sqlite3.connect(db)
        tid = con.execute("SELECT id FROM topics WHERE name=?", (topic,)).fetchone()
        if tid is None:
            con.close()
            continue
        for _, data in con.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
                (tid[0],)):
            frames.append(parse_marker_array(data))
        con.close()
    return frames


# ─────────────────────────────────────────────────────────────────────────────
# Камера
# ─────────────────────────────────────────────────────────────────────────────
def load_calib(path):
    import yaml
    with open(path, 'r', encoding='utf-8') as f:
        y = yaml.safe_load(f)
    K = np.array(y['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
    D = np.array(y['distortion_coefficients']['data'], dtype=np.float64).ravel()
    model = y.get('distortion_model', 'plumb_bob')
    fisheye = (model == 'equidistant' and D.size == 4)
    return K, D, fisheye


def make_undistort(K, D, fisheye):
    """px -> нормализованные координаты идеальной pinhole-камеры."""
    if fisheye:
        def undist(pts):
            p = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
            return cv2.fisheye.undistortPoints(p, K, D).reshape(-1, 2)
    else:
        def undist(pts):
            p = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
            return cv2.undistortPoints(p, K, D).reshape(-1, 2)
    return undist


# ─────────────────────────────────────────────────────────────────────────────
# Геометрия
# ─────────────────────────────────────────────────────────────────────────────
def marker_local_corners(size_m):
    s = size_m / 2.0
    # tl, tr, br, bl — порядок углов cv2.aruco / aruco_detect
    return np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]],
                    dtype=np.float64)


def marker_corners_world(p, size_m):
    x, y, z, yaw = p
    c, s = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return (Rz @ marker_local_corners(size_m).T).T + np.array([x, y, z])


def ippe_pose(size_m, corners_norm):
    ok, rvec, tvec = cv2.solvePnP(
        marker_local_corners(size_m), corners_norm.reshape(-1, 1, 2),
        np.eye(3), None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return rvec.flatten(), tvec.flatten()


# ─────────────────────────────────────────────────────────────────────────────
# Основной алгоритм
# ─────────────────────────────────────────────────────────────────────────────
def initial_map(frames, undist, sizes, anchor, min_pair_obs):
    """Начальная карта из медиан пер-маркерных IPPE-поз (граф от якоря)."""
    pair_d = defaultdict(list)
    pair_yaw = defaultdict(list)
    for det in frames:
        if len(det) < 2:
            continue
        poses = {}
        for mid, cs in det.items():
            p = ippe_pose(sizes[mid], undist(cs))
            if p is not None:
                poses[mid] = p
        for a in poses:
            for b in poses:
                if a == b:
                    continue
                ra, ta = poses[a]
                rb, tb = poses[b]
                Ra, _ = cv2.Rodrigues(ra)
                pair_d[(a, b)].append(Ra.T @ (tb - ta))
                Rb, _ = cv2.Rodrigues(rb)
                Rab = Ra.T @ Rb
                pair_yaw[(a, b)].append(math.atan2(Rab[1, 0], Rab[0, 0]))

    edges = {k: (np.median(np.array(v), axis=0), np.median(pair_yaw[k]))
             for k, v in pair_d.items() if len(v) >= min_pair_obs}
    mp = {anchor: np.zeros(4)}
    frontier = [anchor]
    while frontier:
        a = frontier.pop()
        for (u, v), (d, dyaw) in edges.items():
            if u != a or v in mp:
                continue
            x, y, z, yaw = mp[a]
            c, s = math.cos(yaw), math.sin(yaw)
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            mp[v] = np.array([*(np.array([x, y, z]) + Rz @ d), yaw + dyaw])
            frontier.append(v)
    return mp, edges


def solve_frame(det, mp, sizes, undist, thr_norm, iters=100):
    obj, imn = [], []
    for mid, cs in det.items():
        if mid not in mp:
            continue
        obj.append(marker_corners_world(mp[mid], sizes[mid]))
        imn.append(undist(cs))
    if not obj:
        return None
    obj = np.concatenate(obj)
    imn = np.concatenate(imn)
    if len(obj) < 6:
        return None
    ok, rvec, tvec, inl = cv2.solvePnPRansac(
        obj, imn.reshape(-1, 1, 2), np.eye(3), None,
        iterationsCount=iters, reprojectionError=thr_norm, confidence=0.999,
        flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or inl is None or len(inl) < 6:
        return None
    return rvec, tvec, obj, imn


def refine(frames, mp, sizes, undist, anchor, fx, iters, planar,
           max_obs_per_marker=1500):
    thr_norm = 12.0 / fx
    for it in range(iters):
        obs = defaultdict(list)
        used = 0
        for det in frames:
            if len(det) < 2:
                continue
            r = solve_frame(det, mp, sizes, undist, thr_norm)
            if r is None:
                continue
            rvec, tvec, _, _ = r
            used += 1
            for mid, cs in det.items():
                if mid in mp:
                    obs[mid].append((rvec.copy(), tvec.copy(), undist(cs)))
        print(f"  итерация {it + 1}/{iters}: кадров в решении {used}")
        for mid, data in obs.items():
            if mid == anchor or len(data) < 30:
                continue
            data = data[:max_obs_per_marker]

            def resid(p):
                if planar:
                    p = np.array([p[0], p[1], 0.0, p[2]])
                w = marker_corners_world(p, sizes[mid])
                out = []
                for rvec, tvec, un in data:
                    proj, _ = cv2.projectPoints(w, rvec, tvec, np.eye(3), None)
                    out.append((proj.reshape(-1, 2) - un).ravel())
                return np.concatenate(out)

            x0 = (np.array([mp[mid][0], mp[mid][1], mp[mid][3]]) if planar
                  else mp[mid])
            sol = least_squares(resid, x0, loss='huber', f_scale=3.0 / fx,
                                method='trf')
            mp[mid] = (np.array([sol.x[0], sol.x[1], 0.0, sol.x[2]]) if planar
                       else sol.x)
    return mp


def residual_report(frames, mp, sizes, undist, fx):
    thr_norm = 12.0 / fx
    errs = []
    yields = 0
    elig = 0
    for det in frames:
        if len(det) < 2:
            continue
        elig += 1
        r = solve_frame(det, mp, sizes, undist, thr_norm)
        if r is None:
            continue
        rvec, tvec, obj, imn = r
        proj, _ = cv2.projectPoints(obj, rvec, tvec, np.eye(3), None)
        e = np.linalg.norm(proj.reshape(-1, 2) - imn, axis=1) * fx
        errs.append(e)
        yields += 1
    if not errs:
        print("  нет решённых кадров — отчёт невозможен")
        return
    e = np.sort(np.concatenate(errs))
    print(f"  кадров с >=2 метками: {elig}, решено: {yields} "
          f"({100 * yields / elig:.1f}%)")
    print(f"  остатки репроекции: med {e[len(e) // 2]:.2f} px, "
          f"p90 {e[int(len(e) * 0.9)]:.2f} px, p99 {e[int(len(e) * 0.99)]:.2f} px")


def align_to_reference(mp, ref_path):
    """2D-совмещение (поворот+сдвиг, без масштаба) с прежней картой по общим id."""
    ref = {}
    with open(ref_path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            t = line.split()
            ref[int(t[0])] = (float(t[2]), float(t[3]))
    common = sorted(set(mp) & set(ref))
    if len(common) < 2:
        print(f"  align-to: общих меток с {ref_path} меньше двух — пропускаю")
        return mp
    src = np.array([mp[i][:2] for i in common])
    dst = np.array([ref[i] for i in common])
    sc, dc = src.mean(axis=0), dst.mean(axis=0)
    H = (src - sc).T @ (dst - dc)
    U, _, Vt = np.linalg.svd(H)
    R2 = Vt.T @ U.T
    if np.linalg.det(R2) < 0:
        Vt[1] *= -1
        R2 = Vt.T @ U.T
    t2 = dc - R2 @ sc
    dyaw = math.atan2(R2[1, 0], R2[0, 0])
    out = {}
    for mid, p in mp.items():
        xy = R2 @ p[:2] + t2
        out[mid] = np.array([xy[0], xy[1], p[2], p[3] + dyaw])
    rms = math.sqrt(np.mean(np.sum(
        (np.array([out[i][:2] for i in common]) - dst) ** 2, axis=1)))
    print(f"  align-to: {len(common)} общих меток, невязка RMS {rms * 100:.1f} см, "
          f"поворот {math.degrees(dyaw):+.2f}°")
    return out


def main():
    ap = argparse.ArgumentParser(
        description="Уточнение карты ArUco по бэгу с /aruco/det/markers")
    ap.add_argument('--bag', action='append', required=True,
                    help='директория rosbag2 (можно несколько раз)')
    ap.add_argument('--calib', required=True, help='YAML калибровки камеры')
    ap.add_argument('--out', required=True, help='выходной markers.txt')
    ap.add_argument('--topic', default='/aruco/det/markers')
    ap.add_argument('--anchor', type=int, default=None,
                    help='id якорной метки (по умолчанию — самая наблюдаемая)')
    ap.add_argument('--align-to', default=None,
                    help='прежний markers.txt: сохранить его мировую систему')
    ap.add_argument('--iters', type=int, default=3)
    ap.add_argument('--min-pair-obs', type=int, default=20)
    ap.add_argument('--planar', action='store_true',
                    help='зажать z=0 (метки на ровном полу)')
    ap.add_argument('--min-frames', type=int, default=200,
                    help='минимум кадров с >=2 метками для survey')
    args = ap.parse_args()

    K, D, fisheye = load_calib(args.calib)
    fx = (K[0, 0] + K[1, 1]) / 2
    undist = make_undistort(K, D, fisheye)
    print(f"калибровка: fx={fx:.1f} "
          f"{'fisheye/equidistant' if fisheye else 'plumb_bob'}")

    raw = []
    for b in args.bag:
        raw += load_bag_frames(b, args.topic)
    counts = defaultdict(int)
    sizes = {}
    for fr in raw:
        for mid, size, _ in fr:
            counts[mid] += 1
            sizes[mid] = size
    if not counts:
        sys.exit("в бэгах нет сообщений с метками")
    # отсечь случайные детекции: метка должна встретиться хотя бы в 1% кадров
    keep = {mid for mid, c in counts.items() if c >= max(20, len(raw) // 100)}
    print(f"кадров: {len(raw)}; метки: "
          + ", ".join(f"{m}(n={counts[m]}, {sizes[m]:.3f} м)"
                      for m in sorted(keep)))
    dropped = sorted(set(counts) - keep)
    if dropped:
        print(f"отброшены редкие id (шум детекции?): {dropped}")

    frames = [{mid: cs for mid, _, cs in fr if mid in keep} for fr in raw]
    multi = sum(1 for f in frames if len(f) >= 2)
    if multi < args.min_frames:
        sys.exit(f"мало кадров с >=2 метками ({multi} < {args.min_frames}) — "
                 f"нужен более длинный пролёт над полем")

    anchor = args.anchor if args.anchor is not None \
        else max(keep, key=lambda m: counts[m])
    print(f"якорь: метка {anchor}")

    print("начальная карта из пер-маркерных поз…")
    mp, edges = initial_map(frames, undist, sizes, anchor, args.min_pair_obs)
    missing = keep - set(mp)
    if missing:
        print(f"ВНИМАНИЕ: метки {sorted(missing)} не связаны с якорем "
              f"(нет совместных наблюдений) — не войдут в карту")

    print("уточнение (позы кадров ↔ позы меток)…")
    mp = refine(frames, mp, sizes, undist, anchor, fx, args.iters, args.planar)

    print("контрольный реплей по уточнённой карте:")
    residual_report(frames, mp, sizes, undist, fx)

    if args.align_to:
        mp = align_to_reference(mp, args.align_to)

    with open(args.out, 'w', encoding='utf-8', newline='\n') as f:
        f.write("# id\tlength\tx\ty\tz\trot_z\trot_y\trot_x\n")
        f.write("# survey_map_from_bag.py: уточнено по "
                + ", ".join(args.bag) + "\n")
        for mid in sorted(mp):
            x, y, z, yaw = mp[mid]
            f.write(f"{mid}\t{sizes[mid]:.4f}\t{x:.4f}\t{y:.4f}\t{z:.4f}\t"
                    f"{yaw:.5f}\t0.0\t0.0\n")
    print(f"карта записана: {args.out}")
    for mid in sorted(mp):
        x, y, z, yaw = mp[mid]
        print(f"  {mid}: x={x:+.4f} y={y:+.4f} z={z:+.4f} "
              f"yaw={math.degrees(yaw):+.2f}°")


if __name__ == '__main__':
    main()
