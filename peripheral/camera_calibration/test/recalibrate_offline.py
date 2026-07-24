#!/usr/bin/env python3
# ============================================================================
# recalibrate_offline.py — переигрывает калибровку по УЖЕ снятым кадрам.
#
# Зачем:
#   - сравнить pinhole и fisheye на ОДНОМ И ТОМ ЖЕ наборе кадров;
#   - найти кадры-выбросы (bad frames), которые задирают ошибку;
#   - пере-калибровать fisheye БЕЗ выбросов и получить честный reproj.
#
# Использует сырые кадры, сохранённые нодой при save_captures=true:
#   ~/camera_calibrations/sessions/<id>/captures/raw_*.jpg
#
# Работает автономно: нужен только OpenCV (тот же cv2 4.5.4, что в контейнере),
# ROS не требуется.
#
# Пример (charuco 11x8, как в твоём прогоне):
#   python3 recalibrate_offline.py \
#     --captures ~/camera_calibrations/sessions/20260701_223212/captures \
#     --board charuco --w 11 --h 8 --square 0.022 --marker 0.016 \
#     --dict DICT_4X4_50 --min-corners 15 --drop-threshold 2.0
# ============================================================================
import argparse
import glob
import os
import sys

import cv2
import numpy as np


def get_aruco_dict(name):
    if hasattr(cv2.aruco, name):
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


def make_charuco_board(w, h, square, marker, dictionary):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(w, h, square, marker, dictionary)
    return cv2.aruco.CharucoBoard((w, h), square, marker, dictionary)


def make_aruco_params():
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def detect_markers(gray, dictionary, params, detector):
    if detector is not None:
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=params)


def collect_points(args):
    """Возвращает (names, objpoints, imgpoints, image_size)."""
    dictionary = get_aruco_dict(args.dict)
    params = make_aruco_params()
    detector = None
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
    board = make_charuco_board(args.w, args.h, args.square, args.marker, dictionary)

    files = sorted(glob.glob(os.path.join(os.path.expanduser(args.captures), "raw_*.jpg")))
    if not files:
        print(f"Нет кадров raw_*.jpg в {args.captures}")
        sys.exit(2)

    names, objpoints, imgpoints = [], [], []
    image_size = None
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])

        if args.board == "charuco":
            corners, ids, _ = detect_markers(gray, dictionary, params, detector)
            if ids is None or len(ids) == 0:
                print(f"  [skip] {os.path.basename(f)}: маркеры не найдены")
                continue
            _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if ch_ids is None or ch_corners is None or len(ch_ids) < args.min_corners:
                n = 0 if ch_ids is None else len(ch_ids)
                print(f"  [skip] {os.path.basename(f)}: углов {n} < {args.min_corners}")
                continue
            objp = board.chessboardCorners[ch_ids.flatten()].astype(np.float32)
            imgp = ch_corners.reshape(-1, 2).astype(np.float32)
        else:  # chessboard
            found, corners = cv2.findChessboardCorners(
                gray, (args.w, args.h),
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if not found:
                print(f"  [skip] {os.path.basename(f)}: доска не найдена")
                continue
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
            )
            objp = np.zeros((args.w * args.h, 3), np.float32)
            objp[:, :2] = np.mgrid[0:args.w, 0:args.h].T.reshape(-1, 2) * args.square
            imgp = corners.reshape(-1, 2).astype(np.float32)

        names.append(os.path.basename(f))
        objpoints.append(objp)
        imgpoints.append(imgp)

    print(f"Собрано кадров: {len(objpoints)} из {len(files)}")
    return names, objpoints, imgpoints, image_size


def per_frame_err_pinhole(objp, imgp, K, D, rv, tv):
    proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
    return float(np.linalg.norm(imgp.reshape(-1, 2) - proj.reshape(-1, 2), axis=1).mean())


def per_frame_err_fisheye(objp, imgp, K, D, rv, tv):
    o = objp.reshape(-1, 1, 3).astype(np.float64)
    i = imgp.reshape(-1, 1, 2).astype(np.float64)
    proj, _ = cv2.fisheye.projectPoints(o, rv, tv, K, D)
    return float(np.linalg.norm((i - proj).reshape(-1, 2), axis=1).mean())


def calib_pinhole(objpoints, imgpoints, size):
    op = [o.reshape(-1, 1, 3).astype(np.float32) for o in objpoints]
    ip = [i.reshape(-1, 1, 2).astype(np.float32) for i in imgpoints]
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(op, ip, size, None, None)
    errs = [per_frame_err_pinhole(op[j], ip[j], K, D, rvecs[j], tvecs[j]) for j in range(len(op))]
    return rms, K, D, errs


def calib_fisheye(objpoints, imgpoints, size):
    op = [o.reshape(-1, 1, 3).astype(np.float64) for o in objpoints]
    ip = [i.reshape(-1, 1, 2).astype(np.float64) for i in imgpoints]
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    keep = list(range(len(op)))
    dropped = []
    while True:
        o = [op[k] for k in keep]
        i = [ip[k] for k in keep]
        if len(o) < 3:
            raise RuntimeError("слишком мало годных кадров")
        K = np.zeros((3, 3)); D = np.zeros((4, 1))
        try:
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                o, i, size, K, D, None, None,
                flags=flags | cv2.fisheye.CALIB_CHECK_COND, criteria=crit)
            break
        except cv2.error as e:
            import re
            m = re.search(r"input array (\d+)", str(e))
            if m and 0 <= int(m.group(1)) < len(keep):
                dropped.append(keep.pop(int(m.group(1))))
                continue
            rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                o, i, size, K, D, None, None, flags=flags, criteria=crit)
            break
    errs = {keep[j]: per_frame_err_fisheye(op[keep[j]], ip[keep[j]], K, D, rvecs[j], tvecs[j])
            for j in range(len(keep))}
    return rms, K, D, errs, dropped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--captures", required=True, help="папка с raw_*.jpg")
    ap.add_argument("--board", default="charuco", choices=["charuco", "chessboard"])
    ap.add_argument("--w", type=int, required=True)
    ap.add_argument("--h", type=int, required=True)
    ap.add_argument("--square", type=float, required=True)
    ap.add_argument("--marker", type=float, default=0.016)
    ap.add_argument("--dict", default="DICT_4X4_50")
    ap.add_argument("--min-corners", type=int, default=15)
    ap.add_argument("--drop-threshold", type=float, default=0.0,
                    help="если >0: убрать кадры с ошибкой выше порога (px) и пере-калибровать fisheye")
    args = ap.parse_args()

    names, objp, imgp, size = collect_points(args)
    if len(objp) < 3:
        print("Недостаточно кадров для калибровки"); sys.exit(2)
    print(f"Размер изображения: {size[0]}x{size[1]}\n")

    # ---- PINHOLE ----
    print("=== PINHOLE (plumb_bob) — для сравнения (для 160° модель неадекватна) ===")
    try:
        rms_p, K_p, D_p, errs_p = calib_pinhole(objp, imgp, size)
        print(f"  rms={rms_p:.3f}  mean_reproj={np.mean(errs_p):.3f}  "
              f"fx={K_p[0,0]:.1f} fy={K_p[1,1]:.1f} cx={K_p[0,2]:.1f} cy={K_p[1,2]:.1f}")
    except cv2.error as e:
        print(f"  pinhole не сошёлся: {e}")

    # ---- FISHEYE ----
    print("\n=== FISHEYE (equidistant) — рабочая модель для 160° ===")
    rms_f, K_f, D_f, errs_f, dropped = calib_fisheye(objp, imgp, size)
    mean_f = np.mean(list(errs_f.values()))
    print(f"  rms={rms_f:.3f}  mean_reproj={mean_f:.3f}  "
          f"fx={K_f[0,0]:.1f} fy={K_f[1,1]:.1f} cx={K_f[0,2]:.1f} cy={K_f[1,2]:.1f}")
    if dropped:
        print(f"  отброшено ill-conditioned кадров: {[names[d] for d in dropped]}")

    # ---- список кадров по ошибке (выбросы сверху) ----
    print("\n  Ошибка по кадрам (fisheye, худшие сверху):")
    ranked = sorted(errs_f.items(), key=lambda kv: kv[1], reverse=True)
    for idx, e in ranked[:10]:
        flag = "  <-- выброс" if e > 2.0 else ""
        print(f"    {names[idx]:>16}: {e:.3f} px{flag}")

    # ---- пере-калибровка без выбросов ----
    if args.drop_threshold > 0:
        good = [k for k, e in errs_f.items() if e <= args.drop_threshold]
        bad = [k for k, e in errs_f.items() if e > args.drop_threshold]
        print(f"\n=== FISHEYE без кадров > {args.drop_threshold} px "
              f"(убрано {len(bad)}, осталось {len(good)}) ===")
        if len(good) < 3:
            print("  слишком мало годных кадров после отсева"); return
        rms2, K2, D2, errs2, dr2 = calib_fisheye(
            [objp[k] for k in good], [imgp[k] for k in good], size)
        print(f"  rms={rms2:.3f}  mean_reproj={np.mean(list(errs2.values())):.3f}  "
              f"fx={K2[0,0]:.1f} fy={K2[1,1]:.1f} cx={K2[0,2]:.1f} cy={K2[1,2]:.1f}")
        print(f"  убранные кадры: {[names[b] for b in bad]}")
        print("\n  Если mean_reproj заметно упал — пересними эти ракурсы в ноде "
              "или подними min_charuco_corners.")


if __name__ == "__main__":
    main()
