#!/usr/bin/env python3
# ============================================================================
# verify_calibration_node.py — инструмент верификации калибровочного YAML.
#
# Читает файл калибровки (формат, который пишет camera_calibration_node),
# выводит полную статистику и вердикт с рекомендациями: калибровка отличная и
# её можно оставить, либо стоит переснять/перекалибровать и почему.
#
# Запуск:
#   ros2 run camera_calibration verify_calibration_node.py --file <path.yaml>
#   ros2 run camera_calibration verify_calibration_node.py            # возьмёт
#       ~/camera_calibrations/latest_calibration.yaml
#
# Опционально — глубокий анализ по исходным кадрам (per-frame ошибки):
#   ... --captures <папка с raw_*.jpg>
# (для этого нужен OpenCV; без --captures работает на голом YAML.)
# ============================================================================
import argparse
import math
import os
import sys

# Гарантируем UTF-8 вывод (символы °, ≈, • на любой платформе/консоли).
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass

try:
    import yaml
except Exception as e:  # noqa
    print(f"PyYAML недоступен: {e}")
    sys.exit(2)


# ---- пороги оценки -----------------------------------------------------------
REPROJ = [
    (0.30, "ОТЛИЧНАЯ", "Калибровка превосходная — оставляй как есть."),
    (0.50, "ОЧЕНЬ ХОРОШАЯ", "Качество высокое, переделывать не нужно."),
    (1.00, "ХОРОШАЯ", "Рабочая калибровка. Можно оставить."),
    (2.00, "ПОСРЕДСТВЕННАЯ", "Годится, но лучше добрать кадров по краям и перекалибровать."),
    (float("inf"), "ПЛОХАЯ", "Рекомендуется ПЕРЕСНЯТЬ: вероятны кадры-выбросы или плохое покрытие."),
]

# смещение главной точки от центра (доля от полудиагонали)
PP_WARN = 0.12   # >12% — подозрительно
PP_BAD = 0.20    # >20% — почти наверняка выбросы/плохая калибровка
ASPECT_WARN = 0.05   # |fx/fy - 1| > 5% — необычно
MIN_FRAMES_WARN = 12


def grade_reproj(e):
    for thr, label, rec in REPROJ:
        if e <= thr:
            return label, rec
    return REPROJ[-1][1], REPROJ[-1][2]


def get_matrix(d, key, expect):
    m = d.get(key)
    if not isinstance(m, dict) or "data" not in m:
        return None
    data = m["data"]
    if len(data) != expect:
        return None
    return [float(x) for x in data]


def fov_deg(model, half, f):
    if f <= 0:
        return 0.0
    if model == "fisheye":  # equidistant: r = f*theta
        return math.degrees(2.0 * (half / f))
    return math.degrees(2.0 * math.atan2(half, f))  # pinhole


def main():
    ap = argparse.ArgumentParser(description="Верификация калибровочного YAML")
    ap.add_argument("--file", "-f", default=None, help="путь к YAML (по умолчанию latest_calibration.yaml)")
    ap.add_argument("--captures", default=None, help="папка с raw_*.jpg для per-frame анализа")
    args = ap.parse_args()

    path = args.file or os.path.expanduser("~/camera_calibrations/latest_calibration.yaml")
    if not os.path.isfile(path):
        print(f"Файл не найден: {path}")
        sys.exit(2)

    with open(path, "r", encoding="utf-8") as fh:
        d = yaml.safe_load(fh)

    warnings = []
    problems = []

    w = int(d.get("image_width", 0))
    h = int(d.get("image_height", 0))
    model = str(d.get("camera_model", "pinhole"))
    dist_model = str(d.get("distortion_model", ""))
    reproj = float(d.get("reprojection_error", 0.0))
    nframes = int(d.get("num_calibration_frames", 0))

    K = get_matrix(d, "camera_matrix", 9)
    Dm = d.get("distortion_coefficients", {})
    D = [float(x) for x in Dm.get("data", [])] if isinstance(Dm, dict) else []

    print("=" * 68)
    print(f"  ВЕРИФИКАЦИЯ КАЛИБРОВКИ: {os.path.basename(path)}")
    print("=" * 68)
    print(f"  Файл:         {path}")
    print(f"  Дата:         {d.get('calibration_date', '—')}")
    print(f"  Камера:       {d.get('camera_name', '—')}")
    print(f"  Разрешение:   {w} x {h}")
    print(f"  Модель:       {model}  (distortion: {dist_model})")
    print(f"  Доска:        {d.get('board_type','—')} "
          f"{d.get('board_width','?')}x{d.get('board_height','?')}, "
          f"square={d.get('square_size','?')} marker={d.get('marker_size','?')}")
    ncap = int(d.get("num_captured_frames", nframes))
    n_out = int(d.get("dropped_outliers", 0))
    n_ill = int(d.get("dropped_illcond", 0))
    extra = ""
    if ncap != nframes or n_out or n_ill:
        extra = f"  (снято {ncap}, отсеяно: выбросов {n_out}, ill-cond {n_ill})"
    print(f"  Кадров:       {nframes}{extra}")

    if K is None or w <= 0 or h <= 0:
        print("\n  [!] Некорректная camera_matrix или размеры — файл повреждён.")
        sys.exit(1)

    fx, fy = K[0], K[4]
    cx, cy = K[2], K[5]

    # --- интринсики ---
    print("\n--- Интринсики ---")
    print(f"  fx={fx:.2f}  fy={fy:.2f}   cx={cx:.2f}  cy={cy:.2f}")

    aspect = abs(fx / fy - 1.0) if fy else 1.0
    print(f"  Соотношение fx/fy: {fx/fy:.4f}  (отклонение {aspect*100:.2f}%)")
    if aspect > ASPECT_WARN:
        warnings.append(f"fx/fy отличается на {aspect*100:.1f}% — необычно для квадратных пикселей "
                        f"(возможен перекос из-за плохого покрытия).")

    # смещение главной точки
    dx = cx - w / 2.0
    dy = cy - h / 2.0
    half_diag = math.hypot(w / 2.0, h / 2.0)
    pp_off = math.hypot(dx, dy) / half_diag
    print(f"  Главная точка от центра: dx={dx:+.1f}px dy={dy:+.1f}px "
          f"({pp_off*100:.1f}% полудиагонали)")
    if pp_off > PP_BAD:
        problems.append(f"Главная точка уехала на {pp_off*100:.0f}% — сильный признак "
                        f"кадров-выбросов или недостаточного покрытия.")
    elif pp_off > PP_WARN:
        warnings.append(f"Главная точка смещена на {pp_off*100:.0f}% — приемлемо, но следи.")

    # FOV
    hfov = fov_deg(model, w / 2.0, fx)
    vfov = fov_deg(model, h / 2.0, fy)
    dfov = fov_deg(model, half_diag, (fx + fy) / 2.0)
    print(f"  Оценка FOV:  H≈{hfov:.1f}°  V≈{vfov:.1f}°  D≈{dfov:.1f}°")

    # соответствие модели и FOV
    if model == "pinhole" and hfov >= 110:
        warnings.append(f"Модель pinhole при H-FOV≈{hfov:.0f}° — для широкого угла "
                        f"radial-tangential модель плохо описывает края. Рассмотри fisheye.")
    if model == "fisheye" and hfov < 80:
        warnings.append(f"Модель fisheye при H-FOV≈{hfov:.0f}° — для неширокого объектива "
                        f"обычно достаточно pinhole.")

    # --- дисторсия ---
    print("\n--- Дисторсия ---")
    if model == "fisheye":
        names = ["k1", "k2", "k3", "k4"]
    else:
        names = ["k1", "k2", "p1", "p2", "k3"][:len(D)]
    print("  " + "  ".join(f"{n}={v:+.4f}" for n, v in zip(names, D)))
    if any(abs(v) > 5.0 for v in D):
        warnings.append("Есть очень большой коэффициент дисторсии (|v|>5) — вероятна "
                        "нестабильная подгонка.")

    # --- reproj + вердикт ---
    label, rec = grade_reproj(reproj)
    print("\n--- Ошибка репроекции ---")
    print(f"  reprojection_error = {reproj:.4f} px  ->  {label}")

    if nframes < MIN_FRAMES_WARN:
        warnings.append(f"Всего {nframes} кадров (<{MIN_FRAMES_WARN}) — маловато для "
                        f"надёжной калибровки, особенно широкого угла.")

    # --- опциональный per-frame анализ ---
    if args.captures:
        _per_frame_report(args, d, model, w, h, problems, warnings)

    # --- итог ---
    print("\n" + "=" * 68)
    print(f"  ВЕРДИКТ: {label}")
    print("=" * 68)
    print(f"  {rec}")

    if problems:
        print("\n  ПРОБЛЕМЫ (стоит переделать):")
        for p in problems:
            print(f"   • {p}")
    if warnings:
        print("\n  ПРЕДУПРЕЖДЕНИЯ:")
        for wmsg in warnings:
            print(f"   • {wmsg}")
    if not problems and not warnings and reproj <= 1.0:
        print("\n  Замечаний нет — калибровка чистая. [v]")

    # рекомендация по действию
    print("\n  РЕКОМЕНДАЦИЯ:")
    if reproj > 2.0 or problems:
        print("   -> ПЕРЕСНЯТЬ/ПЕРЕКАЛИБРОВАТЬ. Добери кадры с доской по краям и углам")
        print("     кадра под наклоном; авто-отсев выбросов уберёт единичные плохие кадры.")
        code = 1
    elif reproj > 1.0 or warnings:
        print("   -> Можно оставить, но при возможности улучши покрытие и перекалибруй.")
        code = 0
    else:
        print("   -> ОСТАВИТЬ КАК ЕСТЬ. Калибровка отличная. [v]")
        code = 0

    print()
    sys.exit(code)


def _per_frame_report(args, d, model, w, h, problems, warnings):
    # Глубокий анализ: пересчёт per-frame ошибок по сохранённым кадрам.
    try:
        import glob
        import cv2
        import numpy as np
    except Exception as e:
        print(f"\n  [per-frame пропущен: нет OpenCV/numpy: {e}]")
        return

    base = os.path.expanduser(args.captures)
    files = sorted(f for f in glob.glob(os.path.join(base, "raw_*.jpg")))
    if not files:
        print(f"\n  [per-frame пропущен: нет raw_*.jpg в {base}]")
        return

    board_type = str(d.get("board_type", "charuco"))
    bw, bh = int(d.get("board_width", 11)), int(d.get("board_height", 8))
    square = float(d.get("square_size", 0.022))
    marker = float(d.get("marker_size", 0.016))
    dict_name = str(d.get("aruco_dict", "DICT_4X4_50"))

    dictionary = cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, dict_name, cv2.aruco.DICT_4X4_50))
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(bw, bh, square, marker, dictionary)
    else:
        board = cv2.aruco.CharucoBoard((bw, bh), square, marker, dictionary)
    params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, "DetectorParameters")
              else cv2.aruco.DetectorParameters_create())

    K = np.array(d["camera_matrix"]["data"], float).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"], float).reshape(-1, 1)

    errs = []
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if board_type == "charuco":
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
            if ids is None or len(ids) == 0:
                continue
            _, cc, ci = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
            if ci is None or len(ci) < 6:
                continue
            objp = board.chessboardCorners[ci.flatten()].astype(np.float64)
            imgp = cc.reshape(-1, 2).astype(np.float64)
        else:
            found, corners = cv2.findChessboardCorners(gray, (bw, bh))
            if not found:
                continue
            objp = np.zeros((bw * bh, 3), np.float64)
            objp[:, :2] = np.mgrid[0:bw, 0:bh].T.reshape(-1, 2) * square
            imgp = corners.reshape(-1, 2).astype(np.float64)

        if model == "fisheye":
            # undistort в нормализованные координаты -> поза через solvePnP с
            # единичной матрицей -> обратная проекция полной fisheye-моделью.
            undist = cv2.fisheye.undistortPoints(imgp.reshape(-1, 1, 2), K, D)
            ok, rv, tv = cv2.solvePnP(
                objp.reshape(-1, 1, 3), undist, np.eye(3), np.zeros((4, 1)))
            proj, _ = cv2.fisheye.projectPoints(objp.reshape(-1, 1, 3), rv, tv, K, D)
            e = float(np.linalg.norm((imgp.reshape(-1, 1, 2) - proj).reshape(-1, 2), axis=1).mean())
        else:
            ok, rv, tv = cv2.solvePnP(objp, imgp, K, D)
            proj, _ = cv2.projectPoints(objp, rv, tv, K, D)
            e = float(np.linalg.norm(imgp - proj.reshape(-1, 2), axis=1).mean())
        errs.append((os.path.basename(f), e))

    if not errs:
        print("\n  [per-frame: ни один кадр не распознан]")
        return

    vals = np.array([e for _, e in errs])
    print("\n--- Per-frame (по сохранённым кадрам) ---")
    print(f"  кадров: {len(errs)}  mean={vals.mean():.3f}  median={np.median(vals):.3f}  max={vals.max():.3f}")
    ranked = sorted(errs, key=lambda kv: kv[1], reverse=True)
    print("  худшие кадры:")
    for name, e in ranked[:5]:
        flag = "  <-- выброс" if e > 2.0 else ""
        print(f"    {name:>16}: {e:.3f} px{flag}")
    n_out = int((vals > 2.0).sum())
    if n_out:
        problems.append(f"{n_out} кадр(ов) с ошибкой >2px — удали их и перекалибруй "
                        f"(нода делает это авто-отсевом).")


if __name__ == "__main__":
    main()
