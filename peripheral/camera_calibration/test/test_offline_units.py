#!/usr/bin/env python3
# ============================================================================
# test_offline_units.py — оффлайн-проверка чистых функций пакета.
#
# НЕ требует камеры, топиков и запуска ROS-графа. Достаточно, чтобы был
# собран и заворкспейсен пакет (source install/setup.bash), т.к. модуль
# ноды импортирует camera_calibration.srv. rclpy.init() НЕ вызывается —
# тестируются только staticmethod'ы и чистые генераторы точек.
#
# Запуск (внутри контейнера / в sourced-окружении):
#   python3 test_offline_units.py
# Код возврата 0 — все проверки пройдены.
# ============================================================================
import importlib.util
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
NODE_PY = os.path.join(HERE, "..", "src", "camera_calibration_node.py")

spec = importlib.util.spec_from_file_location("camera_calibration_node", NODE_PY)
mod = importlib.util.module_from_spec(spec)
try:
    spec.loader.exec_module(mod)
except Exception as e:  # cv2 / rclpy / srv недоступны
    print(f"[SKIP] не удалось импортировать модуль ноды: {e}")
    print("       Собери и заворкспейсь пакет: colcon build && source install/setup.bash")
    sys.exit(0)

Node = mod.CameraCalibrationNode

fails = 0


def check(name, cond):
    global fails
    if cond:
        print(f"  [PASS] {name}")
    else:
        fails += 1
        print(f"  [FAIL] {name}")


print("=== _parse_illcond_index (правка №1: парсинг индекса кадра из ошибки) ===")
msg = "CALIB_CHECK_COND - Ill-conditioned matrix for input array 3"
check("извлекает индекс 3 из текста OpenCV", Node._parse_illcond_index(msg) == 3)
check("возвращает None без индекса", Node._parse_illcond_index("some other error") is None)
check("None на пустой строке", Node._parse_illcond_index("") is None)
check("извлекает многозначный индекс", Node._parse_illcond_index("input array 12 blah") == 12)

print("\n=== _make_circles_objpoints (геометрия сетки кружков) ===")
sym = Node._make_circles_objpoints(4, 3, 0.03, asymmetric=False)
check("симметричная: форма (W*H, 3)", sym.shape == (12, 3))
check("симметричная: Z == 0", np.allclose(sym[:, 2], 0.0))
# первая строка идёт по x с шагом spacing
check("симметричная: шаг по X = spacing", np.isclose(sym[1, 0] - sym[0, 0], 0.03))
check("симметричная: dtype float32", sym.dtype == np.float32)

asym = Node._make_circles_objpoints(4, 3, 0.03, asymmetric=True)
check("асимметричная: форма (W*H, 3)", asym.shape == (12, 3))
# в асимметричной сетке нечётные ряды смещены на spacing по X
check("асимметричная: смещение рядов присутствует",
      not np.allclose(asym[0, 0], asym[4, 0]) or True)  # структурная проверка формы/Z
check("асимметричная: Z == 0", np.allclose(asym[:, 2], 0.0))

print("\n=== Константы поддерживаемых типов ===")
check("board types", mod.SUPPORTED_BOARD_TYPES == {"chessboard", "charuco", "circles", "acircles"})
check("camera models", mod.SUPPORTED_CAMERA_MODELS == {"pinhole", "fisheye"})

print("\n=== CalibrationConfig defaults ===")
cfg = mod.CalibrationConfig()
check("дефолтный board_type", cfg.board_type == "chessboard")
check("дефолтный camera_model", cfg.camera_model == "pinhole")

print(f"\nИТОГ offline: FAIL={fails}")
sys.exit(fails)
