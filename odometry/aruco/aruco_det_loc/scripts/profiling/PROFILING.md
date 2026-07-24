# aruco_det_loc — Profiling Guide

Подробное руководство по сбору и анализу timing-данных пайплайна ArUco-локализации.

---

## Обзор пайплайна

```
Камера
  │  (DDS transport)
  ▼
aruco_detect_node
  ├─ max_detect_fps  ранний пропуск кадров до cv_bridge/detectMarkers (если >0)
  ├─ convert_ms     BGR→Gray или MONO8 zero-copy share
  ├─ detect_ms      cv::aruco::detectMarkers
  ├─ pnp_ms         per-marker solvePnP (если estimate_marker_pose=true)
  └─ total_ms       весь callback (вход → публикация)
         │  (DDS transport — включён в markers_age_ms ноды loc)
         ▼
aruco_loc_node
  ├─ markers_age_ms  возраст сообщения = DDS latency + detect processing
  ├─ map_copy_ms     копия карты под mutex
  ├─ corr_ms         buildCorrespondences (3D–2D matching)
  ├─ pnp_ms          hybrid PnP (RANSAC or single-marker IPPE)
  ├─ refine_ms       refinePoseSolvePnP (если refine_pose=true)
  ├─ tf_ms           TF lookup (known_vertical, если задан)
  ├─ cov_ms          оценка ковариации (Jacobian)
  ├─ pub_ms          публикация pose + TF broadcast
  ├─ total_ms        весь callback ноды loc
  └─ image_to_pose_ms  END-TO-END от захвата кадра до публикации позиции
```

> **image_to_pose_ms** = `markers_age_ms` + `total_ms(loc)`.
> Это ключевая метрика: полная сквозная задержка системы.

---

## Шаг 1 — Запуск нод

### Вариант A: IPC-контейнер (рекомендуется для минимальной задержки)

```bash
ros2 launch aruco_det_loc aruco_composed.launch.py \
  world_frame_id:=world \
  known_vertical_frame:=base_link \
  auto_flip:=true
```

С привязкой к A76-ядрам (найдите ID через `check_cpu_topology.sh`):

```yaml
# affinity.yaml
aruco_detect:
  ros__parameters:
    cpu_affinity_ids: [0, 1]
    opencv_num_threads: 2
aruco_loc:
  ros__parameters:
    cpu_affinity_ids: [0, 1]
```

```bash
ros2 launch aruco_det_loc aruco_composed.launch.py \
  world_frame_id:=world \
  --ros-args --params-file affinity.yaml
```

### Вариант B: Отдельные ноды

```bash
ros2 run aruco_det_loc aruco_detect_node --ros-args \
  -p image_topic:=/camera/image_raw \
  -p camera_info_topic:=/camera/camera_info \
  -p use_map_markers:=true \
  -p map_markers_topic:=/aruco/map/markers \
  -p enable_timing_log:=true

ros2 run aruco_det_loc aruco_loc_node --ros-args \
  -p markers_topic:=/aruco/det/markers \
  -p map_markers_topic:=/aruco/map/markers \
  -p camera_info_topic:=/camera/camera_info \
  -p world_frame_id:=world \
  -p enable_timing_log:=true
```

---

## Шаг 2 — Включение timing-логирования

Параметр `enable_timing_log` можно менять **во время работы**:

```bash
# Включить
ros2 param set /aruco_detect enable_timing_log true
ros2 param set /aruco_loc    enable_timing_log true

# Выключить
ros2 param set /aruco_detect enable_timing_log false
ros2 param set /aruco_loc    enable_timing_log false
```

Timing-данные публикуются в JSON в общий топик `/aruco/timing` (обе ноды);
источник различается полем `"node"` в JSON: `"detect"` или `"loc"`.

При `max_detect_fps > 0` пропущенные кадры не публикуют `/aruco/det/markers` и не создают
timing JSON: в `/aruco/timing` видны только реально обработанные кадры.

Просмотр в терминале:

```bash
ros2 topic echo /aruco/timing
```

Пример JSON от detect:
```json
{
  "node": "detect", "seq": 42,
  "image_stamp": 1234567890.123456, "wall_now": 10.5,
  "convert_ms": 0.5, "detect_ms": 8.5, "pnp_ms": 0.0, "total_ms": 9.2,
  "n_detected": 3, "n_processed": 2,
  "pnp_calls_fresh": 0, "pnp_capped": false, "input_mono": false
}
```

Пример JSON от loc:
```json
{
  "node": "loc", "seq": 38,
  "image_stamp": 1234567890.123456, "wall_now": 10.8,
  "markers_age_ms": 12.5, "map_copy_ms": 0.02, "corr_ms": 0.08,
  "pnp_ms": 3.2, "refine_ms": 0.0, "tf_ms": 0.05,
  "cov_ms": 1.1, "pub_ms": 0.04, "total_ms": 4.5,
  "image_to_pose_ms": 17.0,
  "n_markers": 3, "n_corr_points": 12, "n_inliers": 12,
  "reproj_rmse": 1.23, "gating_passed": true, "tf_ok": true, "put_count_cov": false
}
```

---

## Шаг 3 — Запуск профилировщика

### Установка зависимостей

```bash
pip install pandas numpy tabulate
```

### Режим `--record-only` (снимок текущей конфигурации)

```bash
source install/setup.bash

# Включить timing (если ещё не включён)
ros2 param set /aruco_detect enable_timing_log true
ros2 param set /aruco_loc    enable_timing_log true

# Запустить профилировщик на 30 секунд
python3 share/aruco_det_loc/scripts/profiling/aruco_profiler.py \
  --record-only \
  --duration 30 \
  --label baseline \
  --output-dir ./profile_results
```

Результаты:
```
profile_results/
  baseline_detect.csv   — сырые данные детекции
  baseline_loc.csv      — сырые данные локализации
  report_YYYYMMDD_HHMMSS.md  — аналитический отчёт
```

### Режим `--full-test` (автоматический параметрический тест)

Скрипт перебирает все конфигурации из `PARAM_MATRIX`, меняя параметры
через ROS service `SetParameters`, и собирает данные по каждой:

```bash
python3 share/aruco_det_loc/scripts/profiling/aruco_profiler.py \
  --full-test \
  --duration-per-config 20 \
  --warmup 3 \
  --output-dir ./profile_results
```

> **Важно:** `--full-test` меняет параметры нод через SetParameters.
> Startup-only параметры (`cpu_affinity_ids`, `opencv_num_threads`) в матрицу
> не включены — их нужно задавать при запуске нод.

Список конфигураций в матрице:

| Label | detect params | loc params |
|---|---|---|
| `baseline` | aruco3=off, per_marker_pnp=off | flags=ITERATIVE, iterations=16, refine=off, cov=Jacobian |
| `aruco3_detect` | aruco3=on | то же |
| `sqpnp_solver` | — | flags=SQPNP |
| `aruco3_sqpnp` | aruco3=on | flags=SQPNP |
| `ransac_8iter` | — | iterations=8 |
| `ransac_32iter` | — | iterations=32 |
| `with_refine` | — | refine=on |
| `count_cov` | — | cov=marker_count (без Jacobian) |
| `per_marker_pnp` | per_marker_pnp=on | — |
| `aruco3_ratio005` | aruco3=on, ratio=0.05 | — |

---

## Интерпретация результатов

### Что смотреть в первую очередь

| Метрика | Хороший результат | Требует внимания |
|---|---|---|
| `detect_ms` | < 10 ms (30 fps) | > 33 ms (< 30 fps) |
| `pnp_ms` (loc) | < 5 ms | > 15 ms |
| `cov_ms` | < 2 ms | > 5 ms |
| `markers_age_ms` | < 5 ms | > 20 ms |
| `image_to_pose_ms` | < 30 ms | > 50 ms |

### Какие параметры влияют на какую фазу

| Параметр | Фаза |
|---|---|
| `use_aruco3_detection` | `detect_ms` (уменьшает для крупных маркеров) |
| `aruco3_min_marker_length_ratio` | `detect_ms` (меньше ratio → медленнее, обнаруживает мелкие) |
| `max_detect_fps` | ранний троттл частоты `detectMarkers`, `/aruco/det/markers` и одометрии |
| `estimate_marker_pose` | `pnp_ms` (detect) |
| `pnp_fps_cap` + `pnp_fps_cap_enabled` | только `pnp_ms` per-marker PnP для НЕ-картовых меток; не троттлит `detectMarkers` |
| `dictionary_id` | `detect_ms` (меньший словарь → быстрее) |
| `pnp.iterations` | `pnp_ms` (loc) |
| `pnp.flags` (0=ITERATIVE, 8=SQPNP) | `pnp_ms` (loc) |
| `refine_pose` | `refine_ms` (loc) |
| `put_markers_count_to_covariance` | `cov_ms` (loc) — полностью убирает Jacobian |
| `known_vertical_frame` | `tf_ms` (loc) |
| `cpu_affinity_ids` | все фазы (startup-only) |
| `opencv_num_threads` | `detect_ms` (startup-only) |

### Startup-only параметры (нельзя тестировать через `--full-test`)

Эти параметры применяются **только при старте ноды** и не меняются через
SetParameters:

- `cpu_affinity_ids` — привязка потока к конкретным ядрам CPU.
  Запустите `check_cpu_topology.sh`, найдите ID ядер Cortex-A76 (big-core).
  На Allwinner A733: `[0, 1]`.

- `opencv_num_threads` — число внутренних потоков OpenCV.
  Рекомендуется `2` (по числу A76 ядер).

Для их тестирования нужно перезапускать ноды с разными значениями и запускать
`--record-only` для каждого варианта.

### Проверка `max_detect_fps`

`max_detect_fps` можно менять во время работы. Для CPU-теста обычно сравнивают
baseline без лимита и целевой лимит 20–22 Гц:

```bash
ros2 param set /aruco_detect max_detect_fps 0.0
ros2 topic hz /aruco/det/markers
ros2 topic hz /aruco/world_pose
python3 share/aruco_det_loc/scripts/profiling/aruco_profiler.py \
  --record-only --duration 30 --label detect_unlimited --output-dir ./profile_results

ros2 param set /aruco_detect max_detect_fps 20.0
ros2 topic hz /aruco/det/markers           # ≈20 Hz
ros2 topic hz /aruco/world_pose  # ≈20 Hz when map pose is accepted
python3 share/aruco_det_loc/scripts/profiling/aruco_profiler.py \
  --record-only --duration 30 --label detect_20hz --output-dir ./profile_results
```

Ожидаемый результат: частота `/aruco/det/markers` и `/aruco/world_pose` снижается до
лимита, общая загрузка CPU падает, а `image_to_pose_ms` на обработанных кадрах
остаётся на уровне baseline.

### Как влияет `input_mono` (MONO8 zero-copy)

Если `input_mono: true` в detect JSON — нода использует zero-copy путь:
`cv_bridge::toCvShare` возвращает ссылку на буфер ROS без копирования.
Конвертация BGR→Gray полностью исключается.

Проверьте кодировку камеры:
```bash
bash share/aruco_det_loc/scripts/check_camera_encoding.sh /your/image/topic
```

### Как читать `markers_age_ms`

`markers_age_ms = ros_now(loc) − image_stamp`

Это сумма:
1. Время захвата кадра камерой до публикации в ROS (задержка драйвера)
2. Время работы `aruco_detect_node.imageCb` (`total_ms` detect)
3. Время DDS-сериализации и передачи топика `/aruco/det/markers`

Если `markers_age_ms` >> `total_ms(detect)` — узким местом является DDS или
задержка драйвера камеры. В IPC-режиме (composable container) шаг 3 → 0,
поэтому `markers_age_ms ≈ total_ms(detect) + camera_driver_latency`.

---

## Сценарии тестирования

### Сценарий 1: Базовая производительность

1. Запустить ноды в IPC-режиме (aruco_composed.launch.py)
2. Включить timing: `ros2 param set /aruco_detect enable_timing_log true`
3. `python3 aruco_profiler.py --record-only --duration 60 --label ipc_baseline`
4. Повторить с отдельными нодами для сравнения DDS overhead

### Сценарий 2: Подбор cpu_affinity_ids

1. Запустить ноды **без** привязки, собрать baseline
2. Запустить с `cpu_affinity_ids:=[0,1]` (A76 big cores)
3. Сравнить `detect_ms` p95 и p99 (должны уменьшиться и стабилизироваться)

### Сценарий 3: ArUco3 vs стандартный детектор

```bash
python3 aruco_profiler.py --full-test --duration-per-config 30 \
  --output-dir ./aruco3_test
```

Сравните `baseline` vs `aruco3_detect` в отчёте.

### Сценарий 4: Эффект ковариации

Сравните `baseline` (Jacobian covariance) vs `count_cov` (без Jacobian):
- `cov_ms` должен упасть до ~0
- `image_to_pose_ms` должен уменьшиться на `cov_ms` baseline

### Сценарий 5: Монокулярная камера с MONO8

1. Проверить кодировку: `check_camera_encoding.sh`
2. Если `mono8` — в JSON появится `"input_mono": true`
3. Сравнить `convert_ms` baseline (BGR→Gray) vs mono8 (zero-copy)

---

## Файлы вывода

```
profile_results/
  <label>_detect.csv         сырые JSON-поля по кадрам (detect)
  <label>_loc.csv            сырые JSON-поля по кадрам (loc)
  report_YYYYMMDD_HHMMSS.md  аналитический отчёт по всем конфигурациям
```

CSV-файл можно открыть в pandas для дополнительного анализа:

```python
import pandas as pd
df = pd.read_csv("profile_results/baseline_loc.csv")
print(df["image_to_pose_ms"].describe())
df["image_to_pose_ms"].hist(bins=50)
```
