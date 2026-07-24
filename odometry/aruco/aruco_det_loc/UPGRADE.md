# ArUco Detection & Localization — архитектура и заметки по реализации

Документ описывает устройство пакета `aruco_det_loc` и связанного
`px4_local_pose_publisher`: архитектуру нод, ключевые проектные решения детекции
и локализации, затронутые файлы и методику проверки.

---

## Содержание

1. [Архитектура блока ArUco](#1-архитектура-блока-aruco)
2. [Coarse-to-fine детекция](#2-coarse-to-fine-детекция)
3. [Оптимизация solvePnP в detect-ноде](#3-оптимизация-solvepnp-в-detect-ноде)
4. [Нода ArucoTfNode](#4-нода-arucotfnode)
5. [Устройство VpePublisher](#5-устройство-vpepublisher)
6. [IPC zero-copy в composable контейнере](#6-ipc-zero-copy-в-composable-контейнере)
7. [Guard OpenCV ArUco3](#7-guard-opencv-aruco3)
8. [Рекомендованные параметры запуска](#8-рекомендованные-параметры-запуска)
9. [Полный список параметров](#9-полный-список-параметров)

---

## 1. Архитектура блока ArUco

### Предыдущая схема

```
┌─────────────────────────────────────────────────────┐
│  aruco_container  (1 OS process)                    │
│                                                     │
│  ┌──────────────────┐    /aruco/det/markers (DDS IPC)         │
│  │ ArucoDetectNode  │──────────────────────────────►│
│  │                  │    • detect ALL markers       │
│  │ solvePnP:        │    • solvePnP for EACH marker │
│  │  ALL markers     │      (map + non-map) ← лишнее │
│  └──────────────────┘                               │
│                                                     │
│  ┌──────────────────┐    /aruco/loc/pose_cov (DDS)   │
│  │  ArucoLocNode    │──────────────────────────────►│
│  │                  │    • global solvePnP (map)    │
│  └──────────────────┘                               │
└─────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────┐
│  px4_local_pose_publisher  (отдельный процесс)       │
│                                                      │
│  ┌───────────────────────────────────────────────┐  │
│  │  VpePublisher                                  │  │
│  │  • TF lookups                                  │  │
│  │  • offset computation (aruco_map → world)      │  │
│  │  • ENU → NED                                   │  │
│  │  • PX4 update()                                │  │
│  └───────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────┘
```

### Текущая схема

```
Камера (1280×960)
    │ sensor_msgs/Image
    ▼
┌───────────────────────────────────────────────────────────────────────────┐
│  aruco_container  (1 OS process, intra-process comms)                     │
│                                                                           │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │  ArucoDetectNode                                                     │ │
│  │                                                                      │ │
│  │  1. cvtColor → gray (1280×960) — ОДИН РАЗ                           │ │
│  │  2. [опц.] resize → 320×240 (INTER_AREA)                            │ │
│  │  3. detectMarkers на 320×240 (CORNER_REFINE_NONE)                   │ │
│  │  4. scale corners ×4 → полное разрешение                            │ │
│  │  5. [опц.] cornerSubPix на gray 1280×960                            │ │
│  │  6. solvePnP только для non-map маркеров (опц. флаг)                │ │
│  └──────────────────────────────┬──────────────────────────────────────┘ │
│                                  │ /aruco/det/markers  unique_ptr  (IPC zero-copy)  │
│                                  ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │  ArucoLocNode                                                        │ │
│  │                                                                      │ │
│  │  • buildCorrespondences (только map-маркеры)                        │ │
│  │  • solvePnPRansac (глобальная поза камеры)                          │ │
│  │  • T_world_camera → PoseWithCovarianceStamped                       │ │
│  └──────────────────────────────┬──────────────────────────────────────┘ │
│                                  │ /aruco/loc/pose_cov  unique_ptr  (IPC)  │
│                                  ▼                                        │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │  ArucoTfNode                    ◄── НОВАЯ НОДА                      │ │
│  │                                                                      │ │
│  │  • вычисляет offset: T_world_aruco_map                              │ │
│  │  • применяет offset: T_world_camera = offset * T_aruco_camera       │ │
│  │  • T_world_base = T_world_camera * T_camera_base                    │ │
│  │                                                                      │ │
│  │  Публикует:                                                          │ │
│  │    static TF:   world → aruco_map          (offset)                 │ │
│  │    dynamic TF:  world → aruco_map_detected (поза камеры)            │ │
│  │    dynamic TF:  world → base_link_aruco    (поза дрона)             │ │
│  │    topic:       /aruco/world_pose  unique_ptr                        │ │
│  │    topic:       /aruco/reset_counter  (UInt8)                        │ │
│  └──────────────────────────────┬──────────────────────────────────────┘ │
└─────────────────────────────────┼─────────────────────────────────────────┘
                                  │ /aruco/world_pose  (DDS, между процессами)
                                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  px4_local_pose_publisher  (отдельный процесс)                              │
│                                                                             │
│  ┌─────────────────────────────────────────────────────────────────────┐   │
│  │  VpePublisher  (упрощён)                                             │   │
│  │                                                                      │   │
│  │  • получает world_pose (ENU) готовую позу base_link                 │   │
│  │  • конвертирует ENU → NED                                            │   │
│  │  • update() → PX4 EKF2                                              │   │
│  └─────────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Поток данных и zero-copy

| Топик | Publisher → Subscriber | Тип | IPC zero-copy |
|---|---|---|---|
| `/aruco/det/markers` | detect → loc | `unique_ptr<MarkerArray>` | да |
| `/aruco/loc/pose_cov` | loc → tf | `unique_ptr<PoseWithCovStamped>` | да |
| `/aruco/world_pose` | tf → VPE | `unique_ptr<PoseWithCovStamped>` | разные процессы |
| `/aruco/reset_counter` | tf → VPE | `UInt8` | разные процессы |

---

## 2. Coarse-to-fine детекция

**Файл:** `src/aruco_detect_node.cpp`

### Суть

| Шаг | Что происходит |
|---|---|
| 1 | `cvtColor` → `gray` (1280×960) — **один раз**, без цветной копии |
| 2 | `cv::resize(gray, coarse_gray, {320,240}, INTER_AREA)` |
| 3 | `detectMarkers` на `coarse_gray` с `CORNER_REFINE_NONE` |
| 4 | Умножаем все углы и `rejected` на `scale_inv = 1/coarse_scale` |
| 5 | `cornerSubPix(gray, flat_corners, Size(hw,hw), ...)` на полном разрешении |
| 6 | Уточнённые углы записываются обратно в `corners[i][k]` |

`cornerSubPix` принимает **весь** полноразмерный кадр, не ROI. `winSize = subpix_win_size`
(полуокно) покрывает неопределённость ±4–6 px от масштабирования ×4.

### Новые параметры

| Параметр | Тип | По умолч. | Описание |
|---|---|---|---|
| `coarse_to_fine_enabled` | bool | `false` | Включить механизм |
| `coarse_scale` | double | `0.25` | Доля от исходного размера (0.25 = 320×240 при 1280×960) |
| `corner_refine_pnp_only` | bool | `false` | `true` = рефайнить только маркеры из whitelist/map |
| `subpix_win_size` | int | `6` | Полуокно `cornerSubPix` в пикселях |

### Компромисс

Маркеры с проекцией стороны **< 16–20 px** в 320×240 не будут найдены —
это ожидаемо. Точность `solvePnP` растёт пропорционально точности углов.

### Методика тестирования

**A1. Проверка включения/выключения без потери детекций:**
```bash
# Запустить ноду с 1280×960 камерой
ros2 param set /aruco_detect coarse_to_fine_enabled false
# Записать bag: маркеры на расстоянии 1–3 м
ros2 bag record /aruco_detect/markers /aruco/timing -o baseline.bag

ros2 param set /aruco_detect coarse_to_fine_enabled true
ros2 bag record /aruco_detect/markers /aruco/timing -o coarse_fine.bag

# Сравнить n_detected в timing JSON:
ros2 bag play baseline.bag --topics /aruco/timing | \
  python3 -c "import sys,json; [print(json.loads(l.split('data:')[1].strip())) for l in sys.stdin if 'data:' in l]"
```

**A2. Замер прироста точности позы:**
```bash
# Включить timing log
ros2 param set /aruco_detect enable_timing_log true

# Слушать refine_ms (= 0 при выключенном рефайне, > 0 при включённом)
ros2 topic echo /aruco/timing | grep -o '"refine_ms":[0-9.]*'

# Сравнить reprojection RMSE из aruco_loc:
ros2 topic echo /aruco/timing | grep -o '"reproj_rmse":[0-9.]*'
```

**A3. Стресс-тест нагрузки:**
```bash
# Режим A: 1280×960, detect на 320, refine ВЫКЛ
ros2 param set /aruco_detect coarse_to_fine_enabled true
ros2 param set /aruco_detect corner_refine_pnp_only false
# (посмотреть что refine_ms ≈ 0 в timing)

# Режим B: refine ВКЛ
# (посмотреть refine_ms и сравнить total_ms)

# CPU load:
top -b -n 60 -d 1 | grep aruco > cpu_log.txt

# Или через taskset если affinity настроен:
pidstat -u -p $(pgrep -f aruco_container) 1 60
```

**A4. Визуальная проверка:**
```bash
ros2 param set /aruco_detect publish_debug_image true
ros2 run rqt_image_view rqt_image_view
# Открыть /aruco/vis/debug_image — углы должны лежать точно на углах маркеров
```

---

## 3. Оптимизация solvePnP в detect-ноде

**Файл:** `src/aruco_detect_node.cpp`

### Суть

До изменения: `solvePnP` запускался **для каждого** обнаруженного маркера,
включая те, что в карте. `aruco_loc_node` затем снова делал `solvePnPRansac`
по тем же маркерам карты — двойная работа.

После изменения:

```
is_map_marker  →  пропустить PnP в detect (loc делает это глобально)
!is_map_marker →  PnP только если pnp_non_map_markers = true
```

Углы маркеров карты **по-прежнему публикуются** в `/aruco/det/markers` и доходят до
`aruco_loc_node` — он использует `corners`, а не `pose`.

### Новый параметр

| Параметр | Тип | По умолч. | Описание |
|---|---|---|---|
| `pnp_non_map_markers` | bool | `false` | Запускать solvePnP для маркеров **вне** карты |

### Методика тестирования

**B1. Проверка что loc-нода получает корректные углы:**
```bash
# Подписаться на маркеры и убедиться что corners не нули
ros2 topic echo /aruco/det/markers | grep -A 20 "corners"

# Позиция из loc-ноды должна быть стабильной:
ros2 topic echo /aruco/loc/pose_cov | grep -A 3 "position"
```

**B2. Измерение экономии CPU:**
```bash
# Включить timing в detect-ноде
ros2 param set /aruco_detect enable_timing_log true

# pnp_non_map_markers = false (по умолчанию): pnp_calls_fresh должен = 0
ros2 topic echo /aruco/timing | python3 -c "
import sys, json
for line in sys.stdin:
    if 'data:' in line:
        d = json.loads(line.split('data:')[1].strip().strip('\"').replace('\\\\\"','\"'))
        print(f'pnp_fresh={d[\"pnp_calls_fresh\"]}  pnp_ms={d[\"pnp_ms\"]:.2f}ms')
"
# Ожидаем: pnp_calls_fresh=0, pnp_ms≈0
```

**B3. TF для non-map маркеров (если нужны):**
```bash
ros2 param set /aruco_detect pnp_non_map_markers true
ros2 run tf2_ros tf2_echo map aruco_99  # для маркера ID=99 вне карты
```

---

## 4. Новая нода ArucoTfNode

**Файлы:**
- `src/aruco_tf_node.cpp`
- `include/aruco_det_loc/aruco_tf_node.hpp`
- `CMakeLists.txt` (добавлен `aruco_tf_component`)

### Суть

Вся TF-логика вынесена из `VpePublisher` в отдельный composable-компонент.

**Входы:**
- `pose_cov_topic` — `PoseWithCovarianceStamped` от `aruco_loc_node`

**Выходы:**

| Тип | Topic / Frame | Описание |
|---|---|---|
| Static TF | `world_frame → aruco_map_frame` | Оффсет; переиздаётся при каждом сбросе |
| Dynamic TF | `world_frame → aruco_detected_frame` | Поза камеры в world-frame |
| Dynamic TF | `world_frame → base_aruco_frame` | Поза `base_link` от ArUco |
| Topic | `world_pose_topic` | `PoseWithCovStamped` (ENU, base_link в world) |
| Topic | `reset_counter_topic` | `UInt8` для PX4 EKF2 |

### Логика вычисления оффсета

```
T_world_aruco_map = T_world_base * T_base_camera * T_camera_aruco_map

где:
  T_world_base      — из TF дерева (world → base_link)
  T_base_camera     — из TF дерева (camera → base_link, инвертируется)
  T_camera_aruco    — обратная к входной T_aruco_camera
```

При получении нового `pose_cov`:
```
T_world_camera = offset * T_aruco_camera
T_world_base   = T_world_camera * T_camera_base
```

### Параметры

| Параметр | По умолч. | Описание |
|---|---|---|
| `pose_cov_topic` | `/aruco/loc/pose_cov` | Вход от aruco_loc |
| `world_pose_topic` | `/aruco/world_pose` | Выход для VPE |
| `reset_counter_topic` | `/aruco/reset_counter` | Выход для VPE |
| `camera_frame_id` | `main_camera_optical` | |
| `base_link_frame_id` | `base_link` | |
| `world_frame_id` | `map` | |
| `aruco_map_frame_id` | `aruco_map` | Child статического TF |
| `aruco_detected_frame_id` | `aruco_map_detected` | Child динамического TF (камера) |
| `base_aruco_frame_id` | `base_link_aruco` | Child динамического TF (дрон) |
| `allow_offset_timeout_reset` | `false` | Пересчитывать оффсет по таймауту |
| `offset_timeout` | `3.0` | Секунды до пересчёта оффсета |

Все параметры меняются через `ros2 param set` в runtime.

### Методика тестирования

**C1. Проверка статического TF (оффсет):**
```bash
# Первый кадр с маркерами должен опубликовать статический TF
ros2 run tf2_ros tf2_echo map aruco_map
# Ожидаем: трансляция близкая к нулю если дрон стартовал над картой,
# или ненулевую если aruco_map ≠ map

# Убедиться что TF стабилен между кадрами:
ros2 topic echo /aruco/reset_counter
# Счётчик должен инкрементироваться только при пересчёте оффсета
```

**C2. Проверка динамических TF:**
```bash
ros2 run tf2_tools view_frames
# В pdf увидеть: map → aruco_map (static), map → aruco_map_detected, map → base_link_aruco

# Частота динамических TF должна совпадать с частотой камеры (~30 Гц):
ros2 topic hz /tf
```

**C3. Проверка корректности позы base_link:**
```bash
ros2 topic echo /aruco/world_pose | head -30
# position должна совпадать с реальным положением дрона в world-frame

# Сравнить с TF:
ros2 run tf2_ros tf2_echo map base_link_aruco
# Должна совпадать с /aruco/world_pose position/orientation
```

**C4. Тест пересчёта оффсета:**
```bash
ros2 param set /aruco_tf allow_offset_timeout_reset true
ros2 param set /aruco_tf offset_timeout 5.0
# Подождать 5 секунд — в логах увидеть "Offset (re)computed"
# reset_counter должен инкрементироваться
ros2 topic echo /aruco/reset_counter
```

---

## 5. Упрощение VpePublisher

**Файлы:**
- `src/pose_subscriber_tf2.cpp`
- `include/px4_local_pose_publisher/pose_subscriber_tf2.hpp`

### Суть

| Было | Стало |
|---|---|
| TF buffer + listener | удалено |
| `offset_` (T_world_aruco_map) | удалено |
| `computeOffset()` | удалено |
| `applyOffset()` | удалено |
| Воркер-тред + очередь | удалено |
| Подписка на VehicleLocalPosition | удалено |
| Подписка на `pose_cov` | → подписка на `/aruco/world_pose` |
| ENU→NED конвертация | осталась |
| `update()` → PX4 | осталась |
| Fallback с высокой дисперсией | осталась |

Входящая поза `/aruco/world_pose` уже в `world_frame` (ENU) с применённым оффсетом —
VpePublisher только конвертирует и отправляет.

### Новые параметры

| Параметр | По умолч. | Описание |
|---|---|---|
| `world_pose_topic` | `/aruco/world_pose` | Вход: поза base_link в world (ENU) |
| `reset_counter_topic` | `/aruco/reset_counter` | Вход: счётчик сброса EKF2 |
| `allow_send_vio` | `false` | Разрешить отправку в PX4 |
| `publish_rate_hz` | `30.0` | Частота таймера отправки |
| `data_timeout_sec` | `0.5` | Таймаут до fallback |
| `fallback_variance` | `0.5` | Дисперсия в fallback измерении |

### Методика тестирования

**D1. Проверка получения данных от TF-ноды:**
```bash
# Убедиться что VPE нода получает данные
ros2 topic hz /aruco/world_pose   # должна совпадать с частотой камеры
ros2 topic hz /aruco/reset_counter

# Отладка таймаута fallback:
ros2 param set /vpe_publisher data_timeout_sec 0.1
# Остановить aruco_tf_node, убедиться что в логах появится fallback
```

**D2. Проверка корректности NED-конвертации:**
```bash
# Записать world_pose (ENU) и сравнить с VehicleLocalPosition (NED)
ros2 bag record /aruco/world_pose /fmu/out/vehicle_local_position -d 30 -o enu_ned_check.bag

# Проверка: если world_pose.position = [x_enu, y_enu, z_enu], то
# VehicleLocalPosition должна показывать [y_enu, x_enu, -z_enu] (ENU→NED)
```

**D3. Безопасный тест на реальном дроне (allow_send_vio = false):**
```bash
# Сначала гонять с выключенной отправкой, смотреть логи
ros2 param set /vpe_publisher allow_send_vio false
ros2 launch aruco_det_loc aruco_composed.launch.py &
ros2 run px4_local_pose_publisher pose_subscriber_tf2 &
ros2 topic echo /aruco/world_pose  # убедиться что поза корректна

# Только после верификации:
ros2 param set /vpe_publisher allow_send_vio true
```

---

## 6. IPC zero-copy в composable контейнере

**Файлы:**
- `launch/aruco_composed.launch.py`
- `src/aruco_loc_node.cpp` (одна правка)

### Суть

`aruco_loc_node.cpp` — публикация `pose_cov` изменена с value на `unique_ptr`:
```cpp
// Было:
pose_cov_pub_->publish(pose_cov_msg);

// Стало:
pose_cov_pub_->publish(
    std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_cov_msg));
```

Это необходимо для IPC zero-copy: rclcpp передаёт указатель без копии только если
publisher использует `unique_ptr`, subscriber — `SharedPtr` колбэк, и оба узла
в одном контейнере с `use_intra_process_comms: true`.

`aruco_composed.launch.py` добавляет `ArucoTfNode` третьим компонентом.

### Методика тестирования

**E1. Подтверждение что IPC активен:**
```bash
# При IPC zero-copy топик НЕ виден снаружи контейнера через ros2 topic echo
# Проверить: если IPC работает, /aruco/det/markers НЕ будет идти через DDS
ros2 topic bw /aruco/det/markers
# Если IPC работает — bandwidth ≈ 0 (трафика нет в DDS)
# Если IPC НЕ работает — bandwidth будет ненулевой

# Дополнительно: включить логи rclcpp
RCUTILS_LOGGING_SEVERITY_THRESHOLD=10 ros2 launch aruco_det_loc aruco_composed.launch.py 2>&1 | grep -i "intra"
```

**E2. Проверка что все три ноды в одном процессе:**
```bash
ros2 component list
# Вывод должен показать:
# /aruco_container
#   1  /aruco_detect
#   2  /aruco_loc
#   3  /aruco_tf
```

**E3. Latency до публикации позы:**
```bash
ros2 topic echo /aruco/timing | python3 -c "
import sys, json
for line in sys.stdin:
    if 'data:' in line:
        d = json.loads(line.split('data:')[1].strip().strip('\"').replace('\\\\\"','\"'))
        print(f'total_det={d[\"total_ms\"]:.1f}ms')
"
ros2 topic echo /aruco/timing | python3 -c "
import sys, json
for line in sys.stdin:
    if 'data:' in line:
        d = json.loads(line.split('data:')[1].strip().strip('\"').replace('\\\\\"','\"'))
        print(f'image_to_pose={d[\"image_to_pose_ms\"]:.1f}ms')
"
```

---

## 7. Guard OpenCV ArUco3

**Файл:** `src/aruco_detect_node.cpp`

### Суть

Поля `useAruco3Detection` и `minMarkerLengthRatioOriginalImg` появились в OpenCV 4.7.
Ubuntu 22.04 (ROS2 Humble) поставляет OpenCV 4.5.4.

```cpp
#define ARUCO3_SUPPORTED \
  (CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7))

// Все 5 вхождений обёрнуты:
#if ARUCO3_SUPPORTED
  detector_params_->useAruco3Detection = use_aruco3_detection_;
  ...
#else
  if (use_aruco3_detection_) {
    RCLCPP_WARN(..., "ArUco3 not supported in OpenCV %d.%d.%d", ...);
  }
#endif
```

### Методика тестирования

```bash
# Проверить версию OpenCV в пакете:
python3 -c "import cv2; print(cv2.__version__)"

# Убедиться что сборка проходит без ошибок:
colcon build --packages-select aruco_det_loc

# На OpenCV < 4.7: параметр принимается, но предупреждение в логах
ros2 param set /aruco_detect use_aruco3_detection true
ros2 topic echo /rosout | grep -i aruco3
# Ожидаем: "ArUco3 not supported in OpenCV X.Y.Z"
```

---

## 8. Рекомендованные параметры запуска

### Hardware: Allwinner A733 (2×A76 @ 2.0 ГГц + 6×A55), камера 1280×960 @ 30 fps

```bash
ros2 launch aruco_det_loc aruco_composed.launch.py \
  \
  # Входные топики
  image_topic:=/camera/image_raw \
  camera_info_topic:=/camera/camera_info \
  map_markers_topic:=/aruco/map/markers \
  \
  # Размер и словарь маркеров
  marker_size:=0.5 \
  dictionary_id:=3 \
  \
  # ── Coarse-to-fine ──────────────────────────────────────
  coarse_to_fine_enabled:=true \
  coarse_scale:=0.25 \
  # Рефайнить все маркеры (не только map):
  corner_refine_pnp_only:=false \
  # Полуокно cornerSubPix — покрывает ±4–6 px от масштабирования ×4:
  subpix_win_size:=6 \
  \
  # ── solvePnP в detect-ноде ───────────────────────────────
  # Маркеры карты: PnP делает loc-нода, не detect
  estimate_marker_pose:=false \
  pnp_non_map_markers:=false \
  \
  # ── Localisation PnP ────────────────────────────────────
  # 16 итераций RANSAC достаточно при inlier rate ≥ 0.95
  pnp_iterations:=16 \
  pnp_reprojection_error_px:=4.0 \
  pnp_min_used_markers:=3 \
  pnp_min_inlier_points:=4 \
  \
  # ── Фреймы ──────────────────────────────────────────────
  parent_frame_id:=main_camera_optical \
  world_frame_id:=map \
  base_link_frame_id:=base_link \
  aruco_map_frame_id:=aruco_map \
  aruco_detected_frame_id:=aruco_map_detected \
  base_aruco_frame_id:=base_link_aruco \
  \
  # ── TF оффсет ───────────────────────────────────────────
  allow_offset_timeout_reset:=false \
  offset_timeout:=3.0 \
  \
  # ── CPU affinity — A76 ядра ──────────────────────────────
  # Запустить scripts/check_cpu_topology.sh чтобы найти ID ядер A76
  # cpu_affinity_ids:=[0,1] \
  opencv_num_threads:=2
```

### VpePublisher (отдельный процесс)

```yaml
# config/vpe.yaml
vpe_publisher:
  ros__parameters:
    world_pose_topic:    /aruco/world_pose
    reset_counter_topic: /aruco/reset_counter
    allow_send_vio:      false        # true только после верификации позы
    publish_rate_hz:     30.0
    data_timeout_sec:    0.5
    fallback_variance:   0.5
```

```bash
ros2 run px4_local_pose_publisher pose_subscriber_tf2 \
  --ros-args --params-file config/vpe.yaml
```

### Параметры под задачи

#### Только карта, максимальная скорость
```yaml
coarse_to_fine_enabled:    true
coarse_scale:              0.25
corner_refine_pnp_only:    true   # рефайнить только map-маркеры
estimate_marker_pose:      false
pnp_non_map_markers:       false
```

#### Карта + TF для отдельных маркеров
```yaml
estimate_marker_pose:      true
pnp_non_map_markers:       true
pnp_fps_cap_enabled:       true
pnp_fps_cap:               10     # не чаще 10 Hz для non-map PnP
```

#### Отладка с дебаг-изображением
```yaml
publish_debug_image:       true
publish_visualization:     true
enable_timing_log:         true
```

---

## 9. Полный список новых параметров

### `aruco_detect_node`

| Параметр | Тип | По умолч. | Нода | Добавлен |
|---|---|---|---|---|
| `coarse_to_fine_enabled` | bool | `false` | detect | новый |
| `coarse_scale` | double | `0.25` | detect | новый |
| `corner_refine_pnp_only` | bool | `false` | detect | новый |
| `subpix_win_size` | int | `6` | detect | новый |
| `pnp_non_map_markers` | bool | `false` | detect | новый |

### `aruco_tf_node` (новая нода)

| Параметр | Тип | По умолч. |
|---|---|---|
| `pose_cov_topic` | string | `/aruco/loc/pose_cov` |
| `world_pose_topic` | string | `/aruco/world_pose` |
| `reset_counter_topic` | string | `/aruco/reset_counter` |
| `camera_frame_id` | string | `main_camera_optical` |
| `base_link_frame_id` | string | `base_link` |
| `world_frame_id` | string | `map` |
| `aruco_map_frame_id` | string | `aruco_map` |
| `aruco_detected_frame_id` | string | `aruco_map_detected` |
| `base_aruco_frame_id` | string | `base_link_aruco` |
| `allow_offset_timeout_reset` | bool | `false` |
| `offset_timeout` | double | `3.0` |

### `vpe_publisher` (упрощён)

| Параметр | Тип | По умолч. | Изменён |
|---|---|---|---|
| `world_pose_topic` | string | `/aruco/world_pose` | новый |
| `reset_counter_topic` | string | `/aruco/reset_counter` | новый |
| `allow_send_vio` | bool | `false` | остался |
| `publish_rate_hz` | double | `30.0` | остался |
| `data_timeout_sec` | double | `0.5` | остался |
| `fallback_variance` | float | `0.5` | остался |
| `pose_cov_topic` | — | — | удалён |
| `camera_frame_id` | — | — | удалён |
| `offset_frame_id` | — | — | удалён |

---

## Быстрый cheatsheet — runtime команды

```bash
# Включить coarse-to-fine
ros2 param set /aruco_detect coarse_to_fine_enabled true

# Посмотреть тайминги
ros2 topic echo /aruco/timing
ros2 topic echo /aruco/timing

# Сбросить оффсет TF вручную
ros2 param set /aruco_tf allow_offset_timeout_reset true
ros2 param set /aruco_tf offset_timeout 0.1   # спровоцировать сброс
# Затем вернуть:
ros2 param set /aruco_tf allow_offset_timeout_reset false

# Включить дебаг
ros2 service call /aruco_detect/toggle_debug std_srvs/srv/SetBool "{data: true}"

# Проверить компоненты в контейнере
ros2 component list

# Посмотреть TF дерево
ros2 run tf2_tools view_frames && evince frames.pdf
```
