# aruco_det_loc - aruco detect and localize

Пакет ROS 2 с нодами для детекции ArUco‑маркеров и локализации дрона (base_link) в мировой системе координат по заранее известной карте маркеров.

---

## Сборка

```bash
colcon build --packages-select aruco_det_loc
source install/setup.bash
```

---

## Состав пакета

### aruco_detect_node

Нода детекции ArUco‑маркеров на изображении камеры.

**Назначение:**
- Детекция ArUco на входном изображении
- Публикация найденных маркеров (ID, углы в пикселях, размер)
- Опциональная оценка pose маркера относительно камеры
- RViz‑визуализация

**Подписки:**
- `image_topic` — `sensor_msgs/Image`
- `camera_info_topic` — `sensor_msgs/CameraInfo`
- `map_markers_topic` — `aruco_det_loc/msg/MarkerArray`

**Публикации:**
- `/aruco/det/markers` — `aruco_det_loc/msg/MarkerArray`
- `/aruco/vis/markers` — `visualization_msgs/MarkerArray` (RViz, при `publish_visualization`)

**Параметры:**

| Параметр | Тип | По умолчанию | Описание |
|---|---|---|---|
| `marker_size` | double | `0.5` | Размер маркера в метрах (используется если маркер не найден в карте) |
| `dictionary_id` | int | `3` | ID словаря ArUco (`cv::aruco::PREDEFINED_DICTIONARY_NAME`). Совет: переключитесь на `0` (`DICT_4X4_50`), если все ваши маркеры имеют ID < 50 — это ускоряет детекцию на ~5–10 % |
| `estimate_marker_pose` | bool | `false` | Оценивать pose каждого маркера (solvePnP) и публиковать TF |
| `max_detect_fps` | double | `0.0` | Ранний лимит частоты всей детекции и публикации `/aruco/det/markers`. `0`/`<=0` = выключено. Можно менять на лету; фазовый аккумулятор держит среднюю частоту цели (например, камера ~30 Гц + cap 20 → ~20 Гц, не ~15) |
| `pnp_fps_cap_enabled` | bool | `true` | Ограничить частоту запуска solvePnP |
| `pnp_fps_cap` | int | `10` | Максимальная частота solvePnP в Гц |
| `use_map_markers` | bool | `true` | Подписываться на карту маркеров для получения размеров и фильтрации |
| `detect_individual_markers` | bool | `true` | Детектировать маркеры, которых нет в карте |
| `send_tf` | bool | `true` | Публиковать TF-трансформы для детектированных маркеров |
| `use_aruco3_detection` | bool | `false` | Включить ArUco3 — ускоренный многомасштабный детектор (2–5× быстрее). **Внимание:** маркеры меньше `aruco3_min_marker_length_ratio × ширина_кадра` не будут обнаружены |
| `aruco3_min_marker_length_ratio` | double | `0.10` | Минимальный размер стороны маркера как доля ширины кадра для ArUco3. Уменьшить если нужно детектировать мелкие маркеры (ценой снижения скорости) |
| `marker_id_whitelist` | int[] | `[]` | Белый список ID маркеров. Если список непустой — обрабатываются **только** маркеры из него. Пустой список (по умолчанию) = принимать все. Имеет приоритет над `use_map_markers` / `detect_individual_markers`. Можно менять на лету: `ros2 param set /aruco_detect marker_id_whitelist "[7]"`; новое значение применяется сразу |
| `cpu_affinity_ids` | int[] | `[]` | ID процессорных ядер для привязки потока ноды. Запустите `scripts/check_cpu_topology.sh` чтобы найти ID ядер Cortex-A76. Пример для Allwinner A733: `[0, 1]` |
| `opencv_num_threads` | int | `2` | Количество потоков внутреннего пула OpenCV. `2` = по числу ядер A76 (рекомендуется). `0` = полностью последовательный режим. `-1` = автовыбор OpenCV |
| `enable_timing_log` | bool | `false` | Публиковать timing-данные в `/aruco/timing` (`std_msgs/String` JSON). Используется `scripts/profiling/aruco_profiler.py`. Можно включать на лету: `ros2 param set /aruco_detect enable_timing_log true` |

> **Примечание по производительности.** Метод solvePnP использует `SOLVEPNP_IPPE_SQUARE` — аналитическое решение для квадратных планарных маркеров. Он в 20–80× быстрее итеративного метода. При наличии двух решений (планарная неоднозначность) выбирается то, что ближе к позе предыдущего кадра.

> **MONO8 / grayscale (П.4).** Если камера публикует изображение в кодировке `mono8` или `8UC1`, нода автоматически использует путь **zero-copy** — лишняя конвертация BGR→Gray исключается. Для проверки кодировки вашей камеры запустите `scripts/check_camera_encoding.sh`.

---

### aruco_loc_node

Нода локализации камеры в мировой системе координат по ArUco‑маркерам.

**Назначение:**
- Построение 3D–2D соответствий: углы маркеров из карты ↔ пиксели детектора
- Оценка глобальной pose камеры через RANSAC + solvePnP
- Публикация `PoseWithCovarianceStamped` для EKF / полётного контроллера

**Подписки:**
- `markers_topic` — `aruco_det_loc/msg/MarkerArray` (от aruco_detect_node)
- `map_markers_topic` — `aruco_det_loc/msg/MarkerArray` (от aruco_map_node)
- `camera_info_topic` — `sensor_msgs/CameraInfo`

**Публикации:**
- `pose_topic` — `geometry_msgs/PoseWithCovarianceStamped` (pose камеры в world frame)

**Параметры:**

| Параметр | Тип | По умолчанию | Описание |
|---|---|---|---|
| `world_frame_id` | string | `"map"` | Мировой фрейм координат. Используется как `header.frame_id` публикуемой pose |
| `pnp.iterations` | int | `16` | Число итераций RANSAC. При w≈0.95 инлайеров формула даёт N≈3; 16 — безопасный запас. Увеличить до 50–100 только при частых ложных детекциях (w < 0.7) |
| `pnp.reprojection_error_px` | double | `4.0` | Порог ошибки репроекции для инлайеров RANSAC (пиксели) |
| `pnp.confidence` | double | `0.99` | Желаемая вероятность нахождения корректного решения RANSAC |
| `pnp.flags` | int | `0` | Алгоритм solvePnP: `0` = ITERATIVE, `8` = SQPNP |
| `pnp.min_used_markers` | int | `0` | `<=0` auto: 1 метка → IPPE_SQUARE, ≥2 метки → RANSAC; `>0` legacy hard minimum |
| `pnp.min_inlier_points` | int | `0` | `<=0` auto-min-inliers; `>0` legacy hard minimum |
| `gating.max_inlier_rmse_px` | double | `10.0` | Единый RMSE-гейт для RANSAC и single-marker IPPE |
| `refine_pose` | bool | `false` | Дополнительный проход solvePnP на инлайерах после RANSAC |
| `known_vertical_frame` | string | `""` | Фрейм с известной вертикалью (IMU); если задан — roll/pitch берётся из него |
| `auto_flip` | bool | `false` | Автоматически исправлять переворот pose на 180° |
| `tf_parent_frame` | string | `"camera_optical_1"` | Родительский фрейм для публикации TF `aruco_map_detected` |
| `tf_child_frame` | string | `"aruco_map_detected"` | Дочерний фрейм TF |
| `publish_tf` | bool | `true` | Публиковать TF трансформацию |
| `cpu_affinity_ids` | int[] | `[]` | ID процессорных ядер для привязки потока ноды. Запустите `scripts/check_cpu_topology.sh` чтобы найти ID ядер Cortex-A76 |
| `enable_timing_log` | bool | `false` | Публиковать timing-данные в `/aruco/timing` (`std_msgs/String` JSON). Используется `scripts/profiling/aruco_profiler.py`. Можно включать на лету: `ros2 param set /aruco_loc enable_timing_log true` |

> **Примечание по frame_id.** Публикуемый `PoseWithCovarianceStamped.header.frame_id` установлен в `world_frame_id` (`"map"`), так как pose выражает позицию камеры в мировых координатах. Потребитель `px4_local_pose_publisher` игнорирует `frame_id` и использует собственные параметры фреймов, однако корректный `frame_id` необходим для nav2 / robot_localization / RViz.

---

### aruco_tf_node

Нода привязки к мировой системе координат и публикации позы `base_link` для PX4‑VPE.

**Назначение:**
- Получает позу камеры в поле меток (`/aruco/loc/pose_cov` = `T_aruco_camera`).
- Вычисляет/применяет offset и публикует позу тела дрона в мире.

**Подписки:**
- `pose_cov_topic` — `geometry_msgs/PoseWithCovarianceStamped` (от aruco_loc).
- TF: `camera→base_link` (статический) и `world→base_link` (EKF, только single‑режим).

**Публикации:**
- `world_pose_topic` (`/aruco/world_pose`) — `PoseWithCovarianceStamped`, поза `base_link`. **Контракт VPE.**
- `reset_counter_topic` (`/aruco/reset_counter`) — `UInt8`, reliable + transient_local, KeepLast 1.
- `map_offset_topic` (`/aruco/map_offset`) — `TransformStamped`, **latched**, offset `T_world_aruco` (`frame_id=world`, `child=aruco_map`). Публикуется в single‑режиме на каждом пересчёте offset; нужен offboard/координатору роя.
- TF: `world→aruco_map` (static, offset), `world→aruco_map_detected` (камера), `world→base_link_aruco` (дрон).

**Параметры:**

| Параметр | По умолч. | Описание |
|---|---|---|
| `swarm_mode` | `false` | `true`: публиковать `base_link` **прямо в общем `aruco_map`** (offset=identity, нуль общий для всех дронов → абсолютные сравнимые координаты). `false`: привязка к миру дрона (точка взлёта) через offset. **Задавать в launch, не в полёте.** |
| `camera_frame_id` | `camera_optical_1` | Оптический фрейм камеры (должен совпадать с static TF `base_link→camera_optical_1`). |
| `base_link_frame_id` | `base_link` | Тело дрона. |
| `world_frame_id` | `map` | Мировой ENU‑фрейм (single‑режим). |
| `aruco_map_frame_id` | `aruco_map` | Фрейм карты; в swarm‑режиме — выходной фрейм позы. |
| `map_offset_topic` | `/aruco/map_offset` | Топик latched‑offset. |
| `allow_offset_timeout_reset` | `false` | Пересчитывать offset по таймауту (инкрементит `reset_counter`). |
| `offset_timeout` | `3.0` | Секунды до пересчёта offset. |
| `initialize_at_origin` | `false` | Выбрать первую устойчивую ArUco-позу локальным нулём; при `false` совместить её с текущим `world→base_link`. |
| `offset_init_samples` | `15` | Число согласованных offset-кандидатов до первой публикации `/aruco/world_pose`. |
| `offset_candidate_translation_tolerance` | `0.10` m | Максимальное 3D-расхождение кандидатов в одной последовательности. |
| `offset_candidate_rotation_tolerance` | `0.10` rad | Максимальное угловое расхождение кандидатов в одной последовательности. |
| `covariance_xy_floor` | `0.0004` m² | Минимальная variance X/Y (standard deviation 2 cm). |
| `covariance_z_floor` | `0.0009` m² | Минимальная variance Z (standard deviation 3 cm). |
| `covariance_angle_floor` | `0.0027415568` rad² | Минимальная variance roll/pitch/yaw (standard deviation около 3°). |

В real launch используется `initialize_at_origin=false`, в SITL — `true`.
Подходит любой маркер карты. При `allow_offset_timeout_reset=false` offset
сохраняется после потери маркеров.

> Подробный разбор математики, дерева TF и режима роя — в
> [`ARUCO_ODOMETRY_EXPLAINED.md`](ARUCO_ODOMETRY_EXPLAINED.md).
> Настройка параметров/CPU и тесты корректности — в
> [`TUNING_AND_VALIDATION.md`](TUNING_AND_VALIDATION.md).

---

## Режим IPC-композиции (П.5)

Все три ноды (detect + loc + tf) запускаются в **одном процессе** через `rclcpp_components`:

```bash
# Платформенные настройки CPU/потоков/coarse — через cpu_params_file (НЕ через
# cpu_affinity_ids: это массив/startup-параметр, его нельзя передать launch-строкой).
ros2 launch aruco_det_loc aruco_composed.launch.py \
  image_topic:=/camera_1/image_raw \
  camera_info_topic:=/camera_1/camera_info \
  cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_rpi5.yaml

# Рой (общий фрейм aruco_map для всех дронов):
ros2 launch aruco_det_loc aruco_composed.launch.py swarm_mode:=true \
  cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_a733.yaml
```

**Преимущества:**
- Топик `/aruco/det/markers` передаётся без сериализации DDS и без копирования памяти (IPC zero-copy через `unique_ptr` publish).
- Меньше потоков ОС, ниже задержка между детекцией и локализацией.

**Ограничения:**
- `aruco_map_node` использует `transient_local` QoS и должен оставаться отдельным процессом — **не добавляйте его в этот контейнер**.
- IPC zero-copy работает только для топиков между нодами в **одном** контейнере.

---

## Диагностические скрипты (П.6, П.4)

### Определение ядер Cortex-A76

```bash
bash share/aruco_det_loc/scripts/check_cpu_topology.sh
```

Скрипт выводит топологию ядер, максимальные частоты и идентификаторы архитектуры. Для Allwinner A733 (2×A76 + 6×A55) пример вывода:

```
CPU  CLUSTER  MAX_FREQ_MHz  PART_ID  LIKELY_CORE
0    0        1800          0xD0B    A76 BIG
1    0        1800          0xD0B    A76 BIG
2    1        1416          0xD05    A55
...
 Use: cpu_affinity_ids:=[0,1]
```

### Проверка кодировки камеры

```bash
bash share/aruco_det_loc/scripts/check_camera_encoding.sh /your/image/topic
```

Если камера публикует `mono8` — нода автоматически использует zero-copy путь без дополнительных параметров.

---

## Профилирование (timing-логирование)

Обе ноды публикуют подробные timing-данные в JSON в топики `/aruco/timing` при включённом параметре `enable_timing_log`.

### Быстрый старт

```bash
# Включить timing на лету (ноды уже запущены)
ros2 param set /aruco_detect enable_timing_log true
ros2 param set /aruco_loc    enable_timing_log true

# Записать 30 секунд и получить отчёт
python3 share/aruco_det_loc/scripts/profiling/aruco_profiler.py \
  --record-only --duration 30 --label baseline --output-dir ./profile_results
```

### Автоматический параметрический тест

```bash
python3 share/aruco_det_loc/scripts/profiling/aruco_profiler.py \
  --full-test --duration-per-config 20 --output-dir ./profile_results
```

Скрипт перебирает конфигурации параметров через ROS SetParameters service и формирует сравнительный markdown-отчёт.

**Ключевые метрики в отчёте:**
- `detect_ms` — время детекции ArUco (узкое место при большом разрешении)
- `pnp_ms` (loc) — время hybrid PnP (RANSAC или single-marker IPPE)
- `cov_ms` — время оценки ковариации (Jacobian)
- `markers_age_ms` — задержка DDS + detect (от захвата кадра до поступления в ноду локализации)
- `image_to_pose_ms` — **сквозная задержка** от захвата кадра до публикации позиции

Полная документация: `scripts/profiling/PROFILING.md`

---

## Пример запуска

### Публикация камеры из Gazebo

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image

ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
```

### Статическая трансформация base_link → camera (пример для top-down камеры)

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0 \
  -0.707 -0.7071 0.0 0.0 \
  base_link camera_optical_1
```

> **Имена фреймов.** Стек по умолчанию использует оптический фрейм
> `camera_optical_1` (`parent_frame_id` / `tf_parent_frame` / `camera_frame_id`).
> Это имя **обязано** совпадать с child вашего static TF `base_link → camera_optical_1`,
> иначе `aruco_tf_node` не найдёт трансформацию и не опубликует `/aruco/world_pose`.
> Для Gazebo (где фрейм называется иначе) переопределите `parent_frame_id:=...`.

### Запуск в IPC-режиме (рекомендуется для производительности)

```bash
# Определить ID ядер A76
bash share/aruco_det_loc/scripts/check_cpu_topology.sh
# → выведет что-то вроде: cpu_affinity_ids:=[0,1]
```

Платформенные параметры (affinity/потоки/coarse) — это массивы и startup‑параметры,
их нельзя передать launch‑строкой. Для **composable**‑нод используется готовый
YAML через аргумент `cpu_params_file:` (он гарантированно доходит до нод в
контейнере). Готовые профили лежат в `config/`:

```yaml
# config/cpu_a733.yaml  (правьте cpu_affinity_ids под вывод check_cpu_topology.sh)
/**:
  ros__parameters:
    cpu_affinity_ids: [0, 1]
    opencv_num_threads: 2
    coarse_to_fine_enabled: true
    coarse_scale: 0.25
```

```bash
# Запустить все три ноды в одном процессе
ros2 launch aruco_det_loc aruco_composed.launch.py \
  cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_a733.yaml
```

Или через `--ros-args` напрямую при запуске отдельных нод:
```bash
ros2 run aruco_det_loc aruco_detect_node --ros-args \
  -p cpu_affinity_ids:=[0,1] \
  -p opencv_num_threads:=2 \
  -p marker_id_whitelist:=[0,1,2,3,4]
```

### Запуск отдельными нодами

```bash
ros2 run aruco_det_loc aruco_loc_node --ros-args \
  -p markers_topic:=/aruco/det/markers \
  -p map_markers_topic:=/aruco/map/markers \
  -p camera_info_topic:=/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info \
  -p world_frame_id:=world \
  -p camera_frame_id:=camera_optical_1 \
  -p base_link_frame_id:=base_link \
  -p auto_flip:=true \
  -p known_vertical_frame:=base_link
```

```bash
ros2 run aruco_det_loc aruco_detect_node --ros-args \
  -p image_topic:=/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/image \
  -p camera_info_topic:=/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/imager/camera_info \
  -p use_map_markers:=true \
  -p map_markers_topic:=/aruco/map/markers \
  -p dictionary_id:=3
```

### Включение/выключение изображений для дебага
Обратитесь к aruco_detect следующей командой, чтобы включить публикацию debug изображений:
```
rosservice call /toggle_debug std_srvs/srv/SetBool "{data: true}"
```
Обратите внимание, после выключения публикации топики debug изображений все еще будут в списке "ros2 topic list", но они больше не будут обновляться.

### Просмотр визуализации

Нода aruco_detect_node, в случае если параметр /publish_visualization = true, публикует сообщения для визаулиации в топик
/aruco/vis/markers. Для просмотра запустите rviz2, нажмите кнопку 'add' и на вкладке "By topic" выбрите соответствующий топик, установите параметр "Fixed Frame" в значение "camera_optical_1".

> Примечание: в SITL-образе (ros-base) rviz2 не предустановлен — при необходимости поставьте его вручную: `sudo apt install ros-humble-rviz2`.

Также, ноды aruco_detect_node и aruco_loc_node могут публиковать изображения с камеры для дебага, на которых нарисованы оси координат отдельных маркеров и ось начала координат всей маркерной карты соответственно. Для этого ноды должны быть запущены с параметрами
/publish_debug_image (название одинаково для обеих нод) в значении true. Увидеть изображения для дебага можно выбрав соответствующие топики в rviz2.
