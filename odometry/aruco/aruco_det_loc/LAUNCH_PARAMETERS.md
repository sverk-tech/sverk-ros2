# Launch-параметры ArUco — полный справочник

Справочник по всем входным параметрам launch-файлов пакетов `launch_system`
(верхний), `aruco_det_loc` (detect/loc/tf) и `aruco_map`. Для каждого параметра:
что это, как настраивать, допустимые пределы (min/max), дефолт и источник.

## Как это устроено (важно для понимания дефолтов)

```
launch_system/aruco.launch.py   ← ТЫ запускаешь это
   ├─ include aruco_map/aruco_map.launch.py          (карта, отдельный процесс)
   └─ include aruco_det_loc/aruco_composed.launch.py (detect+loc+tf, один контейнер)
                  └─ + cpu_params_file (YAML, переопределяет per-key параметры)
```

- **`launch_system/aruco.launch.py`** — верхний файл, его дефолты приоритетны для
  твоей сборки. Значения отсюда передаются в `aruco_composed`.
- **`aruco_composed.launch.py`** имеет СВОИ дефолты (применяются, если запускать его
  напрямую). Когда его включает `launch_system`, выигрывают значения `launch_system`.
- **`cpu_params_file`** (YAML, напр. `config/cpu_rpi5.yaml`) добавляется ПОСЛЕ и
  переопределяет coarse/threads/affinity на уровне нод.
- Колонка **Дефолт** ниже = значение в `launch_system` (если параметр там есть),
  иначе дефолт `aruco_composed`/ноды (помечено).
- Колонка **Min/Max** = жёсткие пределы (валидация в коде) где есть; иначе —
  рекомендуемый рабочий диапазон (помечено «реком.»).

Сокращения источников: **код** = C++-нода (значение/валидация), **OpenCV** =
докум. cv::aruco/solvePnP, **анализ** = `TUNING_AND_VALIDATION.md`/`ARUCO_ODOMETRY_EXPLAINED.md`,
**контракт** = требования камеры/PX4-VPE, **платформа** = RPi5/A733.

---

## 1. Топики

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `image_topic` | `/camera_1/image_raw` | строка | Вход: сырое изображение для детекции (`sensor_msgs/Image`, BGR8/MONO8). Меняй под реальный топик камеры (для Gazebo — sim-топик). | контракт камеры |
| `camera_info_topic` | `/camera_1/camera_info` | строка | Вход: интринсики камеры (`sensor_msgs/CameraInfo`). Должен соответствовать `image_topic`. QoS подписки в нодах — reliable+transient_local. | контракт камеры |
| `map_markers_topic` | `/aruco/map/markers` | строка | Карта маркеров detect↔loc↔map (`MarkerArray`, transient_local). Менять только при конфликте имён. | код |
| `markers_topic` | `/aruco/det/markers` | строка | Детекции detect→loc (углы меток в пикселях). | код |
| `pose_cov_topic` | `/aruco/loc/pose_cov` | строка | Поза камеры в поле меток loc→tf (`PoseWithCovarianceStamped`). | код |
| `world_pose_topic` | `/aruco/world_pose` | строка | **Выход для VPE**: поза base_link в мире/ENU. Не переименовывай без синхронного изменения VPE. | контракт VPE |
| `reset_counter_topic` | `/aruco/reset_counter` | строка | **Выход для VPE**: счётчик сбросов EKF2 (`UInt8`, latched). | контракт VPE |
| `map_offset_topic` | `/aruco/map_offset` *(composed)* | строка | Latched-offset `T_world_aruco` (`TransformStamped`) для offboard/роя. Публикуется только в single-режиме. | анализ (рой) |

---

## 2. Фреймы (TF)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `tf_parent_frame` / `parent_frame_id` | `camera_optical_1` | непустая строка | Оптический фрейм камеры. **ОБЯЗАН** совпадать с child твоего static TF `base_link→camera_optical_1`, иначе tf-нода молчит. | контракт; код (валидация непустоты) |
| `world_frame_id` | `map` | строка | Мировой ENU-фрейм (от EKF/одометрии). В single-режиме — фрейм выхода. | контракт VPE |
| `base_link_frame_id` | `base_link` | строка | Тело дрона. | код |
| `camera_frame_id` | `` (пусто) *(composed/loc)* | строка | У loc: если пусто — берётся из frame_id детекций. Обычно не задавать. | код |
| `tf_child_frame` | `aruco_map_detected` *(composed)* | строка | Child динамического TF от loc. В composed loc запущен с `publish_tf=false`, так что не публикуется (во избежание коллизии). | код |
| `tf_map_frame` | `aruco_map` *(composed)* | строка | Parent статических TF меток `aruco_<id>` от loc. | код |
| `aruco_map_frame_id` | `aruco_map` *(composed)* | строка | Child static-TF offset; в swarm-режиме — фрейм выхода. | код |
| `aruco_detected_frame_id` | `aruco_map_detected` *(composed)* | строка | Child динамического TF «камера в мире» от tf-ноды. | код |
| `base_aruco_frame_id` | `base_link_aruco` *(composed)* | строка | Child динамического TF «дрон в мире» от tf-ноды. | код |

---

## 3. Карта маркеров (пакет `aruco_map`)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `mode` | `debug` *(launch_system)* / `normal` *(map)* | `normal`\|`debug` | `debug` включает отрисовку debug-изображений плоскостей (лишний CPU). Для рабочих прогонов — `normal`. | код |
| `map_marker_size` / `marker_size` (map) | `0.16` | >0 (м) | Запасной размер метки, если в файле карты size≤0. У тебя все метки имеют свой size, так что не используется. | код (валидация >0) |
| `map_file` | `…/aruco_map/config/markers.txt` | путь | Файл карты `id size x y z rot_z rot_y rot_x` (метры/радианы). Главный источник геометрии — сверяй с физикой. | код |
| `topic_name` | `/aruco/map/markers` | строка | Имя публикуемого топика карты. | код |
| `frame_id` (map) | `map` *(через world_frame_id)* | строка | Фрейм карты (= world для loc). | код |
| `debug_aruco_dictionary` | `DICT_4X4_1000` | enum-имя | Словарь для ОТРИСОВКИ debug-плоскостей. Должен совпадать с `dictionary_id` детектора. | код; OpenCV |
| `debug_aruco_border_bits` | `1` | ≥1 | Толщина рамки маркера при отрисовке. | код (валидация ≥1) |
| `debug_draw_marker_ids` | `false` | bool | Подписывать ID на debug-изображении. | код |
| `plane_angle_threshold_deg` | `10.0` | 0–90 (реком. 5–15) | Порог угла для кластеризации меток в плоскость (debug). | код |
| `plane_distance_threshold_m` | `0.05` | ≥0 (реком. 0.02–0.1) | Порог расстояния для плоскости (debug). | код |
| `debug_pixels_per_meter` | `500.0` | ≥1 | Разрешение debug-изображения, px/м. | код |
| `debug_image_margin_px` | `40` | ≥0 | Поля debug-изображения. | код |
| `debug_image_max_size_px` | `4096` | ≥256 | Ограничение размера debug-изображения. | код |
| `debug_image_topic_prefix` | `/aruco/vis/map_debug_image/plane_` | строка | Префикс топиков debug-плоскостей. | код |
| `debug_image_publish_rate_hz` | `1.0` | ≥0 (0=выкл переиздание) | Частота переиздания debug-изображений. | код |
| `debug_draw_origin` | `true` | bool | Рисовать оси начала координат на debug. | код |
| `debug_origin_axis_length` | `0.5` | >0 (м) | Длина осей начала координат. | код |
| `debug_origin_axis_width_px` | `3` | ≥1 | Толщина осей. | код |

---

## 4. Детекция (нода `aruco_detect`)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `dictionary_id` | `3` (DICT_4X4_1000) | 0–16 (см. приложение) | Словарь ArUco. **Должен совпадать со словарём печатной карты.** Меньший словарь (напр. `1`=DICT_4X4_100) надёжнее и чуть быстрее — но требует перепечатки. | код; OpenCV |
| `marker_size` | `0.5` | >0 (м) | Размер ОДИНОЧНЫХ (вне карты) меток для per-marker PnP. Для меток карты размер берётся из `markers.txt`. | код (валидация >0) |
| `use_aruco3_detection` | `false` | bool | Быстрый мультимасштабный детектор (OpenCV ≥4.7; на Humble 4.5.4 игнорируется). С coarse обычно не нужен. | код; OpenCV ArUco3 |
| `aruco3_min_marker_length_ratio` | `0.10` | 0.0–1.0 | Мин. сторона метки как доля кадра для ArUco3. Меньше → ловит мельче, но медленнее. | код; OpenCV |
| `max_detect_fps` | `0.0` | ≤0=выкл; >0 Гц | Ранний троттл обработки кадров: ограничивает `cv_bridge`/`detectMarkers`/coarse-to-fine и частоту `/aruco/det/markers`, а значит и `/aruco/world_pose`. Фазовый аккумулятор держит среднюю цель: при камере ~30 Гц cap 20 даёт ~20 Гц, не ~15. Для CPU-лимита ставить 20–22 Гц; можно менять на лету. | код; runtime-тест |
| `estimate_marker_pose` | `false` | bool | Считать per-marker PnP и публиковать позы/TF одиночных меток. Нужен для посадочных меток. | код |
| `pnp_non_map_markers` | `false` | bool | Per-marker PnP для меток ВНЕ карты (вместе с `estimate_marker_pose` → TF посадочной площадки). | код; анализ (посадка) |
| `pnp_fps_cap_enabled` | `true` *(composed)* | bool | Ограничивать только per-marker PnP для НЕ-картовых меток. Не снижает стоимость `detectMarkers` и не троттлит `/aruco/det/markers`/одометрию. | код |
| `pnp_fps_cap` | `10` *(composed)* | >0 (Гц) | Макс. частота только per-marker PnP для НЕ-картовых меток. Для CPU/частоты одометрии использовать `max_detect_fps`. | код (валидация >0) |
| `marker_id_whitelist` | `[]` *(composed)* | массив int ≥0 | Если непуст — обрабатываются ТОЛЬКО эти ID (перекрывает всё). Осторожно: исключив ID карты, сломаешь локализацию. Можно менять на лету через `ros2 param set /aruco_detect marker_id_whitelist "[7]"`; callback применяет новое значение сразу. | код; runtime-тест |
| `opencv_num_threads` | `2` | -1,0,1..N | Потоки OpenCV в детекции. `2` = headroom под прочие процессы (RPi5). `0`=серийно, `-1`=авто (может занять A55 на A733). | код; платформа; анализ |

---

## 5. Coarse-to-fine (нода `aruco_detect`)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `coarse_to_fine_enabled` | `true` | bool | Детект на уменьшенном кадре + субпиксельный рефайн на полном. Главный рычаг CPU. | код; анализ |
| `coarse_scale` | `0.5` | (0, 1] | Множитель уменьшения: 0.5=640×480, 0.25=320×240. Меньше → меньше CPU, но короче дальность детекции. Правило выбора — в `TUNING_AND_VALIDATION.md` (под fx≈488). | код (валидация (0,1]); анализ |
| `corner_refine_pnp_only` | `true` | bool | Рефайнить углы только у меток карты (что идут в глобальный PnP). Экономит CPU. | код |
| `subpix_win_size` | `5` | ≥1 (реком. 3–10) | Полуокно `cornerSubPix`. Под scale 0.5 (×2) хватает 5; под 0.25 (×4) — 7–8. | код (валидация ≥1); OpenCV |

---

## 6. Локализация PnP: hybrid RANSAC / IPPE (нода `aruco_loc`)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `pnp_iterations` (`pnp.iterations`) | `100` | >0 (реком. 16–100) | Итерации RANSAC. Формула N=log(1−p)/log(1−wˢ): при w≈0.95 нужно ~3, 16 — запас. Сейчас 100 — обход под низкий inlier-rate (кривая карта); цель после плоского полигона — 16–32. | код; RANSAC-формула; анализ |
| `pnp_reprojection_error_px` (`pnp.reprojection_error_px`) | `12.0` | >0 (реком. 3–8) | Порог инлайера RANSAC, px. Сейчас 12 — обход (метка ID 3 на изогнутой подложке); цель — 4–6 (на коарс 0.5). Финальный gating строже. | код; анализ |
| `pnp_min_used_markers` (`pnp.min_used_markers`) | `0` | `<=0` auto; `>0` hard override | В auto нода сама выбирает метод: 1 метка → `SOLVEPNP_IPPE_SQUARE` без RANSAC, ≥2 метки → `solvePnPRansac`. Положительное значение возвращает жёсткий минимум меток. | код; анализ |
| `pnp_min_inlier_points` (`pnp.min_inlier_points`) | `0` | `<=0` auto; `>0` hard override | В auto для RANSAC требуется `max(6, ceil(0.5 * 4 * used_markers))` инлайеров; single-marker имеет 4 угла и фильтруется по RMSE. Положительное значение возвращает жёсткий минимум инлайеров. | код; анализ |
| `pnp_flags` (`pnp.flags`) | `0` (ITERATIVE) *(composed)* | 0\|8 (см. приложение) | Алгоритм PnP: `0`=ITERATIVE, `8`=SQPNP. SQPNP часто устойчивее на планарной карте — протестировать профайлером. | код; OpenCV |
| `pnp.confidence` | `0.99` *(нода)* | 0–1 | Желаемая вероятность RANSAC. Обычно не трогать. | код; OpenCV |
| `gating_max_inlier_rmse_px` (`gating.max_inlier_rmse_px`) | `6.0` *(composed)* | >0; ≤0=выкл (реком. 4–8) | Единый финальный фильтр качества для RANSAC и single-marker IPPE: отбраковать позу, если RMSE инлайеров/углов выше порога. | код; анализ |
| `refine_pose` | `false` *(loc)* | bool | Доп. `solvePnP` на инлайерах после RANSAC. Чуть точнее, дороже по CPU. | код; OpenCV |

 — параметры сейчас на «обходных» значениях под кривую подложку; вернуть к
реком. диапазону после плоского полигона.

---

## 7. Вертикаль / флип (нода `aruco_loc`)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `known_vertical_frame` | `base_link` *(composed)* | строка; ``=выкл | Брать roll/pitch из этого фрейма (IMU/EKF), yaw — из зрения. Подавляет флип на планарной карте. Требует валидный `world→base_link`. | код; анализ |
| `auto_flip` | `true` *(composed)* | bool | Автокоррекция переворота на 180°. Вместе с `known_vertical_frame`. При надирной камере возможен дребезг — тогда `false`. | код; анализ |
| `flip_vertical` | `false` *(нода)* | bool | Принудительный флип вертикали. Обычно не нужен. | код |

---

## 8. TF-offset / выход (нода `aruco_tf`)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `allow_offset_timeout_reset` | `false` | bool | Пересчитывать offset по таймауту (инкрементит `reset_counter`). `true` для авто-релокализации. | код |
| `offset_timeout` | `3.0` | >0 (с) | Секунды простоя до пересчёта offset (если включён reset). | код (валидация >0) |
| `initialize_at_origin` | `false` | bool | Сделать первую устойчивую ArUco-позу локальным нулём; при `false` совместить её с текущим `world→base_link`. SITL: `true`, real: `false`. | код; контракт VPE |
| `offset_init_samples` | `15` | integer >=1 | Число согласованных кандидатов до commit; используется последний. | код |
| `offset_candidate_translation_tolerance` | `0.10` | >=0 (m) | Максимальная Euclidean translation delta кандидата относительно первого кандидата последовательности. | код |
| `offset_candidate_rotation_tolerance` | `0.10` | >=0 (rad) | Максимальный shortest-path rotation delta относительно первого кандидата последовательности. | код |
| `covariance_xy_floor` | `0.0004` | >=0 (m²) | Нижняя граница variance X/Y; `sqrt=0.02 m`. Применяется как `max(input, floor)`. | код |
| `covariance_z_floor` | `0.0009` | >=0 (m²) | Нижняя граница variance Z; `sqrt=0.03 m`. | код |
| `covariance_angle_floor` | `0.0027415568` | >=0 (rad²) | Нижняя граница variance roll/pitch/yaw; standard deviation около 3°. | код |

Параметры объявлены в `aruco_composed.launch.py`. Real wrapper использует эти
defaults; SITL задаёт `initialize_at_origin=true`.

---

## 9. Режим роя (нода `aruco_tf`)

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `swarm_mode` | `false` *(composed)* | bool | `true`: публиковать base_link ПРЯМО в общем `aruco_map` (offset=identity, нуль общий для всех дронов → абсолютные сравнимые координаты). **Задавать в launch, не в полёте.** Подробно — `ARUCO_ODOMETRY_EXPLAINED.md`. | код; анализ (рой) |

---

## 10. Отладка

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `publish_debug_image` | `false` | bool | Изображение с нарисованными метками (`/aruco/vis/debug_image`). CPU. | код |
| `publish_world_debug_image` | `false` | bool | Изображение с осями карты (`/aruco/vis/world_debug_image`). CPU. | код |
| `publish_visualization` | `false` | bool | RViz MarkerArray (`/aruco/vis/markers`). CPU. | код |
| `enable_timing_log` | `false` *(нода)* | bool | JSON-тайминги в `/aruco/timing` для профайлера. Можно на лету. | код |

---

## 11. Производительность / CPU

| Параметр | Дефолт | Min/Max | Что это / как настраивать | Источник |
|---|---|---|---|---|
| `max_detect_fps` | `0.0` | ≤0=выкл; >0 Гц | Ранний лимит частоты детекции по `header.stamp` с fallback на steady clock. Пропущенные кадры не запускают OpenCV и не публикуют `/aruco/det/markers`. | код |
| `opencv_num_threads` | `2` | -1,0,1..N | (см. §4) RPi5: 2–3; A733: 2 (=число A76). | код; платформа |
| `cpu_params_file` | `` (пусто) | путь к YAML | Платформенный YAML (`config/cpu_rpi5.yaml`/`cpu_a733.yaml`) с affinity/потоками/coarse. Переопределяет per-key. Пусто = дефолты launch. | код; платформа |
| `cpu_affinity_ids` | `[]` *(нода/YAML)* | массив int ≥0 | Пиннинг потока к ядрам. RPi5: не нужно (однородные). A733: ID ядер A76 (`check_cpu_topology.sh`). Только через YAML (массив). | код; платформа |

---

## 12. Параметры нод, НЕ вынесенные в launch (через `-p`/YAML)

Эти параметры объявлены в нодах, но в launch их нет — задавай при необходимости
через `ros2 run … --ros-args -p name:=value` или params-file.

| Параметр | Нода | Дефолт | Назначение |
|---|---|---|---|
| `enabled` | detect/loc | `true` | Вкл/выкл обработку. |
| `use_map_markers` | detect | `true` | Подписываться на карту (размеры/фильтрация). |
| `detect_individual_markers` | detect/loc | `true` | Обрабатывать метки вне карты. |
| `send_tf` | detect | `true` | Публиковать TF одиночных меток. |
| `frame_id_prefix` | detect | `aruco_` | Префикс фреймов одиночных меток. Для посадки задать свой (напр. `target_`). |
| `size_override.ids` / `size_override.sizes` | detect | `[]` | Точные размеры конкретных ID (для посадочной метки). Можно задавать YAML/`-p` или менять в runtime; длины массивов должны совпадать. |
| `publish_tf` | loc | `true` (в composed `false`) | Динамический TF камера→aruco_map_detected. В composed выключен (коллизия). |
| `publish_static_markers_tf` | loc | `true` | Статические TF `aruco_map→aruco_<id>`. |
| `put_markers_count_to_covariance` | loc | `false` | Класть число меток вместо Якобиана (отладка). |
| `pose_cov_is_world_to_camera` | detect | `false` | Семантика входной позы для world-debug. |
| `world_pose_max_age_sec` | detect | `0.2` | Макс. возраст позы для отрисовки world-осей. |
| `world_axes_length` / `world_axes_thickness` | detect | `0.5` / `3` | Параметры отрисовки world-осей. |

---

## Приложение A. `dictionary_id` (enum cv::aruco::PREDEFINED_DICTIONARY_NAME)

```
0  DICT_4X4_50      4  DICT_5X5_50      8  DICT_6X6_50     12 DICT_7X7_50
1  DICT_4X4_100     5  DICT_5X5_100     9  DICT_6X6_100    13 DICT_7X7_100
2  DICT_4X4_250     6  DICT_5X5_250    10  DICT_6X6_250    14 DICT_7X7_250
3  DICT_4X4_1000    7  DICT_5X5_1000   11  DICT_6X6_1000   15 DICT_7X7_1000
16 DICT_ARUCO_ORIGINAL
```
Невалидный id → нода падает на fallback `DICT_4X4_50`. 4X4 — меньше модулей,
лучше для коарс/дальних; больший словарь (1000) — меньше Хэмминг-расстояние
(больше ложных). Для карты с ID ≤ ~100 надёжнее DICT_4X4_100 (=1).

## Приложение B. `pnp.flags` (cv::SolvePnPMethod)

```
0  SOLVEPNP_ITERATIVE   (дефолт; робастно, медленнее)
8  SOLVEPNP_SQPNP       (быстрее/устойчивее на планарных наборах — тестировать)
```
(Прочие значения: 1=EPNP, 2=P3P, 5=IPPE, 6=IPPE_SQUARE. `pnp.flags`
используется только в ветке RANSAC для ≥2 меток; одиночная метка всегда
решается автоматически через IPPE_SQUARE.)

## Приложение C. Источники / документы

- **код**: `aruco_det_loc/src/{aruco_detect_node,aruco_loc_node,aruco_tf_node}.cpp`,
  `aruco_map/src/aruco_map_node.cpp`, заголовок `include/aruco_det_loc/aruco_loc_utils.hpp`
  (там же валидация пределов в `onParams`).
- **OpenCV**: docs.opencv.org — `cv::aruco::DetectorParameters`, `cv::solvePnP`,
  `cv::solvePnPRansac`, `cv::cornerSubPix`.
- **RANSAC-формула**: N = log(1−p)/log(1−wˢ), s=4 (точки на минимальную выборку).
- **анализ/тюнинг**: `TUNING_AND_VALIDATION.md` (CPU, дальности под fx≈488, тесты),
  `ARUCO_ODOMETRY_EXPLAINED.md` (математика, дерево TF, режим роя).
- **калибровка**: `imx219_1280x960.yaml` (fx≈488, fisheye/plumb_bob) — определяет
  дальности детекции и точность PnP.
- **контракт**: камера 1280×960 RELIABLE; выход VPE `/aruco/world_pose` (ENU,
  best_effort) + `/aruco/reset_counter` (reliable).
- **платформа**: RPi5/CM5 (4×A76, affinity не нужна), A733 (2×A76+6×A55, пиннинг
  на A76 обязателен).
