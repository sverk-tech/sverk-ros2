# ArUco-одометрия — настройка параметров, расстановка по CPU и проверка корректности

Документ описывает, **как применять каждый параметр** стека `aruco_det_loc`
(+ `aruco_map`), **как правильно распределить нагрузку по ядрам CPU** на двух
целевых платформах и **как проверить, насколько корректно считается одометрия**
по ArUco. Все команды приведены под реальную камеру `camera_1` (1280×960, BGR8,
~30 Гц) и контракт VPE (`/aruco/world_pose`, `/aruco/reset_counter`).

> Сначала прочитайте раздел [0](#0-значения-по-умолчанию) — дефолты подобраны
> под реальную камеру, а не под Gazebo.

---

## Содержание

0. [Значения по умолчанию](#0-значения-по-умолчанию)
1. [Быстрый старт по платформам](#1-быстрый-старт-по-платформам)
2. [Применение параметров — по каждому](#2-применение-параметров--по-каждому)
3. [Расстановка по CPU (RPi 5 и A733)](#3-расстановка-по-cpu-rpi-5-и-a733)
4. [Калибровка камеры (обязательно перед полётом)](#4-калибровка-камеры-обязательно-перед-полётом)
5. [Проверка корректности одометрии — пошагово](#5-проверка-корректности-одометрии--пошагово)
6. [Чеклист перед полётом](#6-чеклист-перед-полётом)

---

## 0. Значения по умолчанию

Дефолты стека отличаются от upstream/Gazebo — подобраны под реальную камеру
`camera_1`. Ниже прежнее значение, текущий дефолт и обоснование.

| Что | Прежнее | По умолчанию | Зачем |
|---|---|---|---|
| Оптический фрейм по умолчанию | `main_camera_optical` | **`camera_optical_1`** | Совпадение с вашим static TF `base_link→camera_optical_1`. Иначе `aruco_tf_node` не находил TF и **молчал**. |
| Топики камеры по умолчанию | Gazebo-топики | **`/camera_1/image_raw`, `/camera_1/camera_info`** | Готово к реальной камере; для sim — переопределить. |
| QoS `camera_info` (detect+loc) | best_effort + volatile | **reliable + transient_local** | Иначе при latched-камере интринсики не приходят → PnP не стартует. |
| CPU-affinity в `aruco_composed.launch.py` | аргумент-«пустышка» | **через `cpu_params_file:=…`** | Старый `cpu_affinity_ids:=…` в composed **ничего не делал**. |
| `pnp.flags`, `gating.max_inlier_rmse_px` в composed | не пробрасывались | **проброшены** (`pnp_flags`, `gating_max_inlier_rmse_px`) | Возможность подобрать SQPNP и строгость отбраковки. |
| `known_vertical_frame` / `auto_flip` в composed | `'' / false` | **`base_link` / `true`** | Подавление флип-неоднозначности на планарной карте (z=0). |
| `corner_refine_pnp_only` | `false` | **`true`** | Рефайн углов только для map-маркеров → экономия CPU. |
| `max_detect_fps` | не было | **`0.0` (выкл), ставить 20–22 Гц при дефиците CPU** | Ранний троттл всей детекции: снижает `detectMarkers`, `/aruco/det/markers` и частоту `/aruco/world_pose`. |
| `pnp.min_used_markers` / `pnp.min_inlier_points` (дефолт кода и launch) | `3 / 8` | **`0 / 0` (auto)** | Нода сама выбирает метод: 1 метка → IPPE_SQUARE без RANSAC, ≥2 метки → RANSAC с auto-min-inliers. Положительные значения остаются legacy hard overrides. |
| Конфиги CPU | — | **`config/cpu_rpi5.yaml`, `config/cpu_a733.yaml`** | Единая платформенная точка для affinity/потоков/коарса. |

> Если вам нужно вернуть прежнее поведение (например, для Gazebo) — все значения
> переопределяются launch-аргументами, см. раздел 2.

---

## 1. Быстрый старт по платформам

> Предусловия: запущены `aruco_map_node` (публикует `/aruco/map/markers`), драйвер
> камеры (`/camera_1/image_raw` + `/camera_1/camera_info`), и существует static TF
> `base_link → camera_optical_1`, а также `map → base_link` (от EKF/одометрии).

### Raspberry Pi 5 / CM5

```bash
ros2 launch aruco_det_loc aruco_composed.launch.py \
  cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_rpi5.yaml
```

### Orange Pi Zero 3W / Allwinner A733

```bash
# 1) ОДИН РАЗ: определить ID ядер A76 и при необходимости поправить cpu_a733.yaml
bash $(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/scripts/check_cpu_topology.sh

# 2) Запуск с привязкой к A76 + coarse-to-fine
ros2 launch aruco_det_loc aruco_composed.launch.py \
  cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_a733.yaml
```

### Карта (отдельный процесс — всегда)

```bash
ros2 launch aruco_map aruco_map.launch.py            # рабочий режим
ros2 launch aruco_map aruco_map.launch.py mode:=debug # + debug-изображения плоскостей
```

---

## 2. Применение параметров — по каждому

Три способа задать параметр (в порядке предпочтения для разных типов):

| Тип параметра | Как задавать | Пример |
|---|---|---|
| Скаляр (bool/int/float/str) | launch-аргумент **или** `ros2 param set` в runtime | `... coarse_scale:=0.5` |
| Runtime-массив (`marker_id_whitelist`, `size_override.ids/sizes`) | YAML/CLI **или** `ros2 param set` в runtime | `ros2 param set /aruco_detect marker_id_whitelist "[7]"` |
| Startup-only (`cpu_affinity_ids`, `opencv_num_threads`) | задавать **до** старта ноды (YAML/CLI), `ros2 param set` не применит на лету | `cpu_params_file:=…` |

Проверить текущее значение: `ros2 param get /aruco_detect <name>`.
Сменить на лету (если параметр это позволяет): `ros2 param set /aruco_detect <name> <value>`.

### 2.1. `aruco_detect_node` — детекция

| Параметр | По умолч. | Что делает | Когда менять |
|---|---|---|---|
| `dictionary_id` | `3` (DICT_4X4_1000) | Словарь ArUco. Должен совпадать со словарём **печатной** карты (дебаг-рендер `aruco_map` тоже DICT_4X4_1000). | Менять только вместе с перепечаткой карты. При ID<100 более надёжен `1` (DICT_4X4_100). |
| `marker_size` | `0.5` | Размер только для маркеров **вне** карты. Для карты берётся из `markers.txt`. | Не трогать, если работаете только по карте. |
| `coarse_to_fine_enabled` | `false` (RPi5) / `true` (A733, через yaml) | Детект на уменьшенном кадре + cornerSubPix на полном. Главная экономия CPU. | A733 — вкл. RPi5 — вкл. при дефиците CPU. |
| `coarse_scale` | `0.25` | Множитель уменьшения (0.25→320×240, 0.5→640×480). | По правилу дальности (раздел 3.4). |
| `corner_refine_pnp_only` | `true` | Рефайнить углы только для map-маркеров. | Оставить `true` (non-map PnP выключен). |
| `subpix_win_size` | `6` (yaml: 8 для 0.25, 5 для 0.5) | Полуокно `cornerSubPix`. Должно покрывать ошибку коарс-локализации. | 0.25→7–8, 0.5→5. |
| `max_detect_fps` | `0.0` | Ранний лимит частоты обработки кадров и публикации `/aruco/det/markers`. `0`/`<=0` = без ограничения. | Для снижения CPU и частоты одометрии ставить 20–22 Гц; можно менять на лету: `ros2 param set /aruco_detect max_detect_fps 20.0`. |
| `marker_id_whitelist` | `[]` | Если непуст — detect публикует только эти ID; удобно для воспроизводимого single-marker IPPE-теста. Runtime callback применяет новое значение сразу. | Для одной map-метки: `ros2 param set /aruco_detect marker_id_whitelist "[7]"`; вернуть все: `ros2 param set /aruco_detect marker_id_whitelist "[]"`. |
| `use_aruco3_detection` | `false` | Быстрый многомасштабный детектор (OpenCV ≥4.7). На Humble/4.5.4 игнорируется (есть warn). | Обычно `false` при coarse (двойная фильтрация мелких). Проверять профайлером. |
| `estimate_marker_pose` | `false` | Per-marker PnP в detect. | `false` — позу карты считает loc. |
| `pnp_non_map_markers` | `false` | PnP для маркеров вне карты (нужен их TF). | `true` только если нужны отдельные маркеры. |
| `pnp_fps_cap` / `pnp_fps_cap_enabled` | `10` / `true` | Ограничивает только per-marker PnP для НЕ-картовых меток. | Не использовать как CPU-троттл детекции: `detectMarkers`, `/aruco/det/markers` и map-одометрия им не ограничиваются. |
| `opencv_num_threads` | `2` (yaml: 3 для RPi5) | Размер пула OpenCV. | RPi5: 3; A733: 2 (= число A76). |
| `cpu_affinity_ids` | `[]` (yaml) | Привязка потока к ядрам. | A733: A76-ядра; RPi5: пусто. |
| `publish_debug_image` / `publish_visualization` / `publish_world_debug_image` | `false` | Отладочные картинки/маркеры RViz. | Вкл. для отладки (см. раздел 5). Можно `ros2 service call /aruco_detect/toggle_debug …`. |
| `enable_timing_log` | `false` | JSON-тайминги в `/aruco/timing` для профайлера. | Вкл. для замеров (можно на лету). |

### 2.2. `aruco_loc_node` — глобальная поза камеры

| Параметр | По умолч. | Что делает | Когда менять |
|---|---|---|---|
| `pnp.iterations` | `16` | Итерации RANSAC. При w≈0.95 нужно ~3, 16 — запас. | Поднять до 32–100 при w<0.7 (много ложных детекций). |
| `pnp.reprojection_error_px` | `4.0` | Порог инлайера RANSAC (px). | full-res → 3; coarse 0.25 → 4–5. |
| `pnp.flags` | `0` (ITERATIVE) | Алгоритм PnP. `8`=SQPNP часто устойчивее на планарной карте. | Подобрать профайлером (`sqpnp_solver`). |
| `pnp.min_used_markers` | `0` | Auto: 1 метка → IPPE_SQUARE, ≥2 метки → RANSAC. | Оставить `0`; ставить `>0` только если нужен старый жёсткий минимум. |
| `pnp.min_inlier_points` | `0` | Auto: для RANSAC минимум `max(6, ceil(0.5 * 4 * used_markers))`; single-marker даёт 4 угла. | Оставить `0`; ставить `>0` только для legacy hard gate. |
| `gating.max_inlier_rmse_px` | `6.0` | Единый финальный RMSE-гейт для RANSAC и single-marker IPPE. | Снизить (4–5) для строгости; поднять, если хорошие позы режутся. |
| `known_vertical_frame` | `base_link` | Брать roll/pitch из этого фрейма (IMU/EKF), yaw — из vision. | Подавляет флип. **Отключить** (`''`), если нет надёжного `map→base_link`. |
| `auto_flip` | `true` | Автокоррекция переворота на 180°. | Вместе с `known_vertical_frame`. |
| `refine_pose` | `false` | Доп. `solvePnP` на инлайерах. | Вкл. при необходимости чуть более точной позы (стоит CPU). |
| `put_markers_count_to_covariance` | `false` | Класть в ковариацию число маркеров вместо Якобиана. | Только для отладки/экономии; для EKF2 оставить `false`. |

### 2.3. `aruco_tf_node` — offset и поза base_link в world (ENU)

| Параметр | По умолч. | Что делает | Когда менять |
|---|---|---|---|
| `camera_frame_id` | `camera_optical_1` | Камера в TF (для lookup `camera→base_link`). | Должно совпадать с `parent_frame_id` детекта и static TF. |
| `base_link_frame_id` | `base_link` | Тело дрона. | По вашему TF-дереву. |
| `world_frame_id` | `map` | Мировой/ENU фрейм. | Должен быть **ENU** (контракт VPE). |
| `aruco_map_frame_id` | `aruco_map` | Child статического TF offset. | Обычно не трогать. |
| `allow_offset_timeout_reset` | `false` | Пересчитывать offset по таймауту. | `true`, если хотите авто-релокализацию (инкрементит `reset_counter`). |
| `offset_timeout` | `3.0` | Секунды до пересчёта offset. | Вместе с `allow_offset_timeout_reset`. |
| `initialize_at_origin` | `false` | Использовать первую квалифицированную ArUco-позу как local origin. | `true` для SITL/per-drone takeoff origin; `false` для привязки к уже надёжному PX4 world frame. |
| `offset_init_samples` | `15` | Требуемая длина стабильной последовательности. | Уменьшать только при проверенной чистой детекции; увеличение задерживает готовность. |
| `offset_candidate_translation_tolerance` | `0.10 m` | Допустимое расхождение offset translation. | Снижать для строгой инициализации, повышать только после анализа шума. |
| `offset_candidate_rotation_tolerance` | `0.10 rad` | Допустимое расхождение offset rotation. | `0.10 rad` ≈ 5.73°. |
| `covariance_xy_floor` | `0.0004 m²` | Минимальная X/Y variance. | 2 cm standard deviation; не ставить 0 для EKF2. |
| `covariance_z_floor` | `0.0009 m²` | Минимальная Z variance. | 3 cm standard deviation. |
| `covariance_angle_floor` | `0.0027415568 rad²` | Минимальная angular variance. | Около 3° standard deviation. |

> **Контракт выхода** (для PX4-VPE): `/aruco/world_pose`
> (`PoseWithCovarianceStamped`, поза **base_link в ENU**, best_effort) и
> `/aruco/reset_counter` (`UInt8`, reliable, KeepLast 1). Не переименовывайте без
> синхронного изменения VPE-моста.

### 2.4. `aruco_map_node` — карта маркеров

| Параметр | По умолч. | Что делает |
|---|---|---|
| `map_file` | `…/aruco_map/config/markers.txt` | Файл карты `id size x y z rot_z rot_y rot_x`. |
| `frame_id` | `map` | Фрейм карты (= world для loc). |
| `debug_aruco_dictionary` | `DICT_4X4_1000` | Словарь для **отрисовки** дебаг-плоскостей. Должен совпадать с `dictionary_id` детектора. |

> Карта в `markers.txt` **компланарна** (все z≈0). Это нормально, но именно из-за
> этого важны `known_vertical_frame`/`auto_flip`; в auto-режиме одиночная метка
> допускается через IPPE_SQUARE, но без known vertical она менее надёжна.

---

## 3. Расстановка по CPU (RPi 5 и A733)

### 3.1. Бюджет на кадр (33 мс при 30 fps)

Цепочка на кадр: `cvtColor(1280×960)` + `detectMarkers` (на коарс или полном) +
`cornerSubPix` + глобальный hybrid PnP (RANSAC или single-marker IPPE). Ориентиры:

| Этап | full-res 1280×960 | coarse 0.5 (640×480) | coarse 0.25 (320×240) |
|---|---|---|---|
| `detect_ms` на одном A76 | ~10–25 мс | ~3–7 мс | ~1–3 мс |
| `pnp_ms` (loc, 3–10 маркеров) | ~1–4 мс | ~1–4 мс | ~1–4 мс |

### 3.2. Raspberry Pi 5 (4× A76, однородные)

- **Affinity не нужна** — все ядра равны. `cpu_affinity_ids: []`.
- `opencv_num_threads: 3` (оставляем 1 ядро на DDS/прочие ноды).
- Coarse — по желанию; full-res укладывается в бюджет.
- Конфиг: `config/cpu_rpi5.yaml`.

### 3.3. Allwinner A733 (2× A76 + 6× A55, big.LITTLE)

Affinity **критична**: если CV уедет на A55, детект не уложится в 33 мс.

```bash
# 1) Определить ID ядер A76 (PART_ID = 0xD0B):
bash $(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/scripts/check_cpu_topology.sh
# Скрипт печатает готовую строку, например:  cpu_affinity_ids:=[0,1]
```

- Впишите реальные ID в `config/cpu_a733.yaml` → `cpu_affinity_ids: [..]`
  (по умолчанию `[0,1]`, но **проверьте** — нумерация бывает иной).
- `opencv_num_threads: 2` (= число A76).
- `coarse_to_fine_enabled: true`, `coarse_scale: 0.25`, `subpix_win_size: 8`.

**Проверка, что привязка реально работает:**

```bash
PID=$(pgrep -f aruco_container)
# Маска родительского потока:
taskset -cp $PID
# Маски всех потоков процесса (включая пул OpenCV — должны быть только A76-ядра):
for t in /proc/$PID/task/*; do echo -n "$(basename $t): "; taskset -cp $(basename $t) 2>/dev/null; done
# Под нагрузкой убедиться, что заняты именно A76:
htop   # или: top -H -p $PID
```

### 3.4. Правило выбора `coarse_scale` (по физике камеры)

fx ≈ **1060 px** @1280×960 (HFoV≈62° для IMX219 — *допущение*; уточните после
калибровки). Маркер теряется, когда сторона на **коарс**-кадре < ~16–20 px:
`D_max ≈ coarse_scale · fx · S / p_min`.

| Размер маркера | full (1.0) | 0.5 | 0.25 |
|---|---|---|---|
| 0.40 м | ~21–27 м | ~11–13 м | ~5.3–6.6 м |
| 0.15 м | ~8–10 м | ~4–5 м | ~2.0–2.5 м |
| 0.11 м | ~6–7 м | ~3–3.6 м | ~1.5–1.8 м |
| 0.06 м | ~3–4 м | ~1.6–2 м | ~0.8–1 м |

(диапазон = порог 20→16 px; это **видимость**, точность падает раньше).

- `0.25` — низкая высота / крупные маркеры (0.15–0.4 м) в кадре. Максимум экономии CPU.
- `0.5` — если на 0.25 теряются нужные мелкие (0.06–0.11 м) или растёт высота.
- full (выкл. coarse) — RPi5 при запасе CPU или ловля 0.06 м на 3–4 м.

---

## 4. Калибровка камеры (обязательно перед полётом)

`imx219_1280x960.yaml` — **placeholder** (fx≈fy≈1000, cx=640, cy=480, нулевые
дисторсии). Стек копирует `K`/`D` из `camera_info` как есть → placeholder исказит:

- **Дисторсия `D=0`** — реальный IMX219 имеет бочкообразную дисторсию; углы у
  краёв кадра «уедут» на несколько px → систематический наклон позы, усиление флипа.
- **`fx` неточный** — линейная ошибка по **высоте/дальности** (~6% при истинном fx≈1060).
- **`cx,cy=центр`** — небольшой постоянный наклон.

**Что сделать:** прогнать `camera_calibration` именно на 1280×960:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera_1/image_raw -r camera:=/camera_1
# Сохранить → положить реальные K и D в camera_info вашего драйвера.
```

Проверка, что пришла НЕ placeholder-калибровка:

```bash
ros2 topic echo --once /camera_1/camera_info
# d: должно быть НЕ [0,0,0,0,0]; k[0] (fx) — реальный, не ровно 1000.
```

---

## 5. Проверка корректности одометрии — пошагово

Идти строго по порядку: A→H. Каждый шаг имеет **критерий прохождения**.

### A. Связность и QoS (топики, частоты, TF)

```bash
ros2 topic list | grep -E "markers|pose_cov|world_pose|reset_counter|/aruco/map/markers"
ros2 topic hz /camera_1/image_raw      # ~30 Гц
ros2 topic hz /aruco/det/markers                 # ~30 Гц без max_detect_fps; ~20 Гц при max_detect_fps:=20
ros2 topic hz /aruco/loc/pose_cov      # ~частота кадров с видимыми map-маркерами; 1 метка работает при known_vertical
ros2 topic hz /aruco/world_pose        # должна совпадать с pose_cov

# CPU/частота одометрии: ранний троттл детекции, можно менять на лету.
ros2 param set /aruco_detect max_detect_fps 20.0
ros2 topic hz /aruco/det/markers                 # ≈20 Гц
ros2 topic hz /aruco/world_pose        # ≈20 Гц, если loc принимает кадры
ros2 param set /aruco_detect max_detect_fps 0.0   # выключить лимит

# Совместимость QoS (не должно быть "incompatible"):
ros2 topic info -v /camera_1/camera_info
ros2 topic info -v /aruco/map/markers

# TF-дерево: должны быть base_link→camera_optical_1, map→base_link,
# map→aruco_map (static), map→base_link_aruco (динамический):
ros2 run tf2_tools view_frames
```

**Критерий:** без `max_detect_fps` `/aruco/world_pose` публикуется на ~частоте кадров;
при `max_detect_fps:=20` `/aruco/det/markers` и `/aruco/world_pose` падают примерно до 20 Гц;
нет QoS-incompatible; в TF присутствует `map → base_link_aruco`.

> Если `/aruco/world_pose` **молчит**: 99% — не совпали имена фреймов. Проверьте
> `ros2 run tf2_ros tf2_echo camera_optical_1 base_link` (должен находиться) и
> логи `aruco_tf`: `TF camera_optical_1->base_link: ...`.

### B. Статическая точность (дрон неподвижен над картой)

Зафиксируйте дрон неподвижно над картой, запишите позу:

```bash
ros2 topic echo /aruco/world_pose --field pose.pose.position > /tmp/static_pos.txt
# собрать ~30 с, затем оценить разброс:
python3 - <<'PY'
import re, statistics as st
xs=[];ys=[];zs=[]
for l in open('/tmp/static_pos.txt'):
    m=re.match(r'\s*(x|y|z):\s*([-\d.eE]+)',l)
    if m: {'x':xs,'y':ys,'z':zs}[m.group(1)].append(float(m.group(2)))
for n,v in (('x',xs),('y',ys),('z',zs)):
    if v: print(f"{n}: mean={st.mean(v):.4f}  std={st.pstdev(v):.4f}  ptp={max(v)-min(v):.4f}")
PY
```

**Критерий (низкая высота, ≥3 маркера):** std по x,y ≲ 1–2 см, по z ≲ 2–4 см,
без скачков. Большой разброс/прыжки → шум углов (поднять `coarse_scale`/выключить
coarse), мало маркеров, либо placeholder-калибровка.

### C. Известное смещение (рулетка)

Сдвиньте дрон на **измеренное** расстояние (например, +0.50 м по X карты):

```bash
ros2 topic echo /aruco/world_pose --field pose.pose.position
# Δx published ≈ 0.50 м (± несколько %). Проверить знак и ось.
```

**Критерий:** измеренное смещение совпадает с реальным по модулю (ошибка масштаба
≲ 5–10% до калибровки, ≲ 2–3% после) и по **направлению/знаку** осей (ENU).

### D. Reprojection RMSE и число инлайеров (внутренняя метрика качества)

```bash
ros2 param set /aruco_loc enable_timing_log true
ros2 topic echo /aruco/timing | grep -oE '"(pnp_method|required_inliers|reproj_rmse|n_inliers|n_markers|gating_passed)":[^,}]*'
```

**Критерий:** `reproj_rmse` стабильно **< 2 px** (после калибровки), `n_inliers`
≈ 4×`n_markers` (почти все углы — инлайеры), `gating_passed:true`. Для ≥2 меток
`pnp_method:"ransac"`; для одной map-метки `pnp_method:"single_ippe_square"` и
`required_inliers:4`. Высокий RMSE или частый `gating_passed:false` → плохие углы /
неверная калибровка / неверный размер маркеров в `markers.txt`.

Для воспроизводимой проверки single-marker без физического закрытия карты можно
временно ограничить detect одной map-меткой:

```bash
ros2 param set /aruco_detect marker_id_whitelist "[7]"   # заменить 7 на видимый ID карты
ros2 topic hz /aruco/world_pose                          # поза должна продолжать идти
ros2 topic echo /aruco/timing | grep -oE '"(pnp_method|required_inliers|n_inliers|reproj_rmse)":[^,}]*'
ros2 param set /aruco_detect marker_id_whitelist "[]"    # восстановить все ID
```

### E. Консистентность ориентации (нет флипа)

```bash
# Включить отладочное наложение мировых осей на изображение:
ros2 service call /aruco_detect/toggle_debug std_srvs/srv/SetBool "{data: true}"
ros2 run rqt_image_view rqt_image_view   # топик /aruco/vis/world_debug_image
```

**Критерий:** оси `aruco_map` нарисованы на карте корректно и **не переворачиваются**
при наклонах камеры. Если позу «перекидывает» на 180° → проверьте
`known_vertical_frame:=base_link auto_flip:=true` и наличие валидного `map→base_link`.
Также сверьте `/aruco/world_pose` (orientation) с TF:

```bash
ros2 run tf2_ros tf2_echo map base_link_aruco   # должно совпадать с world_pose
```

### F. Сравнение с независимым ground truth

Сопоставьте ArUco-позу с другим источником (рулетка по сетке карты, мокап,
или GPS/RTK на открытом — для помещения обычно рулетка/разметка):

```bash
ros2 bag record /aruco/world_pose /tf -o aruco_gt_check -d 60
# Прогнать дрон по известной траектории (углы карты, метки на полу),
# сравнить published position с замеренными точками.
```

**Критерий:** систематических смещений нет; ошибка позиции в пределах ожидаемой
из RMSE и калибровки.

### G. Латентность и нагрузка (профайлер)

```bash
ros2 param set /aruco_detect enable_timing_log true
ros2 param set /aruco_loc    enable_timing_log true
python3 $(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/scripts/profiling/aruco_profiler.py \
  --record-only --duration 30 --label baseline --output-dir ./profile_results
```

**Критерий:** `detect_ms` p95 < 33 мс (иначе < 30 fps), `image_to_pose_ms` mean
< 30–50 мс. На A733: если `detect_ms` велик/нестабилен — проверьте affinity (3.3).

Подбор `pnp.flags`/итераций/коарса автоматически:

```bash
python3 .../aruco_profiler.py --full-test --duration-per-config 20 --output-dir ./profile_results
# Сравните baseline vs sqpnp_solver vs ransac_8iter в отчёте report_*.md.
```

### H. Поведение `reset_counter`

```bash
ros2 topic echo /aruco/reset_counter
```

**Критерий:** счётчик инкрементируется **только** при (пере)вычислении offset
(первый захват карты; и при таймауте, если `allow_offset_timeout_reset:=true`),
а не на каждом кадре. Проверка пересчёта:

```bash
ros2 param set /aruco_tf allow_offset_timeout_reset true
ros2 param set /aruco_tf offset_timeout 0.1     # спровоцировать пересчёт
# В логах: "Offset (re)computed ... reset_counter=N", счётчик +1
ros2 param set /aruco_tf allow_offset_timeout_reset false
```

---

## 6. Чеклист перед полётом

- [ ] **Калибровка камеры реальная**, не placeholder (раздел 4: `d` ≠ нули).
- [ ] Static TF `base_link → camera_optical_1` существует и совпадает по имени.
- [ ] `map → base_link` (от EKF) присутствует и в **ENU** (для `known_vertical`/offset).
- [ ] Словарь печатной карты = `dictionary_id` (3 = DICT_4X4_1000).
- [ ] Размеры маркеров в `markers.txt` соответствуют **физическим** (метры).
- [ ] A733: `cpu_affinity_ids` = реальные A76-ядра (раздел 3.3), привязка подтверждена `taskset`.
- [ ] Тест B: std позиции в покое в норме; тест C: масштаб/знаки осей верны.
- [ ] Тест D: `reproj_rmse < 2 px`, `gating_passed:true`.
- [ ] Тест E: ориентация не флипается.
- [ ] Тест G: `detect_ms` p95 < 33 мс на целевой плате.
- [ ] VPE-мост запущен с `allow_send_vio:=false` до подтверждения позы, затем `true`.
