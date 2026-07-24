# ArUco-одометрия: от кадра камеры до `/aruco/world_pose`

Документ объясняет, что и как считается в стеке `aruco_det_loc`: какие фреймы
участвуют, какие преобразования их связывают, как из позы камеры в поле меток
получается поза дрона в мире, и как устроено дерево TF. Все формулы — в
моноширинных блоках (надёжно отображаются).

---

## 1. Фреймы (системы координат)

| Фрейм | Что это |
|---|---|
| `map` | Мировой фрейм (ENU). В нём живёт EKF/одометрия дрона. = «world». |
| `base_link` | Тело дрона. |
| `camera_optical_1` | Оптический фрейм камеры (ось Z — вперёд по взгляду). |
| `aruco_map` | Начало координат поля ArUco-меток (origin из `markers.txt`). |
| `aruco_<id>` | Отдельная метка карты (поза задана в `markers.txt`). |
| `aruco_map_detected` | Поза камеры в мире (диагностический фрейм). |
| `base_link_aruco` | Поза дрона в мире по ArUco — то, что уходит в `/aruco/world_pose`. |

---

## 2. Обозначение преобразований

```
T_A_B  — преобразование, выражающее фрейм B в системе A:
             p_A = T_A_B * p_B
         (по-другому: «поза B относительно A»)

Свойства:
   (T_A_B)^-1 = T_B_A          обратное меняет местами фреймы
   T_A_B * T_B_C = T_A_C       цепочка «схлопывается» по общему среднему фрейму
```

`*` здесь — композиция преобразований (сначала применяется правое), а не
обычное умножение чисел.

---

## 3. Конвейер целиком (поток данных)

```
камера /camera_1/image_raw  (1280x960, BGR8, ~30 Гц)
        │
        ▼
aruco_detect_node ── находит метки, отдаёт ПИКСЕЛЬНЫЕ углы каждой метки
        │   topic: /aruco/det/markers   (id + 4 угла в пикселях)
        ▼
aruco_loc_node ── сопоставляет углы с 3D-позициями меток из карты,
        │         решает глобальный PnP: RANSAC (≥2 метки) или IPPE_SQUARE (1 метка)
        │         → поза камеры в поле меток
        │   topic: /aruco/loc/pose_cov   =  T_aruco_camera
        ▼
aruco_tf_node ── применяет offset (где поле в мире) и геометрию камеры
        │   topic: /aruco/world_pose     =  T_world_base  (поза дрона в ENU)
        │   topic: /aruco/reset_counter  (UInt8, для PX4 EKF2)
        ▼
PX4-VPE мост → EKF2
```

Карта меток приходит отдельно: `aruco_map_node` публикует `/aruco/map/markers`
(позы и размеры всех меток в фрейме `aruco_map`), его читают и detect, и loc.

---

## 4. `/aruco/loc/pose_cov` — поза камеры в поле меток

Это вход для `aruco_tf_node` и то, что ты уже понимаешь: **поза камеры в системе
координат поля меток**, то есть `T_aruco_camera`.

Как loc-нода её получает:

1. Для каждой видимой метки карты берёт её 4 угла в мире (3D, фрейм `aruco_map`)
   и те же 4 угла в пикселях (от detect). Получается набор 3D↔2D соответствий.
2. Решает PnP гибридно:
   - если используется ≥2 метки, запускает `solvePnPRansac`;
   - если используется ровно 1 метка (4 угла), запускает `SOLVEPNP_IPPE_SQUARE`
     без RANSAC, выбирает кандидат с меньшей reprojection RMSE.
   В обоих случаях OpenCV возвращает `rvec, tvec` такие, что:

```
   p_camera = R(rvec) * p_aruco + tvec        ⇒   [R | tvec] = T_camera_aruco
```

   То есть «сырой» результат PnP — это `T_camera_aruco` (мир→камера).

   Для одиночной метки это менее уверенное измерение: планарный квадрат имеет
   flip-неоднозначность. Поэтому в рабочем режиме держим включёнными
   `known_vertical_frame` и `auto_flip`; если `known_vertical_frame` пустой,
   нода предупреждает в логах.

3. Поза камеры в поле = обратное:

```
   T_aruco_camera = (T_camera_aruco)^-1        ← публикуется в /aruco/loc/pose_cov
```

> Внимание к именам: «world» внутри loc-ноды — это и есть фрейм `aruco_map`
> (origin карты). Loc не знает про настоящий мир дрона. Связь с настоящим миром
> добавляет уже `aruco_tf_node` через offset.

---

## 5. `/aruco/world_pose` — поза дрона в мире (ENU)

Считается в `aruco_tf_node`. Результат:

```
   /aruco/world_pose  =  T_world_base   (поза base_link в фрейме map/ENU)
```

### 5.1. Зачем нужен offset

`aruco_map` и `map` — два разных НЕПОДВИЖНЫХ фрейма. Поле меток лежит где-то в
мире под каким-то углом. Их связывает постоянное преобразование:

```
   offset  ≡  T_world_aruco     («где поле меток в мире»)
```

Камера через зрение знает только позу относительно поля (`T_aruco_camera`).
Чтобы получить позу в мире, надо знать, где само поле в мире — это offset. Он
принимается после устойчивой последовательности измерений и потом
замораживается. По умолчанию обычная потеря/возврат маркеров offset не меняет.

### 5.2. Вычисление offset

В момент commit квалифицированного offset известны три вещи:

```
   T_world_base     — поза дрона из TF/EKF      (lookupTransform map → base_link)
   T_camera_base    — статический TF камера↔тело (lookupTransform camera → base_link)
   T_aruco_camera   — текущее зрительное измерение (вход pose_cov)
```

offset по цепному правилу:

```
   T_base_camera = (T_camera_base)^-1
   T_camera_aruco = (T_aruco_camera)^-1

   offset = T_world_aruco = T_world_base * T_base_camera * T_camera_aruco
```

Почему верно — раскрываем цепочку (общие фреймы сокращаются):

```
   T_world_base * T_base_camera          = T_world_camera
   T_world_camera * T_camera_aruco       = T_world_aruco
```

При `initialize_at_origin=false` первая ArUco-поза совмещается с позой EKF.
При `initialize_at_origin=true` offset вычисляется так:

```
   T_aruco_base = T_aruco_camera * T_camera_base
   offset = (T_aruco_base)^-1
```

Первая квалифицированная поза корпуса становится local zero; marker ID не важен.

Commit происходит после 15 кандидатов в пределах `0.10 m` и `0.10 rad` от
первого кандидата последовательности.

### 5.3. На каждом кадре

Имея замороженный offset и свежий `T_aruco_camera`:

```
   T_world_camera = offset * T_aruco_camera          (поза камеры в мире)
   T_world_base   = T_world_camera * T_camera_base   (поза дрона в мире)  ← выход
```

### 5.4. Одной строкой

```
   T_world_base =  ( T_world_base(0) * T_camera_base(0)^-1 * T_aruco_camera(0)^-1 )
                   ───────────────────────────────────────────────────────────────
                                  offset, заморожен при initialization commit
                   * T_aruco_camera(t)        ← свежее измерение
                   * T_camera_base            ← статическая геометрия камеры
```

`(0)` — величины при commit квалифицированного offset, `(t)` — текущий кадр.

### 5.5. Что ещё в callback

- **TF-публикации**: `map → aruco_map_detected` = `T_world_camera` (камера),
  `map → base_link_aruco` = `T_world_base` (дрон) — для RViz/диагностики.
- **Ковариация**: копируется из `pose_cov`, после чего диагональные variance
  ограничиваются снизу: XY `0.0004 m²`, Z `0.0009 m²`, углы
  `0.0027415568 rad²`. Численно исходная covariance остаётся аппроксимацией PnP.
- **reset_counter**: инкрементится ТОЛЬКО при (пере)вычислении offset — сигнал
  PX4 EKF2 «внешний источник скачком переинициализировался».

---

## 6. Дерево TF (схема)

```
map  (world, ENU)
│
├── base_link                         [EXT, dynamic ~100 Гц]   EKF/одометрия
│     └── camera_optical_1            [EXT, static]            base_link → camera
│
├── aruco_map                         [aruco_tf, static]       offset = T_world_aruco
│     ├── aruco_0                     [aruco_loc, static]
│     ├── aruco_1                     [aruco_loc, static]
│     └── aruco_<id> ...              [aruco_loc, static]      по карте markers.txt
│
├── aruco_map_detected                [aruco_tf, dynamic]      = T_world_camera
│        ▲
│        ╎ КОЛЛИЗИЯ: второй родитель (см. раздел 7)
│        ╎
│   camera_optical_1 ─ ─ ─ ─ ─ ─ ─ ─ ┘  [aruco_loc, dynamic]  = T_camera_aruco
│
└── base_link_aruco                   [aruco_tf, dynamic]      = T_world_base
                                                               → /aruco/world_pose
```

Кто публикует каждое ребро:

| Ребро | Тип | Источник | Преобразование |
|---|---|---|---|
| `map → base_link` | dynamic | внешний (EKF, ~100 Гц) | `T_world_base` (читает tf-нода) |
| `base_link → camera_optical_1` | static | внешний static TF | `T_base_camera` (читает tf-нода) |
| `map → aruco_map` | static | aruco_tf (offset) | `T_world_aruco` |
| `aruco_map → aruco_<id>` ×N | static | aruco_loc | позы меток карты |
| `map → aruco_map_detected` | dynamic | aruco_tf | `T_world_camera` |
| `map → base_link_aruco` | dynamic | aruco_tf | `T_world_base` → выход |
| `camera_optical_1 → aruco_map_detected` | dynamic | aruco_loc | `T_camera_aruco` |

Связь схемы с формулами:

```
   (map→aruco_map)            (вход pose_cov)         (map→aruco_map_detected)
    T_world_aruco       *      T_aruco_camera     =      T_world_camera

    T_world_camera      *      T_camera_base      =      T_world_base
                                                       (map→base_link_aruco → /aruco/world_pose)
```

---

## 7. Коллизия `aruco_map_detected` (баг + фикс)

Фрейм `aruco_map_detected` получает ДВА родителя:
- `map` — от `aruco_tf` (= поза камеры в мире);
- `camera_optical_1` — от `aruco_loc` (= поза поля относительно камеры).

В tf2 у фрейма может быть только один родитель. Два broadcaster'а на один и тот
же child → буфер TF постоянно перезаписывается, фрейм «дрожит», `tf2_echo`/RViz
дают неверные значения. На сам топик `/aruco/world_pose` это НЕ влияет (топик не
зависит от TF-дерева), но загрязняет дерево.

Причина: `aruco_loc` публиковал этот TF ещё до рефакторинга (когда offset делал
VPE). После появления `aruco_tf` он стал лишним.

**Фикс (применён в `aruco_composed.launch.py`):** у loc выставлено
`publish_tf:=false` — теперь `aruco_tf` единственный владелец world-фреймов.
Статические `aruco_<id>` у loc остаются (это отдельный флаг
`publish_static_markers_tf`).

> Вторая потенциальная (по умолчанию неактивная) коллизия: detect-нода может
> публиковать `camera_optical_1 → aruco_<id>` для меток ВНЕ карты. Включается
> только при `estimate_marker_pose + pnp_non_map_markers + send_tf`. Если
> включишь per-marker PnP — задай detect другой `frame_id_prefix`.

---

## 8. Что лежит в сообщении `/aruco/world_pose`

Тип: `geometry_msgs/PoseWithCovarianceStamped`, QoS `best_effort`.

```
header.stamp        — копия времени кадра (= stamp входного pose_cov)
header.frame_id     — world_frame_id ("map")
pose.pose.position  — трансляция T_world_base (позиция дрона в ENU)
pose.pose.orientation — кватернион вращения T_world_base
pose.covariance     — из /aruco/loc/pose_cov с covariance floors (см. 5.5)
```

Сопутствующие топики:
- `/aruco/reset_counter` (`std_msgs/UInt8`, reliable + transient_local, KeepLast 1) — счётчик
  пересчётов offset для сброса EKF2.
- `/aruco/map_offset` (`geometry_msgs/TransformStamped`, latched) — сам offset
  `T_world_aruco` (`frame_id=map`, `child_frame_id=aruco_map`). Публикуется в
  single-режиме на каждом пересчёте; пригодится offboard/координатору роя.

---

## Краткая суть (3 строки)

```
pose_cov      = где КАМЕРА относительно ПОЛЯ меток        (чистое зрение)
offset        = где ПОЛЕ меток в МИРЕ                      (откалибровано 1 раз по EKF)
world_pose    = offset * (камера в поле) * (тело отн. камеры) = где ДРОН в МИРЕ (ENU)
```

---

# Режим роя (`swarm_mode`)

Для роя нужны НЕ относительные координаты каждого дрона (от его точки взлёта), а
**абсолютные в одной общей системе**. Общая система — `aruco_map` (один и тот же
физический лист меток для всех дронов). Включается параметром
`swarm_mode:=true` у `aruco_tf_node`.

## Чем swarm-режим отличается

| | single (по умолчанию) | swarm (`swarm_mode:=true`) |
|---|---|---|
| Нуль (0,0,0) выхода | точка взлёта дрона (EKF `map`) | центр листа `aruco_map` (общий) |
| Per-drone offset | да (привязка к взлёту) | нет (offset = identity) |
| Зависит от TF `map→base_link` (EKF) | да | **нет** |
| `header.frame_id` у `/aruco/world_pose` | `map` | `aruco_map` |
| Координаты сравнимы между дронами | нет | **да** |

Формула в swarm-режиме (никакого offset, EKF-мир не нужен):

```
   T_aruco_base = T_aruco_camera * T_camera_base
                  └ вход pose_cov   └ статический TF камера->тело

   /aruco/world_pose = T_aruco_base   (тело дрона в aruco_map, frame_id = aruco_map)
```

```
   single:   map ──offset──> aruco_map ──vision──> base   (нуль у каждого свой)
   swarm:    aruco_map ───────vision────────────> base    (нуль общий для всех)
```

> Допущение: лист гравитационно выровнен (лежит горизонтально, Z вверх), иначе
> `aruco_map` не ENU и её нельзя кормить в VPE как ENU. Для напольной карты — ок.

## Конфликт «нуль включения ↔ нуль карты» (главное!)

Внутри узла конфликта нет (ветка роя изолирована). Конфликт — на стороне PX4:

```
   PX4 при включении: нуль local = ТОЧКА ВКЛЮЧЕНИЯ ПИТАНИЯ
   aruco_map:         нуль       = ЦЕНТР ЛИСТА  (другая точка!)
```

Если EKF уже построил оценку от точки включения, а потом «вдруг» приходит поза в
метрах от центра листа -> большая невязка. EKF2 либо **отбросит** её как выброс
(дрон не локализуется), либо **телепортируется** (опасно в воздухе).

## Чеклист настройки PX4, чтобы конфликта НЕ было

- [ ] **EV стабилен ДО арминга.** На реальном дроне сначала проверить PX4 local
      estimate, `/aruco/world_pose`, frames и covariance. В SITL simulated GPS
      удерживает estimator до автоматического ArUco/range handoff.
- [ ] **Vision-primary, без конкурирующего origin.** Зрение — единственный
      абсолютный источник позиции и курса (GPS-aiding выключить). Маску внешнего
      зрения / EV position+yaw **проверить под свою версию PX4** — детали меняются.
- [ ] **`reset_counter` прокинут** до EKF2. Узел бампит его при commit нового
      offset. Обычный dropout/reacquisition со старым offset counter не меняет.
- [ ] **Режим не менять в полёте** (single <-> swarm только в launch).
- [ ] Лист **горизонтальный** (gravity-aligned), Z вверх.

При выполнении этого PX4-local фактически совпадает с `aruco_map` (с поправкой
ENU->NED), и `vehicle_odometry` каждого дрона — в общем абсолютном фрейме.

## Перевод setpoint'ов (если PX4 держит СВОЙ локальный нуль)

Если EKF не принял `aruco_map` за свой фрейм (держишь PX4-local от взлёта
намеренно), команды надо переводить. Связующее звено — **per-drone offset** между
PX4-local и `aruco_map`:

```
   Известно в один и тот же момент времени:
     T_aruco_base    — тело в aruco_map   (из /aruco/world_pose, swarm)
     T_pxlocal_base  — тело в PX4-local    (из vehicle_local_position, NED->ENU)

   Offset (считается 1 раз, дальше ~константа):
     T_pxlocal_aruco = T_pxlocal_base * (T_aruco_base)^-1     -> топик /aruco/map_offset

   Перевод цели роя в локальный setpoint PX4:
     p_pxlocal = T_pxlocal_aruco * p_aruco_target             (затем ENU->NED)
```

Пример (псевдо-Python на стороне offboard):

```python
# offset обновляется редко (дрон ~неподвижен / усреднение)
T_pxlocal_aruco = T_pxlocal_base @ inv(T_aruco_base)   # публикуем в /aruco/map_offset

# на каждый setpoint от координатора (в aruco_map):
p_local_enu = T_pxlocal_aruco @ p_aruco_target
sp = enu_to_ned(p_local_enu)        # PX4 TrajectorySetpoint ждёт NED
publish(sp)
```

> Узел публикует offset `T_world_aruco` в **latched-топик `/aruco/map_offset`**
> (`geometry_msgs/TransformStamped`, `frame_id=world`, `child_frame_id=aruco_map`)
> в **single-режиме** на каждом пересчёте offset. В swarm-режиме offset = identity,
> и в этот топик ничего не публикуется (per-drone PX4-local↔aruco offset считается
> на уровне offboard, см. формулу выше). Topic latched (transient_local) — поздний
> подписчик (offboard) сразу получает последнее значение.

## Архитектура координации роя

```
            ┌──────────── координатор роя (в aruco_map) ─────────────┐
            │  планирует траектории всех дронов в ОДНОМ фрейме aruco_map │
            └──────┬──────────────────┬──────────────────┬───────────┘
                   │ setpoint(aruco)   │ setpoint(aruco)   │
             drone1 offboard     drone2 offboard     drone3 offboard
                   │ -> PX4-local      │ -> PX4-local      │ -> PX4-local
                 PX4_1               PX4_2               PX4_3
                   ▲                   ▲                   ▲
        /aruco/world_pose (aruco_map) — абсолютная поза каждого дрона
```

- Абсолютные позы дронов координатор берёт из `/aruco/world_pose` (swarm-режим).
- Планирует и шлёт цели в `aruco_map`.
- Per-drone offboard переводит цель в локальный фрейм PX4 (если PX4 не в aruco-фрейме).

## Подводные камни роя

- **Покрытие меток.** При потере меток дрон дрейфит по IMU до их возврата.
  Следи, чтобы на рабочей высоте каждый дрон видел >=3 метки (большие 0.4 м
  ловятся издалека, мелкие — нет; см. таблицу дальностей в TUNING_AND_VALIDATION.md).
- **Единый курс.** Зрение задаёт общий «север» (ось X карты) -> дроны согласованы
  по направлению. При потере зрения курс начнёт уходить.
- **Синхронизация времени** (chrony/PTP) между дронами — иначе позы в общем
  фрейме не бьются по времени при близких манёврах.
- **Неймспейсы/TF.** У каждого дрона свой TF-tree в своём неймспейсе; `aruco_map`
  общий ПО СМЫСЛУ. Собирай позы в координаторе как сообщения, не через сквозной TF.
- **Несколько листов** — только с согласованными координатами в одном
  `markers.txt`-пространстве (общий нуль), иначе фреймы разъедутся.

## Запуск роя

```bash
ros2 launch aruco_det_loc aruco_composed.launch.py \
  swarm_mode:=true \
  cpu_params_file:=$(ros2 pkg prefix aruco_det_loc)/share/aruco_det_loc/config/cpu_rpi5.yaml
```

Проверка, что выход в общем фрейме:

```bash
ros2 topic echo --once /aruco/world_pose
# header.frame_id должен быть "aruco_map", а не "map"
```
