# PX4 EKF2 — настройка и отладка VPE/VIO (ArUco → `/fmu/in/vehicle_visual_odometry`)

Справочник по параметрам PX4 и методам отладки узла `pose_subscriber_tf2`
(`VpePublisher`), чтобы не приходилось ресёрчить документацию PX4.

> **Прошивка: PX4 v1.15.4.** Все имена параметров, битовые маски, поля uORB-топиков
> и команды NSH в этом документе соответствуют 1.15.x. На других версиях
> (особенно 1.16+) маски `EKF2_*_CTRL` и набор `estimator_aid_src_*` могут отличаться.

Связанные доки: [`README.md`](README.md) (контракт узла, топики, параметры узла),
а также `aruco_det_loc/` (источник позы).

---

## 0. TL;DR

- **EV-ядро у тебя уже выставлено правильно и провалидировано на борту** (см. §2, группа A) —
  не трогай его «по факту полёта».
- Перед первым полётом разово выстави **группу B** (§3): главное — `EKF2_EV_DELAY`,
  высоту на лидар, и indoor-лимиты скорости/наклона ради безопасности.
- В полёте дотюнивается только **группа C** (§4): регуляторы, `EKF2_EVP_NOISE`,
  тонкая доводка `EKF2_EV_DELAY`.
- Готовый скрипт `param set` — §5.
- Проверка валидности данных — §6, команды MAVLink-консоли — §7, траблшутинг — §8,
  чеклист готовности к полёту — §9.

---

## 1. Контекст конфигурации (на чём летим)

Indoor, **без GPS, без магнитометра, без баро**:

| Источник | Что даёт | Параметр |
|---|---|---|
| ArUco (VPE/VIO) | гориз. позиция XY + yaw (+верт. позиция Z) | `EKF2_EV_CTRL` |
| MicroAir MTF-02 | высота (ToF-лидар) + optical flow (скорость/позиция) | `EKF2_RNG_CTRL`, `EKF2_OF_CTRL` |
| IMU | ускорения/угловые скорости (всегда) | `EKF2_IMU_CTRL` |

Yaw держится **только** на зрении (магнитометра нет) → корректность yaw от ArUco критична.
Roll/pitch EKF берёт из IMU (гравитация), **из зрения roll/pitch НЕ берутся** при `EV_CTRL=11/9`.

---

## 2. Группа A — оставить как есть (уже корректно, провалидировано)

Это рабочая основа. **Не менять.**

| Параметр | Значение | Зачем |
|---|---|---|
| `EKF2_GPS_CTRL` | 0 | GPS выключен (indoor) |
| `SYS_HAS_GPS` | 0 | нет GPS |
| `EKF2_MAG_TYPE` | 5 | магнитометр не используется |
| `SYS_HAS_MAG` | 0 | нет магнитометра |
| `EKF2_BARO_CTRL` | 0 | баро выключено (высота от лидара/зрения) |
| `EKF2_RNG_CTRL` | 2 | лидар-высота включена (см. §3.4) |
| `EKF2_OF_CTRL` | 1 | optical flow включён |
| `UXRCE_DDS_SYNCT` | 1 | **timesync ON** — без него `timestamp_sample` бессмыслен |
| `COM_ARM_WO_GPS` | 1 | арм без GPS разрешён |
| `EKF2_EV_NOISE_MD` | 0 | шум EV из сообщения с флором по `EVP/EVA_NOISE` (см. §3.4) |
| `EKF2_EVP_GATE` | 5 | гейт инноваций EV-позиции, 5σ |
| `EKF2_EV_POS_X/Y/Z` | 0 | **правильно 0**: экстринсик камеры применяется на стороне ArUco/tf (поза уже `base_link`). Дублировать lever-arm здесь нельзя. |
| `EKF2_EV_QMIN` | 0 | минимальный `quality` для приёма EV. **0 = не гейтит** (см. §6, гочи с `quality`) |
| `COM_ARM_EKF_POS/VEL/YAW/HGT` | 0.5/0.5/0.5/1.0 | пред-арм пороги инноваций; твои test_ratio ~1e-4 проходят с запасом |

---

## 3. Группа B — выставить ДО полёта (детерминированно, не «по факту»)

### 3.1 Сводная таблица

| Параметр | Сейчас | Рекомендую | Тип правки | Раздел |
|---|---|---|---|---|
| `EKF2_EV_DELAY` | 50 | **0** (старт) | под нашу систему | §3.2 |
| `EKF2_HGT_REF` | 3 | **2** (Range) | стратегия высоты | §3.3 |
| `EKF2_EV_CTRL` | 11 | **11** (оставить) или 9 | стратегия высоты | §3.3 |
| `MPC_XY_VEL_MAX` | 12 | **1.5–2.0** | безопасность indoor | §3.5 |
| `MPC_XY_CRUISE` | 5 | **1.0–2.0** | безопасность indoor | §3.5 |
| `MPC_TILTMAX_AIR` | 45 | **20–25** | FOV + безопасность | §3.5 |
| `MPC_Z_VEL_MAX_UP` | 3 | **1.0** | безопасность indoor | §3.5 |
| `MPC_Z_VEL_MAX_DN` | 1.5 | **1.0** | безопасность indoor | §3.5 |
| `MPC_TKO_SPEED` | 1.5 | **1.0** | мягкий взлёт | §3.5 |

### 3.2 `EKF2_EV_DELAY` — 50 → 0

**Что это:** задержка EV-измерения относительно IMU, в **миллисекундах**. EKF2
кладёт EV-выборку в свой буфер по времени `timestamp_sample − EKF2_EV_DELAY` и
сопоставляет с IMU той же эпохи.

**Ключевой момент (частая путаница):** `EV_DELAY` — это ошибка в **самой метке
времени** сообщения, а НЕ возраст сообщения при доставке.
- Те ~40 мс, что ты видишь как разницу `timestamp_sample` ↔ `timestamp` в
  `vehicle_visual_odometry`, — это транспортная задержка (ArUco-обработка + DDS).
  EKF2 её компенсирует сам через буфер по `timestamp_sample`, пока она < `EKF2_DELAY_MAX` (200 мс).
- `timestamp_sample` у нас = **время захвата кадра** (`header.stamp`, протянут через
  весь пайплайн) + timesync переводит его в часовой домен PX4 → остаточная ошибка ≈ 0.

Поэтому **корректное стартовое значение — 0**, а 50 завышено: оно искусственно
сдвигает измерение на 50 мс «в прошлое» сильнее реальности → лаг/ошибка позиции
в динамике (на стенде стоя — незаметно, в полёте — поедет).

**Как дотюнить (в полёте, §4):** полетай вперёд-назад/влево-вправо и смотри
`estimator_aid_src_ev_pos.innovation`. Если позиционная инновация стабильно
смещена против направления движения — увеличь delay на 5–10 мс. Диапазон поиска 0…20 мс.

### 3.3 Высота: `EKF2_HGT_REF` 3 → 2, и судьба `EKF2_EV_CTRL`

**Проблема текущей схемы (`HGT_REF=3`, Vision):** Z по ArUco — самая
слабонаблюдаемая ось (дальность от камеры до плоскости меток) и именно туда
подмешается остаточная ошибка экстринсика/pitch. Высоту лучше доверить лидару.

**`EKF2_HGT_REF`** выбирает **референсный** (датумный) источник высоты — то, что
задаёт ноль высоты. При этом ВСЕ включённые источники высоты всё равно фьюзятся.

**Рекомендация:**
- `EKF2_HGT_REF = 2` (**Range**) — лидар MTF-02 становится референсом высоты (он точнее indoor и ты ему доверяешь).
- `EKF2_EV_CTRL` — два варианта:
  - **`11` (оставить, рекомендую для старта):** EV-Z остаётся слабым (`EVP_NOISE`=0.1 м)
    бэкапом, а range (0.01 м) доминирует ~100:1. Сохраняется **резерв по высоте**, если
    лидар на миг отвалится.
  - **`9` (h-pos + yaw, без EV-Z):** чистое разделение «зрение=XY+yaw, лидар=высота».
    Бери, если в логах увидишь, что EV-Z «дерётся» с range (растут hgt-инновации).

**Обязательно переснять на стенде после смены** (см. §6.5): `cs_ev_hgt` может стать
false, но `z_valid` обязан остаться true (высота от range), `hgt_test_ratio` мал.

**Fallback:** если высота поведёт себя плохо — вернись к провалидированной связке
`HGT_REF=3` + `EV_CTRL=11` (она рабочая, это твой безопасный откат).

#### Битовая маска `EKF2_EV_CTRL`
| Бит | Значение | Что включает |
|---|---|---|
| 0 (1) | Horizontal position | гориз. позиция XY от EV |
| 1 (2) | Vertical position | верт. позиция Z от EV |
| 2 (4) | Velocity | скорость от EV (у нас не используется) |
| 3 (8) | Yaw | курс от EV |

- `11` = 1+2+8 = XY + Z + yaw (текущее)
- `9` = 1+8 = XY + yaw
- `3` = 1+2 = XY + Z (без yaw — **не наш случай**, yaw нужен)

#### `EKF2_HGT_REF`
| Значение | Источник-референс высоты |
|---|---|
| 0 | Barometric |
| 1 | GPS |
| 2 | **Range (лидар)** ← рекомендую |
| 3 | Vision (текущее) |

### 3.4 Шумы EV (понимать, но менять — позже, §4)

| Параметр | Значение | Смысл |
|---|---|---|
| `EKF2_EVP_NOISE` | 0.1 м | СКО шума EV-позиции (флор). Variance = 0.1² = 0.01 м². |
| `EKF2_EVA_NOISE` | 0.1 рад | СКО шума EV-угла (yaw). ≈ 5.7°. |
| `EKF2_EV_NOISE_MD` | 0 | `0` = берётся ковариация из сообщения, но **не ниже** `EVP/EVA_NOISE` (floor = `max(сообщение, параметр)`). `1` = всегда параметр. |

**Что это объясняет:** наш паблишер шлёт `position_variance=1e-4` (СКО 1 см,
оптимистично). EKF при `MD=0` floor-ит до `0.1²=0.01` → в `estimator_aid_src_ev_pos`
ты видишь `observation_variance: 0.01`. Это нормально и безопасно (EKF сам себя
защищает от нашей переуверенности).

### 3.5 Лимиты безопасности indoor (MPC)

12 м/с макс. горизонтальной скорости в помещении — опасно. До полёта:

| Параметр | Сейчас | Рекомендую | Смысл |
|---|---|---|---|
| `MPC_XY_VEL_MAX` | 12 | 1.5–2.0 | абсолютный потолок гориз. скорости |
| `MPC_XY_CRUISE` | 5 | 1.0–2.0 | целевая гориз. скорость авто-режимов |
| `MPC_TILTMAX_AIR` | 45 | 20–25 | макс. наклон. Меньше наклон → fisheye **не теряет карту из FOV** (иначе дропаут EV) |
| `MPC_Z_VEL_MAX_UP` | 3 | 1.0 | верт. скорость вверх |
| `MPC_Z_VEL_MAX_DN` | 1.5 | 1.0 | верт. скорость вниз |
| `MPC_TKO_SPEED` | 1.5 | 1.0 | скорость взлёта |

> Удобно: `MPC_XY_VEL_ALL` (сейчас −10 = выкл.) — если задать положительное
> значение, оно разом ограничивает все гориз. скорости. Но прозрачнее править
> `MPC_XY_VEL_MAX`/`MPC_XY_CRUISE` по отдельности.

---

## 4. Группа C — тюнить уже в полёте

| Что | Старт | Метод доводки |
|---|---|---|
| Регуляторы `MC_*RATE_*`, `MPC_XY_P`, `MPC_Z_P` | дефолты | править ТОЛЬКО при раскачке/вялости |
| Висение `MPC_THR_HOVER` | 0.55 | **не трогать руками**: `MPC_USE_HTE=1` (включён) оценивает hover-thrust автоматически |
| `EKF2_EV_DELAY` | 0 | по `estimator_aid_src_ev_pos.innovation` в динамике (§3.2) |
| `EKF2_EVP_NOISE` | 0.1 | когда стабильно — можно зажать до 0.05–0.03 для более цепкого удержания по ArUco (растёт доверие зрению, позиция «жёстче», но шумнее) |

---

## 5. Готовый скрипт `param set` (группа B)

Залить в NSH (MAVLink-консоль) или через QGC. После — **перезагрузка** полётника.

```sh
# --- VPE/EV: задержка под нашу систему ---
param set EKF2_EV_DELAY 0

# --- Высота на лидар (range — референс), EV оставляем XY+Z+yaw как бэкап Z ---
param set EKF2_HGT_REF 2
param set EKF2_EV_CTRL 11
# если EV-Z будет драться с range — заменить на: param set EKF2_EV_CTRL 9

# --- Безопасность indoor ---
param set MPC_XY_VEL_MAX 2
param set MPC_XY_CRUISE 1.5
param set MPC_TILTMAX_AIR 25
param set MPC_Z_VEL_MAX_UP 1
param set MPC_Z_VEL_MAX_DN 1
param set MPC_TKO_SPEED 1

param save
```

Проверить, что применилось: `param show EKF2_EV_DELAY` и т.д.

---

## 6. Проверка валидности данных (ArUco → VIO → фьюз)

Идём по потоку: VPE публикует → конверсия верна → данные пришли в PX4 → EKF фьюзит → решение трекает.

### 6.1 Сторона ROS 2 — что выдаёт VPE
```bash
# узел публикует, PX4 подписан:
ros2 topic info -v /fmu/in/vehicle_visual_odometry
#   Publisher: pose_subscriber_tf2
#   Subscription: _CREATED_BY_BARE_DDS_APP_  ← это PX4/uXRCE

# содержимое одного сообщения:
ros2 topic echo --once /fmu/in/vehicle_visual_odometry

# частота (должна ≈ частоте /aruco/world_pose):
ros2 topic hz /fmu/in/vehicle_visual_odometry

# включить отправку (предохранитель):
ros2 param set /pose_subscriber_tf2 allow_send_vio true
```
**Что должно быть в сообщении:**
- `pose_frame: 1` (NED), `velocity_frame: 0` (скорость не передаём — ок)
- `position`/`q` — конечные, не NaN
- `timestamp_sample` ≠ 0 (= `header.stamp`), `timestamp` = 0 (**это норма**: либа
  оставляет 0, PX4 проставит на приёме — см. README/код интерфейс-либы)
- `reset_counter` = текущий счётчик, `quality: 0`

> **Гоча с `quality: 0`:** мы шлём `quality=0` («не задано»). Это ОК **только пока
> `EKF2_EV_QMIN=0`**. Если когда-нибудь поставишь `EKF2_EV_QMIN>0` — EKF начнёт
> отбраковывать наши сообщения (0 < QMIN). Либо не трогай QMIN, либо начни слать
> реальный `quality` из узла.

### 6.2 Валидность конверсии ENU→NED
Сравни `/aruco/world_pose` (ENU/FLU) и `vehicle_visual_odometry` (NED/FRD):
- `NED.x (position[0]) == ENU.y`
- `NED.y (position[1]) == ENU.x`
- `NED.z (position[2]) == −ENU.z`

Численно ошибка должна быть на уровне мм (у нас p95 ≈ 1.5 мм по позиции, 0.11° по кватерниону).

### 6.3 Known-displacement (позиция) — **ground-truth тест**
Сдвинь дрон на известную величину (напр. 0.5 м) по одной оси и проверь, что
`position` в `vehicle_visual_odometry` (или `vehicle_local_position.x/y/z`)
изменилась **на ту же величину и в правильную сторону**. Это проверка, что поза
ArUco соответствует реальности, а не только что мост честно конвертит.

### 6.4 Поворот (yaw) — **ground-truth тест**
Поверни дрон на месте на известный угол (напр. +90° по часовой, вид сверху) и
сравни yaw из кватернионов двух записей. Извлечение yaw из `q=[w,x,y,z]`:

```
yaw = atan2( 2*(w*z + x*y),  1 − 2*(y² + z²) )   # радианы
```
В NED поворот по часовой (вид сверху) должен **увеличивать** yaw. Масштаб ~1:1
(±неточность поворота рукой). Альтернатива — поле `heading` в `vehicle_local_position`.

> Проверено: +90° CW → Δyaw ≈ +99° (знак верный, масштаб 1:1, лишнее — ручная неточность).

### 6.5 Сторона PX4 — фьюзит ли EKF (главное)
В MAVLink-консоли (см. §7) — критерий «зрение валидно и фьюзится»:
- `listener estimator_status_flags` → `cs_ev_pos / cs_ev_yaw` (и `cs_ev_hgt`, если высота от EV) = **true**
- `listener estimator_aid_src_ev_pos` и `...ev_yaw` → `fused: True`, `innovation_rejected: False`, `test_ratio < 1.0` (у нас ~1e-4)
- `listener estimator_status` → `filter_fault_flags: 0`, `innovation_check_flags: 0`, `pos_test_ratio < 1`
- `listener vehicle_local_position` → `xy_valid/z_valid = True`, `heading_good_for_control: True`, `dead_reckoning: False`

После смены `HGT_REF=2`: `z_valid` обязан остаться **true** (высота берётся от range,
даже если `cs_ev_hgt` стал false).

---

## 7. MAVLink-консоль — справочник команд

**Как зайти:** QGroundControl → **Analyze Tools → MAVLink Console**.
(Либо `mavlink_shell.py <порт>` с хоста, если настроен.) Внутренние uORB-топики
PX4 (`estimator_aid_src_*`, `estimator_status_flags` и т.д.) в ROS-мост не
проброшены, поэтому смотреть их можно только здесь.

> `listener <топик> [N]` — печатает N сэмплов (по умолчанию 1). Напр. `listener vehicle_visual_odometry 5`.

### 7.1 Самое важное (этим закрывается отладка узла)

| Команда | Что показывает | На что смотреть / «хорошо» |
|---|---|---|
| `listener vehicle_visual_odometry` | **вход** EV в PX4 (то, что прислал VPE) | `position/q` конечные, `pose_frame=1`, `timestamp_sample`≠0, `timestamp`≠0 (PX4 проставил), `reset_counter` верный. Возраст `timestamp_sample` к `timestamp` ≈ латентность (~40 мс) |
| `listener estimator_status_flags` | **именованные** булевы флаги фьюза (без расшифровки масок!) | `cs_ev_pos=True`, `cs_ev_yaw=True`, `cs_ev_hgt` (если высота от EV), `cs_tilt_align/cs_yaw_align=True` |
| `listener estimator_aid_src_ev_pos` | фьюз EV-позиции (2 инстанции) | `fused=True`, `innovation_rejected=False`, `test_ratio<1`, `innovation` малы |
| `listener estimator_aid_src_ev_yaw` | фьюз EV-курса | то же |
| `listener estimator_aid_src_ev_hgt` | фьюз EV-высоты (если `EV_CTRL` бит 1 включён) | то же; после `HGT_REF=2`+`EV_CTRL=9` топик будет неактивен |
| `ekf2 status` | здоровье обеих инстанций EKF2 | `healthy`, `attitude: 1`, `local position: 1`, нет роста `IMU message missed` |
| `uxrce_dds_client status` | мост uXRCE-DDS | `Running, connected`, `timesync converged: true`, растёт `Payload rx` |

### 7.2 Состояние оценщика и итог

| Команда | Что показывает | На что смотреть |
|---|---|---|
| `listener estimator_status` | сводный статус EKF2 | `filter_fault_flags=0`, `innovation_check_flags=0`, `pos_horiz_accuracy`/`pos_vert_accuracy` (у нас ~2 см / 0.8 см), `pre_flt_fail_*=False`. Маски — см. §7.4 |
| `listener vehicle_local_position` | **итоговая** локальная поза (выход EKF) | `x/y/z`, `heading`, `xy_valid/z_valid=True`, `heading_good_for_control=True`, `dead_reckoning=False`, `dist_bottom_valid=True`, `eph/epv` |
| `listener vehicle_odometry` | одометрия-выход EKF2 (для ROS-моста `/fmu/out/...`) | следует ли за ArUco-позой |
| `listener vehicle_attitude` | ориентация от EKF | сверка с EV (помни: roll/pitch EKF из IMU, не из EV) |
| `listener estimator_innovation_test_ratios` | test_ratio **всех** датчиков разом | `ev_hpos/ev_vpos/ev_yaw` < 1 |
| `listener estimator_innovations` | сырые инновации всех датчиков | поиск «улетевшего» источника |
| `listener timesync_status` | синхронизация часов ROS↔PX4 | `estimated_offset` стабилен, `round_trip_time` мал (~6 мс) |

### 7.3 Инфраструктура / частоты / параметры

| Команда | Назначение |
|---|---|
| `uorb top` | живые частоты публикации всех топиков. `uorb top -1` — один снимок. Проверить, что `vehicle_visual_odometry` идёт с ожидаемой частотой |
| `uorb status` | список топиков, число публикаций/подписчиков |
| `work_queue status` | загрузка рабочих очередей (нет ли перегруза) |
| `param show EKF2_EV*` | показать значения параметров по маске |
| `param set <NAME> <val>` / `param save` | задать / сохранить параметр |
| `dmesg` | сообщения ядра/драйверов (старт, ошибки) |
| `mavlink status` | состояние MAVLink-линков и стримов |
| `listener sensor_optical_flow` | сырой optical flow от MTF-02 |
| `listener distance_sensor` | сырой лидар-дальномер (высота) MTF-02 |

### 7.4 Расшифровка битовых масок

Удобнее `listener estimator_status_flags` (готовые булевы поля), но если читаешь сырые маски:

**`estimator_status.control_mode_flags`** (фьюз-режим EKF2, биты):
| Бит | Имя | | Бит | Имя |
|---|---|---|---|---|
| 0 | tilt_align | | 11 | gps_hgt |
| 1 | yaw_align | | **12** | **ev_pos** |
| 2 | gps | | **13** | **ev_yaw** |
| 3 | opt_flow | | **14** | **ev_hgt** |
| 4 | mag_hdg | | 24 | ev_vel |
| 9 | baro_hgt | | 26 | vehicle_at_rest |
| 10 | rng_hgt | | 29 | inertial_dead_reckoning |

> Пример: `control_mode_flags=2214622219` → стоят биты 0,1,3,10,12,13,14,26 =
> tilt/yaw align + opt_flow + rng_hgt + **ev_pos+ev_yaw+ev_hgt** + at_rest. Это и есть «всё фьюзится».

**`estimator_status.solution_status_flags`** (валидность решения, биты):
| Бит | Имя | Бит | Имя |
|---|---|---|---|
| 0 | attitude valid | 5 | pos vert abs valid |
| 1 | velocity horiz valid | 6 | pos vert AGL valid |
| 2 | velocity vert valid | **7** | **const pos mode** (плохо, если 1 в полёте) |
| 3 | pos horiz rel valid | 8 | pred pos horiz rel |
| 4 | pos horiz abs valid | 9 | pred pos horiz abs |

> Пример: `solution_status_flags=895` → биты 0–6,8,9 set, **бит 7 = 0** (НЕ в
> constant-position-mode) → здоровое решение.

---

## 8. Траблшутинг

| Симптом | Вероятная причина | Действие |
|---|---|---|
| `listener vehicle_visual_odometry` пустой | VPE не шлёт (`allow_send_vio=false`) или мост не поднят | `ros2 param set /pose_subscriber_tf2 allow_send_vio true`; `uxrce_dds_client status` |
| Сообщения идут, но `cs_ev_pos=False` | `EKF2_EV_CTRL` бит не включён; либо `EV_QMIN>0` при `quality=0`; либо `timestamp_sample` невалиден | проверить `EKF2_EV_CTRL`, `EKF2_EV_QMIN=0`, `UXRCE_DDS_SYNCT=1` |
| `innovation_rejected=True` / `test_ratio>1` | поза ArUco расходится с предсказанием (плохая поза/экстринсик/карта) или `EV_DELAY` неверен | сверить позу (§6.3/6.4), экстринсик `base_link→camera_optical_1`, `markers.txt`; подстроить `EV_DELAY` |
| Позиция лагает/опережает в динамике | `EKF2_EV_DELAY` завышен/занижен | тюнинг `EV_DELAY` по `estimator_aid_src_ev_pos.innovation` (§3.2) |
| Высота «дышит»/дерётся | EV-Z (слабый) конфликтует с range | `EKF2_HGT_REF=2`, при необходимости `EKF2_EV_CTRL=9` (§3.3) |
| EV пропадает при манёврах | fisheye теряет карту из FOV при наклоне | уменьшить `MPC_TILTMAX_AIR` (§3.5) |
| `dead_reckoning=True` | потеряны все абсолютные источники (EV+flow) | проверить детекцию ArUco и flow; `EKF2_NOAID_TOUT` (5 с) |
| `timestamp` всегда 0 в NSH | (на самом деле норма для ROS-стороны) в NSH PX4 должен проставить ≠0 | если в NSH тоже 0 — проблема приёма uXRCE; `uxrce_dds_client status` |
| Спайки латентности (>100 мс) | перегруз CPU на companion (ArUco) | CPU-affinity ArUco-нод; следить `image_to_pose_ms`; держать < `EKF2_DELAY_MAX`=200 мс |

---

## 9. Чеклист готовности к полёту (на земле, винты сняты)

- [ ] Группа B выставлена (`EKF2_EV_DELAY=0`, высота на range, indoor-лимиты), `param save`, ребут
- [ ] `uxrce_dds_client status` → connected, timesync converged
- [ ] `vehicle_visual_odometry`: `pose_frame=1`, поля конечные, `timestamp_sample`≠0
- [ ] known-displacement (§6.3) и yaw-поворот (§6.4) — знак и масштаб верны
- [ ] `estimator_status_flags`: `cs_ev_pos`, `cs_ev_yaw` = True; высота валидна (`z_valid`)
- [ ] `estimator_aid_src_ev_pos/yaw`: `fused=True`, `test_ratio<1`, `innovation_rejected=False`
- [ ] `estimator_status`: `filter_fault_flags=0`, `pos_horiz_accuracy` мал (~см)
- [ ] `vehicle_local_position`: `xy_valid/z_valid/heading_good_for_control=True`, `dead_reckoning=False`
- [ ] экстринсик `base_link→camera_optical_1` сверен с реальным креплением (остаточный pitch снят)
- [ ] (перед точностью) плоский полигон → ревёрт band-aid PnP (`pnp_reprojection_error_px` 12→4–6, `pnp_iterations` 100→16–32); fisheye-калибровка

---

*Контракт VPE (`/aruco/world_pose` = `base_link` ENU/FLU, `frame_id=map`,
`header.stamp`=время захвата) — стабилен, менять не нужно. Этот документ только про
PX4-сторону и отладку.*
