# Offboard Control

ROS 2-нода для управления мультикоптером в режиме **Offboard** через PX4 (uXRCE-DDS). Поддерживает навигацию по целевым точкам, полёт в разных системах координат, телеметрию и набор сервисов для поточечного и траекторного управления.

---

## Содержание

- [Требования](#требования)
- [Сборка](#сборка)
- [Запуск](#запуск)
- [Параметры ноды и launch](#параметры-ноды-и-launch)
- [Сервисы](#сервисы)
- [Действия (actions)](#действия-actions)
- [Топики ноды](#топики-ноды)
- [Системы координат (frame_id)](#системы-координат-frame_id)
- [Примеры вызовов](#примеры-вызовов)
- [Тестовые скрипты](#тестовые-скрипты)
- [Возбудитель (run_excitation)](#возбудитель-system-id-excitation--action-run_excitation)
- [RTL и failsafe-политика](#rtl-и-failsafe-политика-ноды)
- [Фрейм heading](#фрейм-heading-управление-vs-отображение)
- [RC-перехват](#rc-перехват-kill-switch--стик--для-всех-манёвров)

---

## Требования

- ROS 2 (Humble / Iron или совместимый)
- Пакеты: `px4_msgs`, `px4_ros_com`, `offboard_interfaces`, `tf2_ros`, `tf2_geometry_msgs`, `geometry_msgs`, `std_srvs`
- PX4 с поддержкой uXRCE-DDS (SITL или борт) и при необходимости **MicroXRCEAgent** на связи с PX4

---

## Сборка

Из корня workspace:

```bash
cd ~/sverk_ws
source /opt/ros/<distro>/setup.bash
colcon build --packages-select offboard_control
source install/setup.bash
```

---

## Запуск

### Запуск ноды (без launch)

Обычный режим (реальный дрон / один агент):

```bash
ros2 run offboard_control offboard_control
```

Режим симулятора (топики PX4 с суффиксом `_v1` и т.п.):

```bash
ros2 run offboard_control offboard_control --ros-args -p simulator:=true
```

### Запуск через launch

Базовый запуск:

```bash
ros2 launch offboard_control offboard_control.launch.py
```

С переопределением параметров:

```bash
# Другая система координат (например, odom вместо map)
ros2 launch offboard_control offboard_control.launch.py map_frame_id:=odom

# Таймаут преобразований и скорость по умолчанию
ros2 launch offboard_control offboard_control.launch.py transform_timeout:=1.0 default_speed:=1.0

# Таймауты и режимы
ros2 launch offboard_control offboard_control.launch.py \
  offboard_timeout:=5.0 \
  arming_timeout:=6.0 \
  land_only_in_offboard:=true
```

---

## Параметры ноды и launch

Параметры, которые нода реально использует:

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `body_frame_id` | string | `base_link` | Имя фрейма корпуса в TF |
| `map_frame_id` | string | `map` | Локальная СК (map/odom и т.д.) |
| `aruco_map_frame_id` | string | `aruco_map` | Префикс/имя для маркерной СК |
| `transform_timeout` | double | 0.5 | Таймаут ожидания TF (с) |
| `offboard_timeout` | double | 10.0 | Таймаут перехода в OFFBOARD (с) |
| `arming_timeout` | double | 5.0 | Таймаут арминга (с) |
| `default_speed` | double | 0.5 | Скорость по умолчанию для navigate (м/с) |
| `land_only_in_offboard` | bool | true | Разрешать land только в OFFBOARD |
| `check_kill_switch` | bool | true | Учитывать kill switch с пульта |
| `local_position_timeout` | double | 2.0 | Таймаут локальной позиции (с) |
| `global_position_timeout` | double | 10.0 | Таймаут глобальной позиции (с) |
| `battery_timeout` | double | 2.0 | Таймаут батареи (с) |
| `manual_control_timeout` | double | 0.0 | Таймаут manual control (с); 0 = отключено; при >0 потеря RC в полёте → onboard failsafe |
| `simulator` | bool | false | Режим симулятора (другие топики PX4) |
| `enforce_rc_in_sim` | bool | false | Включить RC-перехват и в симуляторе (SITL с джойстиком) |
| `rc_stick_abort_thresh` | double | 0.05 | Порог отклонения стика (roll/pitch/yaw), при котором управление отдаётся пилоту |
| `enforce_freshness` | bool | true | Watchdog свежести одометрии: устаревшая поза в offboard → onboard failsafe |
| `telemetry_rate_hz` | double | 0.0 | Частота топика `~/telemetry`; 0 = выключен; меняется на лету через `ros2 param set` |
| `geofence_enable` | bool | false | Общая геозона (цилиндр) для всех манёвров; цели вне зоны отклоняются, вылет → onboard failsafe |
| `geofence_radius` | double | 10.0 | Радиус геозоны (м) |
| `geofence_center_x`, `geofence_center_y` | double | 0.0 | Центр геозоны в map (ENU) |
| `alt_floor`, `alt_ceil` | double | 0.0 / 50.0 | Коридор высоты геозоны (м, ENU z) |
| `max_speed` | double | 0.0 | Лимит скорости для `navigate`/`set_velocity`; 0 = без лимита |
| `onboard_failsafe_action` | string | `hold` | Действие при критических откатах: `hold` \| `land` \| `rtl` |
| `land_from_hold` | bool | false | Дополнительно разрешить `land` из AUTO.LOITER/POSCTL/ALTCTL |
| `zero_heading_at_start` | bool | = `simulator` | Обнулять **отображаемый** yaw по первому сэмплу одометрии (см. раздел про heading) |

Параметры возбудителя (`excite_*`, `allow_rate_loop_excitation`) — в разделе про
`run_excitation` ниже. Launch-файл может объявлять дополнительные аргументы
(например `setpoint_rate`, `nav_from_sp`); нода использует только те, что
объявлены в коде выше.

---

## Сервисы

Все сервисы доступны в namespace ноды (по умолчанию без префикса, т.е. `/navigate`, `/land` и т.д.).

Общее для сервисов, задающих цель (`navigate`, `set_position`, `set_velocity`):
цели проходят **единую валидацию** — NaN/Inf отклоняются с понятным `message`
(для `navigate` частично-NaN цель по-прежнему легальна: NaN = «оставить текущую
координату»), скорость ограничивается `max_speed` (если задан), при
`geofence_enable:=true` цель вне геозоны отклоняется.

---

### 1. `navigate` — полёт к точке (высокоуровневый)

**Тип:** `offboard_interfaces/srv/Navigate`

Плавный полёт к целевой позиции с заданной скоростью и системой координат. Курс (`yaw`) устанавливается сразу в целевое значение при старте команды. Подходит для взлёта, перелёта по прямой и простых миссий.

| Поле | Тип | Описание |
|------|-----|----------|
| `x`, `y`, `z` | float32 | Целевая позиция (м) |
| `yaw` | float32 | Целевой курс (рад) |
| `speed` | float32 | Скорость по траектории (м/с) |
| `frame_id` | string | СК: `map`, `body`, `terrain`, `aruco_map` и др. |
| `auto_arm` | bool | Перевести в OFFBOARD и заармить перед полётом |

**Ответ:** `success`, `message`.

**Примеры:**

```bash
# Взлёт на 1.5 м (относительно текущей позиции, body)
ros2 service call /navigate offboard_interfaces/srv/Navigate "{x: 0.0, y: 0.0, z: 1.5, yaw: 0.0, speed: 1.0, frame_id: 'body', auto_arm: true}"

# Полёт в точку в мировой СК (map)
ros2 service call /navigate offboard_interfaces/srv/Navigate "{x: 1.0, y: 0.0, z: 1.0, yaw: 0.0, speed: 0.5, frame_id: 'map', auto_arm: false}"
```

---

### 2. `land` — посадка

**Тип:** `std_srvs/srv/Trigger`

Команда на посадку (PX4 NAV_LAND). При `land_only_in_offboard:=true` принимается
только в режиме OFFBOARD; параметр `land_from_hold:=true` дополнительно разрешает
посадку из hold-режимов (`AUTO.LOITER`/`POSCTL`/`ALTCTL`).

```bash
ros2 service call /land std_srvs/srv/Trigger "{}"
```

---

### 3. `get_telemetry` — телеметрия

**Тип:** `offboard_interfaces/srv/GetTelemetry`

Текущее состояние: позиция (x, y, z в ENU), курс (yaw в NED, 0 = север), скорость, режим, арм, батарея, глобальные координаты.

| Запрос | Ответ (основное) |
|--------|-------------------|
| `frame_id` (опционально) | `connected`, `armed`, `mode`, `x`, `y`, `z`, `yaw`, `vx`, `vy`, `vz`, `roll_rate`, `pitch_rate`, `yaw_rate`, `lat`, `lon`, `alt`, `voltage`, `frame_id` |

```bash
ros2 service call /get_telemetry offboard_interfaces/srv/GetTelemetry "{frame_id: 'map'}"
```

Примечания:
- `roll_rate`/`pitch_rate`/`yaw_rate` заполняются из гироскопа
  (`vehicle_angular_velocity`) — на реальном дроне только с прошивкой,
  публикующей этот топик по DDS; иначе нули.
- `yaw` в map-фрейме учитывает `zero_heading_at_start` (см. раздел про heading).
- Та же телеметрия доступна потоком: топик `~/telemetry` и action
  `receive_telemetry` (см. ниже) — все три пути используют один код.

---

### 4. `set_altitude` — смена целевой высоты

**Тип:** `offboard_interfaces/srv/SetAltitude`

Меняет только целевую высоту в текущей миссии (navigate или set_position). Остальная логика полёта не прерывается.

| Поле | Тип | Описание |
|------|-----|----------|
| `z` | float32 | Высота (м) |
| `frame_id` | string | СК высоты: `map`, `terrain`, `aruco_map`, `body` и др. |

```bash
# 2 м над землёй (terrain)
ros2 service call /set_altitude offboard_interfaces/srv/SetAltitude "{z: 2.0, frame_id: 'terrain'}"

# 1 м в мировой СК
ros2 service call /set_altitude offboard_interfaces/srv/SetAltitude "{z: 1.0, frame_id: 'map'}"
```

---

### 5. `set_yaw` — смена целевого курса

**Тип:** `offboard_interfaces/srv/SetYaw`

Меняет целевой курс в текущей миссии (navigate/set_position), не отменяя её.

| Поле | Тип | Описание |
|------|-----|----------|
| `yaw` | float32 | Курс (рад). **NaN** — сбросить override и остановить вращение по yaw_rate |
| `frame_id` | string | СК курса: `body`, `map`, `aruco_map` и др. |

```bash
# Поворот на 90° по часовой относительно корпуса
ros2 service call /set_yaw offboard_interfaces/srv/SetYaw "{yaw: -1.5708, frame_id: 'body'}"

# Остановить вращение по рысканью (после set_yaw_rate)
ros2 service call /set_yaw offboard_interfaces/srv/SetYaw "{yaw: nan, frame_id: 'body'}"
```

---

### 6. `set_yaw_rate` — угловая скорость по рысканью

**Тип:** `offboard_interfaces/srv/SetYawRate`

Задаёт целевую угловую скорость по рысканью (рад/с), не отменяя текущую миссию. Положительное значение — против часовой (вид сверху).

| Поле | Тип | Описание |
|------|-----|----------|
| `yaw_rate` | float32 | Угловая скорость (рад/с) |

```bash
# Вращение 0.5 рад/с против часовой
ros2 service call /set_yaw_rate offboard_interfaces/srv/SetYawRate "{yaw_rate: 0.5}"
```

Остановка — вызовом `set_yaw` с `yaw: nan`.

---

### 7. `set_position` — целевая позиция и курс (поток целей)

**Тип:** `offboard_interfaces/srv/SetPosition`

Задаёт целевую точку и курс; нода переходит в режим удержания/полёта к этой точке. Удобно для сложных траекторий (дуги, круги) при частом обновлении цели.

| Поле | Тип | Описание |
|------|-----|----------|
| `x`, `y`, `z` | float32 | Позиция (м); **NaN/Inf отклоняются** с описанием в `message` |
| `yaw` | float32 | Курс (рад) |
| `frame_id` | string | СК (по умолчанию `map`) |
| `auto_arm` | bool | Перевести в OFFBOARD и заармить |

```bash
# Зависание на месте (body: 0,0,0)
ros2 service call /set_position offboard_interfaces/srv/SetPosition "{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, frame_id: 'body', auto_arm: false}"

# Точка на 3 м выше текущей
ros2 service call /set_position offboard_interfaces/srv/SetPosition "{x: 0.0, y: 0.0, z: 3.0, yaw: 0.0, frame_id: 'body', auto_arm: false}"

# 1 м вперёд по корпусу
ros2 service call /set_position offboard_interfaces/srv/SetPosition "{x: 1.0, y: 0.0, z: 0.0, yaw: 0.0, frame_id: 'body', auto_arm: false}"
```

---

### 8. `set_velocity` — целевая скорость и курс

**Тип:** `offboard_interfaces/srv/SetVelocity`

Режим управления по скорости (vx, vy, vz) и курсу. `frame_id` задаёт СК, в которой заданы компоненты скорости и курс.

| Поле | Тип | Описание |
|------|-----|----------|
| `vx`, `vy`, `vz` | float32 | Скорость (м/с); **NaN/Inf отклоняются**; \|v\| ≤ `max_speed` (если задан) |
| `yaw` | float32 | Курс (рад) |
| `frame_id` | string | СК (по умолчанию `map`) |
| `auto_arm` | bool | OFFBOARD и арминг при необходимости |

```bash
# Полёт вперёд по корпусу 1 м/с
ros2 service call /set_velocity offboard_interfaces/srv/SetVelocity "{vx: 1.0, vy: 0.0, vz: 0.0, yaw: 0.0, frame_id: 'body', auto_arm: false}"
```

---

### 9. `set_attitude` — тангаж, крен, курс, газ

**Тип:** `offboard_interfaces/srv/SetAttitude`

Низкоуровневый режим по углам и тяге (аналог STABILIZED).

| Поле | Тип | Описание |
|------|-----|----------|
| `roll`, `pitch`, `yaw` | float32 | Углы (рад) |
| `thrust` | float32 | Газ 0…1 |
| `frame_id` | string | СК для yaw (по умолчанию `map`) |
| `auto_arm` | bool | OFFBOARD и арминг при необходимости |

---

### 10. `set_rates` — угловые скорости и газ

**Тип:** `offboard_interfaces/srv/SetRates`

Режим по угловым скоростям и тяге (аналог ACRO).

| Поле | Тип | Описание |
|------|-----|----------|
| `roll_rate`, `pitch_rate`, `yaw_rate` | float32 | Угловые скорости (рад/с) |
| `thrust` | float32 | Газ 0…1 |
| `auto_arm` | bool | OFFBOARD и арминг при необходимости |

Положительное направление: yaw — против часовой (вид сверху), pitch — нос вверх, roll — влево.

---

### 11. `flip` — флип по выбранной оси

**Тип:** `offboard_interfaces/srv/Flip`

Выполняет **флип** (почти полный оборот по одной оси) в три фазы: подъём по скорости, вращение по угловой скорости, возврат к начальной позиции.

#### Как это работает

1. **Запоминание начальной позиции**
   В момент вызова сервиса сохраняются текущие координаты и курс в NED: `(x, y, z, yaw)` из одометрии (VehicleOdometry).

2. **Фаза подъёма (CLIMB)**
   В течение `climb_duration` секунд нода подаёт в PX4 **TrajectorySetpoint** в режиме **velocity**:
   - `position` = NaN (позиция не задаётся);
   - `velocity` = (0, 0, **−vz**) в NED (т.е. вверх в локальной СК при vz > 0);
   - `yaw` = начальный курс.
   То есть дрон просто летит вверх с заданной скоростью по Z, без изменения курса.

3. **Фаза вращения (ROTATE)**
   После окончания подъёма нода переключается на **VehicleRatesSetpoint** (режим body_rate в Offboard):
   - по выбранной оси (`axis`) подаётся угловая скорость **rate** (рад/с);
   - по остальным осям — 0;
   - `thrust_body[2]` = **−thrust** (тяга по оси тела для удержания высоты).
   Угол поворота считается по времени: на каждом тике таймера (частота `control_rate_`, обычно 30 Гц) к накопленному углу прибавляется `|rate| * dt`. Как только накопленный угол ≥ `target_angle`, фаза завершается.

4. **Возврат к начальной позиции (POSITION_HOLD)**
   Чтобы дрон не продолжал вращаться, нода переключается обратно на **TrajectorySetpoint** в режиме **position** и один раз публикует целевую позицию, равную сохранённой начальной: `(x, y, z, yaw)` в NED. После этого тип сетапойнта сбрасывается в `NONE`, и дальше в цикле публикуется уже обычный idle setpoint (удержание последней позиции).

#### Что именно подаётся в PX4

| Фаза | Режим OffboardControlMode | Сообщение | Содержимое |
|------|---------------------------|-----------|------------|
| CLIMB | `velocity=true` | TrajectorySetpoint | position=NaN; velocity=(0,0,−vz) NED; yaw=начальный |
| ROTATE | `body_rate=true` | VehicleRatesSetpoint | одна ось = rate, остальные 0; thrust_body[2]=−thrust |
| POSITION_HOLD | `position=true` | TrajectorySetpoint | position=начальная (x,y,z), yaw=начальный |

Таким образом, для флипа по очереди используются те же низкоуровневые режимы, что и для сервисов `set_velocity`, `set_rates` и `set_position`, но в автоматической последовательности без дополнительных вызовов.

#### Параметры запроса

| Поле | Тип | Описание |
|------|-----|----------|
| `axis` | string | Ось вращения: `"roll"`, `"pitch"` или `"yaw"` |
| `vz` | float32 | Скорость вверх по локальной Z, м/с (например 1.0) |
| `climb_duration` | float32 | Длительность подъёма, сек |
| `rate` | float32 | Угловая скорость по выбранной оси, рад/с (знак задаёт направление) |
| `target_angle` | float32 | Целевой угол поворота, рад (например 6.28 ≈ 2π; можно 6.0, чтобы остановить чуть раньше полного оборота) |
| `thrust` | float32 | Тяга 0…1 во время фазы ROTATE |
| `auto_arm` | bool | При true — перевести в OFFBOARD и заармить перед стартом флипа |

**Ответ:** `success`, `message`.

**Пример вызова сервиса flip на базовой комплектации коптера (кувырок вокруг оси Roll с вращением на полный оборот):**
Перед отправкой команды на кувырок убедитесь, что запас по высоте составляет не менее 2 метров!!!
```bash
ros2 service call /flip offboard_interfaces/srv/Flip "{axis: 'roll', vz: 2.0, climb_duration: 0.5, rate: 16.0, target_angle: 6.00, thrust: 0.1, auto_arm: false}"
```

Перед вызовом дрон должен быть в OFFBOARD и armed (или использовать `auto_arm: true`). Рекомендуется сначала взлететь и зависнуть, затем вызывать flip.

---

### 12. `emergency_stop` — универсальный аварийный стоп

**Тип:** `offboard_interfaces/srv/EmergencyStop`

Немедленно останавливает **любой** активный манёвр (`navigate`/`set_*`/`flip`/
`navigate_to`/возбуждение) — работает даже пока другой сервис ждёт арминга.

| Поле | Тип | Описание |
|------|-----|----------|
| `land` | bool | `false` — удержание текущей позиции (hold); `true` — остановить манёвр и сесть |

```bash
# парковка в hold
ros2 service call /emergency_stop offboard_interfaces/srv/EmergencyStop "{land: false}"
# аварийная посадка
ros2 service call /emergency_stop offboard_interfaces/srv/EmergencyStop "{land: true}"
```

---

### 13. `return_to_launch` — возврат на точку старта (RTL)

**Тип:** `offboard_interfaces/srv/ReturnToLaunch`

Корректно останавливает активный манёвр и шлёт
`VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` — дрон летит на home-позицию и садится
(PX4 `AUTO.RTL`). Запрос без полей; ответ `success`, `message`.

```bash
ros2 service call /return_to_launch offboard_interfaces/srv/ReturnToLaunch "{}"
```

---

## Действия (actions)

### `navigate_to` — полёт к точке с ожиданием прибытия

**Тип:** `offboard_interfaces/action/NavigateTo`

Тот же запуск, что у сервиса `navigate` (общий код), но клиент получает
feedback до прибытия и result по факту прилёта — «navigate_wait» на уровне ноды.

| Goal | Feedback | Result |
|------|----------|--------|
| `x`, `y`, `z`, `yaw`, `speed`, `frame_id`, `auto_arm` — как в `Navigate.srv`; плюс `timeout` (с) и `tolerance` (м) | `distance_remaining` (м), `eta` (с), `phase` (`arming`\|`moving`\|`arrived`) | `success`, `message`, `final_distance_error` (м) |

**Пути завершения** (action гарантированно не виснет, если цель физически
недостижима):

1. **Прибытие** — таймер зафиксировал `isEnd` (штатный допуск 0.08 м + yaw) →
   `success=true`.
2. **`tolerance` > 0** — ослабленный радиус завершения: result `success=true`
   отдаётся, как только `distance_remaining ≤ tolerance`; сам полёт продолжает
   сходиться к точной цели (дрон остаётся висеть у неё).
3. **`timeout`** — дрон не долетел за отведённое время → парковка в hold на
   текущей позе, result `success=false` («Timeout ...»). Дефолт `timeout=0` =
   **авто**: `3 × (дистанция/скорость) + 10 с`; `timeout<0` = без лимита
   (ждать до cancel).
4. **Cancel** — мягкий стоп: удержание текущей позы, `success=false`.
5. **Вытеснение** другой командой (`navigate`/`set_*`/`emergency_stop`) —
   result `success=false` («superseded»), чужой манёвр не трогается.

Одновременно активен один навигатор: при `busy_`/активном манёвре goal отклоняется.

```bash
ros2 action send_goal -f /navigate_to offboard_interfaces/action/NavigateTo \
  "{x: 1.5, y: 1.0, z: 2.5, yaw: 0.0, speed: 0.6, frame_id: 'map', auto_arm: false, timeout: 30.0, tolerance: 0.2}"
```

### `receive_telemetry` — поток телеметрии с заданной частотой

**Тип:** `offboard_interfaces/action/ReceiveTelemetry`

Стрим телеметрии в feedback с частотой клиента и управляемым жизненным циклом.

| Goal | Feedback | Result |
|------|----------|--------|
| `rate_hz` (0 < f ≤ 200), `frame_id`, `duration` (с, 0 = до cancel) | `telemetry` (`msg/Telemetry`) | `success`, `message`, `samples_sent` |

```bash
ros2 action send_goal -f /receive_telemetry offboard_interfaces/action/ReceiveTelemetry \
  "{rate_hz: 10.0, frame_id: 'map', duration: 5.0}"
```

Action `run_excitation` описан в отдельном разделе ниже.

---

## Топики ноды

| Топик | Тип | Частота | Описание |
|-------|-----|---------|----------|
| `~/telemetry` | `offboard_interfaces/msg/Telemetry` | `telemetry_rate_hz` (0 = выкл) | Та же телеметрия, что `get_telemetry`, потоком; частота меняется на лету: `ros2 param set /offboard_control telemetry_rate_hz 50.0` |
| `~/status` | `offboard_interfaces/msg/OffboardStatus` | ~2 Гц | Диагностика ноды: `setpoint_type`, `busy`, `armed`, `nav_mode`, `offboard_active`, последняя цель, `active_maneuver`, `last_abort_reason`, возраст одометрии, батарея |
| `~/excitation/reference` | `offboard_interfaces/msg/ExcitationSample` | на окне возбуждения | Референс r(t) для анализатора (см. раздел возбудителя) |

```bash
ros2 topic echo /offboard_control/status --once
```

---

## Системы координат (frame_id)

| frame_id | Описание |
|----------|----------|
| `map` | Локальная мировая СК (задаётся `map_frame_id`, часто map или odom). |
| `body` | Связанная с корпусом: ось X — вперёд, Y — влево, Z — вверх. |
| `terrain` | Высота относительно «пола» (например, под дроном). |
| `aruco_map` | Маркерная СК (общая для карты маркеров). |
| `aruco_<N>` | Конкретный маркер (N — число). |

Для `navigate` с `frame_id: 'body'` целевая точка и курс задаются относительно текущей позиции и курса; при `yaw: 0` курс сохраняется.

---

## Примеры вызовов

**Взлёт и полёт в точку по map:**

```bash
source install/setup.bash
ros2 service call /navigate offboard_interfaces/srv/Navigate "{x: 1.0, y: 0.0, z: 1.0, yaw: 0.0, speed: 0.5, frame_id: 'map', auto_arm: true}"
```

**Взлёт по body (0, 0, высота), затем посадка:**

```bash
ros2 service call /navigate offboard_interfaces/srv/Navigate "{x: 0.0, y: 0.0, z: 0.5, yaw: 0.0, speed: 1.0, frame_id: 'body', auto_arm: true}"
# ... после завершения ...
ros2 service call /land std_srvs/srv/Trigger "{}"
```

**Зависание на месте (set_position):**

```bash
ros2 service call /set_position offboard_interfaces/srv/SetPosition "{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0, frame_id: 'body', auto_arm: false}"
```

---

## Тестовые скрипты

В каталоге `examples/`:

- **test_fly_cube.py** — взлёт и полёт по траектории «куб» (относительные смещения в body).
- **test_all_services.py** — последовательная проверка сервисов: взлёт, set_altitude, set_yaw, set_yaw_rate, set_position, set_velocity, set_attitude, set_rates, посадка.
- **test_excitation.py** — SITL-проверка возбудителя: взлёт, малый roll-doublet (attitude, Pattern A), печать feedback, проверка `result.success` и возврата к старту, посадка.
- **test_offboard_hardening.py** — SITL-smoke блока упрочнения: взлёт (auto_arm),
  `get_telemetry`, action `navigate_to` (feedback до прибытия + сценарий
  таймаута недостижимой цели), стрим `receive_telemetry`, `emergency_stop`
  (hold), посадка; без регрессий существующих сервисов.

Юнит-тесты C++ (без ROS-графа): `colcon test --packages-select offboard_control`
(gtest `test_offboard_logic` — трансформации, SpeedController, валидатор,
маппинги; `test_excitation_signals` — библиотека сигналов). Для ловли гонок
(промпты 91–92) доступна сборка с ThreadSanitizer:
`colcon build --packages-select offboard_control --cmake-args -DCMAKE_CXX_FLAGS=-fsanitize=thread -DCMAKE_EXE_LINKER_FLAGS=-fsanitize=thread`.

Запуск (из workspace, после `source install/setup.bash`):

```bash
python3 src/sverk_drone/offboard/offboard_control/examples/test_fly_cube.py
python3 src/sverk_drone/offboard/offboard_control/examples/test_all_services.py
```

Перед запуском должны быть запущены нода `offboard_control` и (для SITL) симуляция и MicroXRCEAgent.

---

## Типичный сценарий (SITL)

1. Запуск симуляции и агента (например PX4 SITL + Gazebo, MicroXRCEAgent на порту 8888).
2. Запуск ноды:
   ```bash
   ros2 launch offboard_control offboard_control.launch.py
   ```
   или с `simulator:=true` при необходимости.
3. Взлёт и миссия через сервисы (см. примеры выше).
4. Посадка: `ros2 service call /land std_srvs/srv/Trigger "{}"`.

---

## Возбудитель (system-ID excitation) — action `run_excitation`

> **НЕ read-only.** Нода активно трясёт летящий дрон управляемыми возмущениями
> (step/doublet/3-2-1-1/chirp/multisine/prbs) по одной оси для идентификации
> rate/attitude-контуров. Порядок ввода в эксплуатацию строго:
> **SITL → стенд на привязи (или со снятыми пропеллерами) → помещение с геозоной.**
> Значения по умолчанию консервативные; повышать амплитуду только при плохой
> когерентности.

### Интерфейс

**Action `run_excitation`** (`offboard_interfaces/action/RunExcitation`):

| Goal | Result | Feedback |
|------|--------|----------|
| `axis` (roll/pitch/yaw), `signal` (step/doublet/3211/chirp/multisine/prbs), `loop` (0=attitude, 1=rate), `amplitude` (рад или рад/с), `duration`, `f_min`/`f_max` (Гц), `base_period`, `seed`, `taper`, `auto_arm`, `hold_alt`, `max_tilt_deg`, `geofence_radius`, `confirm` (фраза-подтверждение) | `success`, `message`, `samples_injected`, `achieved_rate_hz`, `max_tilt_seen_deg`, `max_drift_m`, `abort_reason`, `run_tag` | `elapsed`, `progress`, `phase`, `current_ref`, `current_rate`, `current_tilt_deg`, `current_drift_m` |

**Сервис `stop_excitation`** (`offboard_interfaces/srv/StopExcitation`):
`emergency=true` — немедленный revert; `false` — плавный settle-out → revert.

**Топик `~/excitation/reference`** (`msg/ExcitationSample`) — референс r(t)
для анализатора `pid_tuning_assistant` (ось, сигнал, фаза, run_tag).

### Параметры возбудителя

| Параметр | По умолчанию | Описание |
|----------|--------------|----------|
| `excite_stream_hz` | 200.0 | Частота потока сетпоинтов на окне возбуждения (Гц) |
| `excite_settle_in_s` / `excite_settle_out_s` | 1.5 / 1.5 | Стабилизация hover до/после инъекции (с) |
| `excite_max_inject_s` | 20.0 | Dead-man инъекции на ось (с) |
| `excite_max_session_s` | 90.0 | Dead-man всей сессии (с) |
| `allow_rate_loop_excitation` | false | Разрешить Pattern B (rate-loop, advanced) |
| `excite_geofence_radius` | 2.0 | Радиус геозоны возбуждения от точки старта (м) |
| `excite_agl_floor` / `excite_agl_ceil` | 1.0 / 4.0 | Коридор высоты AGL (м) |
| `excite_kp_pos` | 0.04 | P удержания горизонтали (рад наклона на м ошибки) |
| `excite_kp_alt` | 0.05 | P удержания высоты (тяга на м ошибки) |
| `excite_max_baseline_tilt` | 0.087 | Кап наклона hover-контура (рад, ~5°) |
| `excite_hover_thrust` | 0.5 | Базовая hover-тяга [0..1] |
| `excite_stick_abort_thresh` | 0.05 | Порог стика для RC-abort возбуждения |

### Паттерны

- **Pattern A (attitude, по умолчанию, `loop=0`)** — нода сама держит hover
  (медленный P по горизонтали + P по высоте → `q_hover`), накладывает малое
  угловое возмущение `δq` по оси: `q_d = q_hover ⊗ δq`. Горизонталь удерживается.
- **Pattern B (rate, advanced, `loop=1`)** — публикует `VehicleRatesSetpoint`
  `ω_sp = ω_stab + δω`, где `ω_stab` — attitude-P стабилизатор. **Позиция по
  горизонтали НЕ удерживается** (контроллер позиции PX4 обойдён) → возможен дрейф,
  ловится геозоной. Отключён по умолчанию; включается параметром
  `allow_rate_loop_excitation=true`. Использовать только indoor с узкой геозоной.

### Предохранители (§4)

Жёсткие капы (rate ≤ 0.5 рад/с, угол ≤ 10°, полный наклон ≤ `max_tilt_deg` ≤ 25°),
окно Tukey + washout (zero-mean), таймауты (инъекция ≤ `excite_max_inject_s`,
сессия ≤ `excite_max_session_s`), геозона по дрейфу и AGL-коридор, RC-abort
(kill-switch + стик > `excite_stick_abort_thresh`), при любом нарушении —
немедленный возврат в Position/Hold на захваченную стартовую позу. Обязательная
фраза `confirm = "I_UNDERSTAND_WHAT_I_AM_DOING"` (иначе goal отклоняется).

### Требуемые параметры PX4 (для RC-abort и failsafe)

- `COM_RC_OVERRIDE=3` — иначе стик **не** прерывает offboard (дефолт 1);
- `COM_OBL_RC_ACT ∈ {0 (Position), 5 (Hold)}` — действие при потере offboard;
- `COM_OF_LOSS_T=1` — таймаут потери offboard, с.
- Гироскоп `vehicle_angular_velocity` (feedback `current_rate`, запись отклика)
  публикуется только с прошивкой `fmu/matek_h743-slim_pid_on.px4`.

### Аварийный стоп

Сервис `stop_excitation` (`StopExcitation.srv`): `emergency=true` → немедленный
REVERT (минуя settle-out); `emergency=false` → плавный settle-out → revert.
Штатная отмена — через cancel самого action.

### SITL-проверка (обязательна перед стендом)

```bash
# терминал 1: PX4 SITL + Gazebo + MicroXRCEAgent
# терминал 2:
ros2 run offboard_control offboard_control
# терминал 3:
python3 src/sverk_drone/offboard/offboard_control/examples/test_excitation.py
```

Ожидание: дрон взлетает, делает малый doublet по roll и возвращается к старту;
`result.success=true`, `max_drift_m` мал, `abort_reason` пуст.

Идентификацию/синтез гейнов и проверку запасов делает анализатор
(`pid_tuning_assistant`, контракт данных — промпт 74); возбудитель гейны **не пишет**.

---

## RTL и failsafe-политика ноды

- Сервис **`return_to_launch`** (`ReturnToLaunch.srv`): корректно останавливает
  активный манёвр (включая возбуждение) и шлёт
  `VEHICLE_CMD_NAV_RETURN_TO_LAUNCH` — дрон летит на точку старта и садится
  (PX4 `AUTO.RTL`).
- Параметр **`onboard_failsafe_action`** (`hold` | `land` | `rtl`, дефолт
  `hold`) — действие ноды при критических откатах: потеря RC
  (`manual_control_timeout`), устаревшая одометрия (`enforce_freshness`),
  вылет за геозону (`geofence_enable`). `hold` = удержание текущей позы.
  Перехват пилотом (kill-switch/стик) параметром НЕ переопределяется — там
  нода всегда отдаёт поток PX4 (`COM_RC_OVERRIDE`/`COM_OBL_RC_ACT`).
- Параметр **`land_from_hold`** (дефолт `false`): аддитивно разрешает `land`
  из hold-режимов (`AUTO.LOITER`/`POSCTL`/`ALTCTL`), не ослабляя дефолтную
  защиту `land_only_in_offboard`.
- PX4-сторона failsafe (`COM_OBL_RC_ACT`, `COM_OF_LOSS_T`, `COM_RC_OVERRIDE`)
  нодой не дублируется — см. раздел про RC-перехват.

---

## Фрейм heading: управление vs отображение

- **Управление** всегда использует сырой NED-heading от PX4
  (`current_local_position_.heading`) — никакие оффсеты к нему не применяются,
  иначе поехали бы команды (`navigate`/`set_yaw` и т.д.).
- **Отображаемый yaw** (`get_telemetry` / топик `~/telemetry` / debug-логи) при
  `zero_heading_at_start=true` обнуляется по курсу первого сэмпла одометрии —
  только для вывода в `map`-фрейме; фрейм-относительные yaw (`aruco_*`, `body`)
  не трогаются.
- Параметр `zero_heading_at_start`: дефолт повторяет историческое поведение —
  `true` в симуляторе (`simulator:=true`), `false` на реальном дроне. Теперь
  поведение одинаково реализовано в обоих режимах и явно переключается.

---

## RC-перехват (kill-switch / стик) — для всех манёвров

RC-перехват работает в каждом тике управляющего таймера для ЛЮБОГО активного
сетпоинта (`navigate`, `set_position`, `set_velocity`, `set_attitude`,
`set_rates`, `flip`), а не только в возбудителе:

- **kill-switch** — бит 12 `manual_control_setpoint.buttons` (параметр
  `check_kill_switch`, дефолт `true`) → немедленная остановка потока сетпоинтов
  (`stopOffboardHeartbeat`) и снятие `busy`;
- **потеря RC** — параметр `manual_control_timeout` (сек, дефолт `0` =
  выключено): при устаревшем `manual_control_setpoint` манёвр останавливается;
- **отклонение стика** — параметр `rc_stick_abort_thresh` (дефолт `0.05`):
  |roll|/|pitch|/|yaw| выше порога → управление отдаётся пилоту.

В симуляторе перехват по умолчанию выключен; `enforce_rc_in_sim:=true` включает
его в SITL (например, при тесте с джойстиком).

**Требуемые параметры PX4** (иначе PX4 сам не отдаст пилоту управление):
`COM_RC_OVERRIDE=3`, `COM_OBL_RC_ACT ∈ {0 (Position), 5 (Hold)}`,
`COM_OF_LOSS_T=1`. После остановки потока нода полагается на PX4-failsafe
(Position/Hold) — это то же поведение, что и у RC-abort возбудителя.

---

*Документация актуальна для пакета `offboard_control` в составе sverk_ws.*
