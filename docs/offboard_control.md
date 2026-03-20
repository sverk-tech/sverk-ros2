# Offboard Control

ROS 2-нода для управления мультикоптером в режиме **Offboard** через PX4 (uXRCE-DDS). Поддерживает навигацию по целевым точкам, полёт в разных системах координат, телеметрию и набор сервисов для поточечного и траекторного управления.

---

## Содержание

- [Требования](#требования)
- [Сборка](#сборка)
- [Запуск](#запуск)
- [Параметры ноды и launch](#параметры-ноды-и-launch)
- [Сервисы](#сервисы)
- [Системы координат (frame_id)](#системы-координат-frame_id)
- [Примеры вызовов](#примеры-вызовов)
- [Тестовые скрипты](#тестовые-скрипты)

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
| `manual_control_timeout` | double | 0.0 | Таймаут manual control (с); 0 = отключено |
| `simulator` | bool | false | Режим симулятора (другие топики PX4) |

Launch-файл может объявлять дополнительные аргументы (например `setpoint_rate`, `nav_from_sp`); нода использует только те, что объявлены в коде выше.

---

## Сервисы

Все сервисы доступны в namespace ноды (по умолчанию без префикса, т.е. `/navigate`, `/land` и т.д.).

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

Команда на посадку (PX4 NAV_LAND). При `land_only_in_offboard:=true` принимается только в режиме OFFBOARD.

```bash
ros2 service call /land std_srvs/srv/Trigger "{}"
```

---

### 3. `get_telemetry` — телеметрия

**Тип:** `offboard_interfaces/srv/GetTelemetry`

Текущее состояние: позиция (x, y, z в ENU), курс (yaw в NED, 0 = север), скорость, режим, арм, батарея, глобальные координаты.

| Запрос | Ответ (основное) |
|--------|-------------------|
| `frame_id` (опционально) | `connected`, `armed`, `mode`, `x`, `y`, `z`, `yaw`, `vx`, `vy`, `vz`, `lat`, `lon`, `alt`, `voltage`, `frame_id` |

```bash
ros2 service call /get_telemetry offboard_interfaces/srv/GetTelemetry "{frame_id: 'map'}"
```

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
| `x`, `y`, `z` | float32 | Позиция (м) |
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
| `vx`, `vy`, `vz` | float32 | Скорость (м/с) |
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

*Документация актуальна для пакета `offboard_control` в составе sverk_ws.*
