# SVERK SITL — симулятор дрона: запуск и работа

Симулятор дрона на **PX4 v1.15.4 + Gazebo Harmonic + ROS 2 Humble**. Внутри —
тот же полётный стек, что на реальном дроне: навигация по ArUco-меткам,
дальномеры, камера, взлёт/посадка по сервису. Ставится одной командой `pull` и
запускается через `docker compose`.

---

## 1. Что нужно на ноутбуке

- **Docker** (Docker Desktop на Windows/macOS, либо Docker Engine на Linux).
- ~10 ГБ места под образ, 8+ ГБ RAM.
- Ничего больше: **noVNC-режим работает на любом железе** (картинка считается
  на CPU и открывается в браузере). GPU нужен только для «красивого» WSLg-режима.

Проверьте, что Docker работает:

```bash
docker run --rm hello-world
```

---

## 2. Установка образа

Из папки `compose/` скопируйте `.env.example` в `.env` (там прописаны имена
образов) и скачайте их:

**Windows (PowerShell):**
```powershell
cd compose
Copy-Item .env.example .env
.\sim.ps1 pull
```

**Linux / WSL / macOS:**
```bash
cd compose
cp .env.example .env
./sim.sh pull
```

`pull` скачивает образ выбранного режима (по умолчанию noVNC). Разово и долго —
образ большой. Для другого режима укажите его: `./sim.sh pull wslg-nvidia`.

> Если `sim.ps1` не запускается из-за политики PowerShell — выполните один раз:
> `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass` и повторите.

---

## 3. Быстрый старт — noVNC (рекомендуется всем)

noVNC не требует GPU и одинаково работает на Windows и Linux. Картинка Gazebo
открывается в обычном браузере.

**1) Поднять контейнер** (из папки `compose/`):

```bash
./sim.sh up novnc          # Windows: .\sim.ps1 up novnc
```

**2) Открыть рабочий стол** в браузере: <http://localhost:6080/vnc.html> → «Connect».

**3) Войти в контейнер** (в отдельном терминале):

```bash
./sim.sh shell             # Windows: .\sim.ps1 shell
# то же самое: docker exec -it sverk_sitl bash
```

**4) Запустить симулятор** (внутри контейнера). Для noVNC — флаг `vnc_gui:=true`:

```bash
ros2 launch launch_system full_system_sitl.launch.py \
  vnc_gui:=true obrik_config:=base gz_world:=obrik_aruco
```

Через несколько секунд в окне браузера (:6080) появится поле с ArUco-метками и
дрон. Дальше — раздел [«Полёт»](#7-полёт-взлёт-посадка).

**5) Остановить:**

```bash
./sim.sh down novnc        # Windows: .\sim.ps1 down novnc
```

---

## 4. Другие режимы отображения

Симулятор один и тот же — меняется только способ показа картинки. Выберите строку
под своё железо. Флаг в `ros2 launch` **обязан совпадать** с режимом контейнера.

| Режим | Когда | Поднять (из `compose/`) | Флаг launch | Где картинка |
|---|---|---|---|---|
| **noVNC** | любой ноут, надёжнее всего | `./sim.sh up novnc` | `vnc_gui:=true` | браузер :6080 |
| **WSLg + NVIDIA** | Windows + видеокарта NVIDIA | `./sim.sh up wslg-nvidia` | `wslg_gpu_gui:=true` | окно на рабочем столе |
| **WSLg + Intel Arc** | Windows + Intel Arc | `./sim.sh up wslg-intel` | `wslg_gpu_gui:=true` | окно (часто падает → берите noVNC) |
| **Linux X11** | нативный Linux-десктоп | `./sim.sh up linux-host` | `host_x11_gui:=true` | окно на рабочем столе |
| **headless** | без картинки, только логика | `./sim.sh up headless` | `headless:=true` | нет окна |

**Web-IDE (опция).** Аддон `ide` добавляет к любому режиму, **кроме noVNC**,
VS Code и терминал прямо в браузере — можно работать без `docker exec`/ssh:

```bash
./sim.sh up headless ide       # Windows: .\sim.ps1 up headless ide
# code-server → http://localhost:5467 ,  терминал → http://localhost:8575
```

Терминал :8575 открывается уже внутри контейнера с загруженным ROS-окружением —
запускайте `ros2 launch ...` прямо там. С noVNC аддон не нужен: в его рабочем
столе :6080 терминал открывается штатно.

**MCP-сервер (опция).** Аддон `mcp` поднимает вторым контейнером ROS 2 MCP
HTTP-сервер на `:8765` — набор инструментов над живым ROS-графом симулятора
(список нод/топиков, echo, вызов сервисов, публикация, чтение параметров).
Сочетается с **любым** режимом, включая noVNC:

```bash
./sim.sh up novnc mcp          # Windows: .\sim.ps1 up novnc mcp
# несколько аддонов сразу:  ./sim.sh up headless ide+mcp
```

Проверка и использование с хоста:

```bash
curl -s http://localhost:8765/healthz     # {"ok": true, "node_ready": true}
curl -s http://localhost:8765/tools       # список инструментов
curl -s http://localhost:8765/policy      # что сейчас разрешено писать
```

Пишущие инструменты (публикация, вызов сервисов, установка параметров)
ограничены политикой `compose/mcp_policy.yaml` — по умолчанию режим `guarded`
(разрешено только то, что подходит под allow-паттерны). Инструменты чтения
доступны всегда. К `:8765` можно подключить своего LLM-агента.

### WSLg (Windows, GPU-окно)

> Compose с WSLg-оверлеями запускайте **строго изнутри WSL-дистрибутива**
> (`wsl -d Ubuntu-24.04`), а не из PowerShell: бинды `/mnt/wslg`, `/usr/lib/wsl`,
> `/dev/dxg` есть только внутри дистрибутива. `docker exec` в уже поднятый
> контейнер работает откуда угодно.

```bash
# из Ubuntu-24.04, папка compose/
./sim.sh up wslg-nvidia
docker exec -it sverk_sitl bash
ros2 launch launch_system full_system_sitl.launch.py \
  wslg_gpu_gui:=true obrik_config:=base gz_world:=obrik_aruco
```

Используйте именно `wslg_gpu_gui:=true`, **не** `wslg_gui` — последний течёт по
памяти (2–5 ГБ/мин, gz-sim#2873); в compose стоит лимит 10 ГБ как страховка.

### Linux X11 (нативный десктоп)

```bash
xhost +local:docker            # один раз, разрешить X11
./sim.sh up linux-host
docker exec -it sverk_sitl bash
ros2 launch launch_system full_system_sitl.launch.py \
  host_x11_gui:=true obrik_config:=base gz_world:=obrik_aruco
```

### headless (без картинки)

Физика, камера, ArUco и VPE работают — просто нет окна Gazebo. Полезно для
автотестов и слабых машин.

```bash
./sim.sh up headless
docker exec -it sverk_sitl bash
ros2 launch launch_system full_system_sitl.launch.py \
  headless:=true obrik_config:=base gz_world:=obrik_aruco
```

---

## 5. Модели дрона и миры

`obrik_config` выбирает модель (оснащение), `gz_world` — мир (поле). Задаются в
команде `ros2 launch`.

### Модели (`obrik_config`)

| `obrik_config` | Оснащение | ArUco-навигация |
|---|---|---|
| `base` | камера, нижний дальномер, LED | да |
| `lidar` | + 2D-лидар LD19 | да |
| `one_rangefinder` | + доп. дальномер | да |
| `three_rangefinders_30` | + три обзорных дальномера 30° | да |
| `three_rangefinders_45` | + три обзорных дальномера 45° | да |
| `graffiti` | + передний дальномер, servo/спрей | да |
| `graffiti_old` | прежний servo/спрей | да |
| `gigaobrik` | MTF-01 + дальномер (без камеры/ArUco) | нет |
| `x500` / `stock` / `legacy` | штатная модель PX4 | зависит от модели |

### Миры (`gz_world`)

| `gz_world` | Поле |
|---|---|
| `obrik_aruco` | поле обрика с ArUco-метками (карта `markers_large.txt`) |
| `obrik_aruco_graffiti_wall` | поле ArUco со стеной для распылителя |
| `aruco` | штатный мир PX4 (карта `markers.txt`) |

Свою карту меток можно задать: `aruco_map_file:=/absolute/path/markers.txt`
(координаты и размеры меток должны совпадать с миром Gazebo).

---

## 6. Режимы навигации

Все команды — **внутри контейнера** (`docker exec -it sverk_sitl bash`). Флаг
показа (`vnc_gui` / `wslg_gpu_gui` / `host_x11_gui` / `headless`) подставьте под
свой режим из раздела 4 — ниже для примера noVNC.

**Навигация по ArUco** (XY и yaw — от меток, высота — от нижнего дальномера):

```bash
ros2 launch launch_system full_system_sitl.launch.py \
  vnc_gui:=true obrik_config:=graffiti gz_world:=obrik_aruco \
  enable_vio:=true allow_send_vio:=true
```

Взлетать после сообщения **`GPS-to-ArUco handoff complete`** в логе.

**Навигация по GPS** (XY от GPS, ArUco не подаётся в PX4):

```bash
ros2 launch launch_system full_system_sitl.launch.py \
  vnc_gui:=true obrik_config:=graffiti gz_world:=obrik_aruco \
  enable_vio:=false allow_send_vio:=false
```

**Проверка ArUco без подачи в PX4** (смотрим позу, но EKF её не использует):

```bash
ros2 launch launch_system full_system_sitl.launch.py \
  vnc_gui:=true obrik_config:=graffiti gz_world:=obrik_aruco \
  enable_vio:=true allow_send_vio:=false
```

---

## 7. Полёт: взлёт, посадка

Команды — **в новом терминале внутри контейнера** (симулятор оставьте
запущенным в первом). Если окружение не подхватилось:

```bash
source /opt/ros/humble/setup.bash && source /home/sverk/sverk_ws/install/setup.bash
```

**Взлёт на 1 м** (`auto_arm: true` — сам заармит):

```bash
ros2 service call /navigate offboard_interfaces/srv/Navigate \
  "{x: 0.0, y: 0.0, z: 1.0, yaw: 0.0, speed: 0.2, frame_id: 'map', auto_arm: true}"
```

**Полёт в точку** — меняйте `x/y/z/yaw`. **Посадка:**

```bash
ros2 service call /land std_srvs/srv/Trigger "{}"
```

**Распылитель (для `graffiti`)** — нажать / вернуть:

```bash
ros2 topic pub --once /servo_node/target_angle_deg std_msgs/msg/Float32 '{data: 156.52}'
ros2 topic pub --once /servo_node/target_angle_deg std_msgs/msg/Float32 '{data: 90.0}'
```

---

## 8. Что проверить

### EKF сошёлся (перед взлётом, ~2–3 мин)

```bash
ros2 topic echo /fmu/out/vehicle_local_position \
  --qos-reliability best_effort --qos-durability volatile
```
Ждём `xy_valid: true` и `z_valid: true`.

### Симуляция идёт в реальном времени (эталон ~1.0)

```bash
gz topic -e -t /world/obrik_aruco/stats -n 1 | grep real_time_factor
```
На слабом CPU в noVNC значение будет ниже 1.0 — это нормально, симуляция просто
идёт медленнее реального времени.

### ArUco детектируется

```bash
ros2 topic hz /aruco/world_pose          # есть частота — метки видны
ros2 topic echo --once /aruco/world_pose
ros2 topic echo --once /fmu/out/estimator_status_flags
```
После handoff ожидаются `cs_ev_pos=true`, `cs_ev_yaw=true`, `cs_rng_hgt=true`,
`cs_gps=false`, `dead_reckoning=false`.

### Веб-интерфейсы (в браузере на хосте)

| URL | Что |
|---|---|
| <http://localhost:6080/vnc.html> | рабочий стол Gazebo (только noVNC) |
| <http://localhost:8888> | rosboard — список топиков ROS |
| <http://localhost:8080> | web_video_server — видео с камеры |
| <http://localhost:5467> | VS Code в браузере (code-server) — только с аддоном `ide` |
| <http://localhost:8575> | butterfly — терминал в браузере — только с аддоном `ide` |

### Порты

| Порт | Назначение |
|---|---|
| 2222 | SSH в контейнер (`ssh sverk@localhost -p 2222`, пароль `sverk`) |
| 8888 / 9090 / 8080 | rosboard / ros_services_bridge / web_video_server |
| 6080 / 5900 | noVNC / VNC (только noVNC-режим) |
| 5467 / 8575 | code-server / веб-терминал (только с аддоном `ide`) |
| 8765 | MCP HTTP API (только с аддоном `mcp`) |
| 14550/udp | MAVLink → QGroundControl на хосте |
| 8086 | PID-ассистент (при `enable_pid_tuning:=true`) |

### QGroundControl (по желанию)

Если установлен QGroundControl — он сам подхватит поток MAVLink на UDP 14550 и
покажет дрон на карте, телеметрию, параметры.

---

## 9. Остановка и сброс

```bash
# внутри контейнера остановить симулятор: Ctrl+C
# либо снаружи убить процессы:
docker exec sverk_sitl bash -c 'pkill -f "ros2 launch"; pkill -f "gz sim"; pkill -x px4'

# остановить контейнер (том /home/sverk сохраняется — параметры, логи, сборки):
./sim.sh down novnc        # режим — тот же, что использовали при up
```

Полный сброс (удалить и данные, при следующем `up` — чистый `/home/sverk` из
образа) — добавьте `-v` к `down` вручную:

```bash
docker compose -f docker-compose.sim.yml -f docker-compose.sim.novnc.yml down -v
```

Набор `-f` у `down` должен совпадать с тем, каким делали `up` (обёртки `sim.sh`/
`sim.ps1` подставляют его по имени режима автоматически).

---

## 10. Если что-то не так

- **Окно Gazebo не появилось в WSLg / Intel Arc** — переключитесь на noVNC:
  `./sim.sh down wslg-intel` → `./sim.sh up novnc`. noVNC работает всегда.
- **`docker exec ... sverk_sitl` → No such container** — контейнер не поднят,
  сделайте `./sim.sh up <режим>`.
- **Дрон не армится / не взлетает** — не сошёлся EKF: подождите 2–3 мин и
  проверьте `xy_valid/z_valid` (раздел 8). В ArUco-режиме дождитесь
  `GPS-to-ArUco handoff complete`.
- **noVNC чёрный/не грузится** — обновите вкладку :6080 через 10–20 с после `up`
  (Xvfb/websocket поднимаются не мгновенно).
- **`ros2: command not found` внутри** — подгрузите окружение (раздел 7).
- **Порт занят (2222/8888/6080…)** — остановите прошлый контейнер (`./sim.sh
  down …`) или закройте занявшую порт программу.
