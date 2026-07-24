<div align="center">

# Sverk ROS 2 — фреймворк для программирования дронов

Открытый стек автономного управления дроном на **ROS 2** и **PX4**: от захвата
изображения с камеры и ArUco-навигации до offboard-полёта и высокоуровневого
Python-API.

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![PX4](https://img.shields.io/badge/PX4-v1.15.4-0b6bcb?logo=px4&logoColor=white)](https://px4.io/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white)](#)
[![C++](https://img.shields.io/badge/C++-17-00599C?logo=cplusplus&logoColor=white)](#)
[![Docs](https://img.shields.io/badge/docs-GitBook-3884FF?logo=gitbook&logoColor=white)](docs/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Platforms](https://img.shields.io/badge/platforms-Orange_Pi_·_Radxa_·_RPi-C51A4A?logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCIgZmlsbD0iI2ZmZiI%2BPHBhdGggZD0iTTkgMWgydjNoMlYxaDJ2M2gxYTMgMyAwIDAgMSAzIDN2MWgzdjJoLTN2MmgzdjJoLTN2MWEzIDMgMCAwIDEtMyAzaC0xdjNoLTJ2LTNoLTJ2M0g5di0zSDhhMyAzIDAgMCAxLTMtM3YtMUgydi0yaDN2LTJIMlY5aDNWOGEzIDMgMCAwIDEgMy0zaDFWMXpNOCA2YTIgMiAwIDAgMC0yIDJ2OGEyIDIgMCAwIDAgMiAyaDhhMiAyIDAgMCAwIDItMlY4YTIgMiAwIDAgMC0yLTJIOHptMSAzaDZ2Nkg5Vjl6Ii8%2BPC9zdmc%2BCg%3D%3D)](#поддерживаемые-платформы)

[Документация](docs/) · [Быстрый старт](docs/first-flight.md) · [Примеры](sverk_interfaces/examples) · [Структура](#структура-проекта)

</div>

---

## Возможности

- **ArUco-навигация** — детекция меток (coarse-to-fine), локализация и подача позы в PX4 EKF2.
- **Offboard-управление** — полёт по позиции/скорости/ориентации, траектории, excitation-сигналы для идентификации.
- **Автонастройка PID** — сбор телеметрии, оценка модели объекта и подсказки по коэффициентам.
- **Камера** — захват с MIPI/CSI (RPi, RK3588, Allwinner A733), калибровка и проверка.
- **Периферия** — LED WS2812, сервоприводы, лидар LD19, датчики VL53L1X, мост rosserial для Arduino.
- **Высокоуровневый API** — управление дроном из Python в несколько строк.
- **Веб-слой** — мониторинг ROS (rosboard), Blockly и HTTP/MCP-мост.
- **Единое окружение** — один Docker-контейнер на разных платах (см. [архитектуру](docs/knowledge_base/docker_architecture.md)).

## Поддерживаемые платформы

| Плата | Чип | Операционная система |
|---|---|---|
| **Orange Pi 5 Pro** (основная) | RK3588S + NPU 6 TOPS | Orange Pi OS |
| Radxa CM5 | RK3588S + NPU 6 TOPS | Radxa OS |
| Raspberry Pi CM5 | BCM2712 | Raspberry Pi OS |
| Orange Pi Zero 3W | Allwinner A733 | Orange Pi OS |
| Horizon RDK X3 | Sunrise X3 + BPU | vendor runtime |

## Быстрый старт

1. [Основы ROS 2](docs/ros2-basics.md) и [подготовка бортового компьютера](docs/onboard-computer.md).
2. [Настройка полётного контроллера](docs/flight-controller-setup.md) и [загрузка параметров](docs/params-upload.md).
3. [Первый полёт](docs/first-flight.md) — от арма до посадки.
4. Автономность — примеры в [`sverk_interfaces/examples`](sverk_interfaces/examples):

```python
import sverk_interfaces

drone = sverk_interfaces.init(Nodename="quickstart")
try:
    # взлёт на 1.5 м с авто-армом
    drone.control.navigate(x=0, y=0, z=1.5, frame_id="body", speed=0.5, auto_arm=True)
    # метр вперёд по оси body X
    drone.control.navigate(x=1, y=0, z=0, frame_id="body", speed=0.5)
    drone.control.land(timeout=10.0)
finally:
    drone.close()
```

> Полный набор сценариев (миссии, камера, LED, GPIO, телеметрия) — в [примерах](sverk_interfaces/examples) и [документации](docs/).

### Симулятор (без железа)

Попробовать стек без дрона можно в готовом SITL-симуляторе (PX4 + Gazebo Harmonic + ROS 2) — образ скачивается, собирать ничего не нужно:

```bash
cd simulation/compose
cp .env.example .env          # Windows: Copy-Item .env.example .env
./sim.sh up novnc             # Windows: .\sim.ps1 up novnc
```

Откройте <http://localhost:6080/vnc.html>. Подробности и остальные режимы (WSLg, IDE, MCP) — в [руководстве по симулятору](simulation/docs/guide.md).

## Структура проекта

```plaintext
.
├── airframes                         # Конфигурации рам и предустановки параметров
│   └── ros-uwb                       # Драйвер UWB-позиционирования
├── docs                              # Документация (GitBook)
├── main_package                      # Запуск и самопроверка системы
│   ├── launch_system                 # Launch-файлы (реальный дрон и SITL) + хелперы
│   ├── self_check                    # Автопроверка работоспособности компонентов
│   └── self_check_interfaces         # Интерфейсы для self_check
├── odometry                          # Оценка положения и карта
│   ├── aruco
│   │   ├── aruco_det_loc             # Детекция ArUco + локализация, TF-нода, CPU-affinity
│   │   ├── aruco_map                 # Публикация карты меток
│   │   └── aruco_pose                # Генерация карт и поза по меткам
│   └── vpe
│       ├── px4_local_pose_publisher  # Подача VPE/VIO в PX4 (+ lidar-одометрия)
│       └── px4-ros2-interface-lib    # Отправка одометрии/команд в PX4 (submodule)
├── offboard                          # Внешнее управление (offboard)
│   ├── fmu_control                   # Калибровки, арм/дизарм, reboot, параметры, логи
│   ├── offboard_control              # Автономный полёт, траектории, excitation
│   ├── offboard_interfaces           # Сообщения/сервисы offboard
│   └── pid_tuning_assistant          # Ассистент автонастройки PID
├── peripheral                        # Драйверы периферии
│   ├── a733_csi_cam_ros2             # CSI-камера для Allwinner A733 (submodule)
│   ├── camera_calibration            # Калибровка и проверка камеры
│   ├── camera_ros                    # Захват изображения с камеры (submodule)
│   ├── ld19_lidar                    # Драйвер лидара LDROBOT LD19
│   ├── led
│   │   ├── led_control               # Узел управления WS2812 (SPI/UART)
│   │   └── led_interfaces            # Сообщения/сервисы LED
│   ├── mipi_csi_cam_ros2             # MIPI CSI-камера (submodule)
│   ├── rosserial_ros2                # Мост rosserial для Arduino-устройств
│   ├── sensors_broadcaster           # Публикация датчиков (VL53L1X и др.)
│   ├── serial_bridge                 # Общий мост по serial
│   └── servo_control                 # Управление сервоприводами
├── px4                               # Интеграция с PX4
│   ├── Micro-XRCE-DDS-Agent          # Агент DDS (submodule)
│   ├── px4_msgs                      # Сообщения PX4 (submodule)
│   └── px4_ros_com                   # Мост ROS2 ↔ PX4 (submodule)
├── simulation                        # Симуляция (SITL)
├── sverk_interfaces                  # Высокоуровневый Python-API + примеры
└── web                               # Веб-слой
    ├── blockly                       # Визуальное программирование (Blockly)
    ├── ros2_mcp                      # HTTP/MCP-мост (ros2 CLI для LLM-агента)
    ├── ros_services_bridge           # Мост веб-сервисов ROS
    └── rosboard                      # Веб-мониторинг ROS (submodule)
```

## Ключевые компоненты

**Запуск и проверка**
- [launch_system](main_package/launch_system) — launch-файлы для реального дрона и SITL, хелперы параметров PX4.
- [self_check](main_package/self_check/README.md) — автоматическая проверка компонентов системы.

**Одометрия**
- [aruco_det_loc](odometry/aruco/aruco_det_loc/README.md) — детекция ArUco и локализация дрона по карте меток.
- [aruco_map](odometry/aruco/aruco_map/README.md) · [aruco_pose](odometry/aruco/aruco_pose) — карта меток и генерация карт.
- [px4_local_pose_publisher](odometry/vpe/px4_local_pose_publisher/README.md) — подача внешней позы в PX4 EKF2.

**Управление полётом**
- [offboard_control](offboard/offboard_control/README.md) — автономный полёт: позиция, скорость, ориентация, траектории.
- [fmu_control](offboard/fmu_control/README.md) — арм/дизарм, reboot, калибровки, параметры и логи по uXRCE-DDS.
- [pid_tuning_assistant](offboard/pid_tuning_assistant/README.md) — ассистент настройки PID.

**Периферия**
- [camera_calibration](peripheral/camera_calibration/README.md) · [ld19_lidar](peripheral/ld19_lidar/README.md) · [led_control](peripheral/led/led_control/README.md)
- [sensors_broadcaster](peripheral/sensors_broadcaster/README.md) · [serial_bridge](peripheral/serial_bridge/README.md) · [servo_control](peripheral/servo_control) · [rosserial_ros2](peripheral/rosserial_ros2/rosserial_ros2_bridge/README.md)

**Веб-слой**
- [rosboard](web/rosboard/README.md) — мониторинг ROS · [blockly](web/blockly) — визуальное программирование · [ros_services_bridge](web/ros_services_bridge/README.md) · [ros2_mcp](web/ros2_mcp/README.md) — HTTP/MCP-мост.

## Высокоуровневый API

[`sverk_interfaces`](sverk_interfaces/README.md) — Python-обёртка над ROS 2-сервисами стека:

- **offboard** — полёт, позиции, скорости, флип, телеметрия;
- **fmu** — арм/дизарм, kill switch, калибровки датчиков;
- **led** — эффекты и управление лентой WS2812.

## Документация

Полная документация — в директории [`docs/`](docs/) (GitBook): от основ ROS 2 и
подготовки платы до навигации по ArUco и архитектуры Docker.

## Лицензия

Проект распространяется под лицензией [MIT](LICENSE).
