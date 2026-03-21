# Sverk ROS2

Проект автономного управления дроном на базе ROS 2 и PX4 v1.15.4

## Структура проекта

Ниже представлена организация файлов и директорий репозитория. Каждый элемент сопровождается кратким описанием его назначения.

```plaintext
.
├── airframes                    # Конфигурации различных типов дронов (рамы, параметры)
├── docs                         # Документация проекта
├── main_package                 # Центральный пакет для запуска всей системы
│   ├── launch_system           # Командный пункт: скрипты запуска
│   │   └── launch              # Launch-файлы для различных сценариев
│   └── self_check              # Пакет для автоматической проверки работоспособности всех компонентов
├── odometry                     # Модули для оценки положения и карты
│   ├── aruco                    # Работа с Aruco-метками
│   │   ├── aruco_det_loc       # Детекция меток и локализация по ним
│   │   ├── aruco_map           # Публикация карты Aruco-меток
│   │   └── aruco_pose          # Оценка позы по Aruco-меткам
│   └── vpe                      # Визуальная одометрия (Visual Pose Estimation)
│       ├── px4_local_pose_publisher  # Публикация локальной позиции от PX4 в ROS
│       └── px4-ros2-interface-lib    # Библиотека для отправки одометрии и команд в PX4 (подмодуль)
├── offboard                     # Режимы внешнего пилотирования (offboard control)
│   ├── fmu_calibration_control  # Пакет для отправки команд на полётный контроллер
│   ├── offboard_control         # Основной пакет для управления дроном из ROS (offboard)
│   │   └── examples             # Примеры скриптов на Python для offboard-полётов
│   └── offboard_interfaces      # Пользовательские сообщения для offboard-режима
├── peripheral                   # Драйверы и интерфейсы периферийных устройств
│   ├── camera_calibration       # Калибровка камеры
│   ├── camera_ros               # ROS-пакет для получения изображений с камеры (подмодуль)
│   ├── led                      # Управление светодиодной индикацией
│   │   ├── led_control          # Узел управления светодиодами
│   │   └── led_interfaces       # Сообщения и сервисы для управления светодиодами
│   └── sensors_broadcaster      # Публикация данных с датчиков
├── px4                          # Интеграция с автопилотом PX4
│   ├── Micro-XRCE-DDS-Agent     # Агент для связи PX4 с ROS2 через DDS (подмодуль)
│   ├── px4_msgs                 # Сообщения ROS2 для PX4 (подмодуль)
│   └── px4_ros_com              # Мост между ROS2 и PX4 (подмодуль)
├── simulation                   # Компоненты для симуляции
├── sverk_interfaces             # Высокоуровневая Python-библиотека для управления дроном
│   └── examples                 # Примеры использования библиотеки
└── web                          # Веб-интерфейсы и связанные компоненты
    ├── blockly                  # Визуальное программирование через Blockly
    ├── ros_services_bridge      # Мост для веб-сервисов ROS
    └── rosboard                 # Веб-интерфейс для мониторинга ROS (подмодуль)
```

## Ключевые компоненты

### Система запуска (`main_package`)

- **`launch_system`** — главные launch-файлы для различных сценариев:
  - Запуск всей системы на реальном дроне
  - Запуск в симуляторе с различными сенсорами (камера, камера глубины, лидар)
- [self_check](main_package/self_check/Readme.md) — автоматическая проверка работоспособности всех компонентов системы

### Одометрия (`odometry`)

- [aruco](odometry/aruco/aruco_map/README.md) — оценка положения с использованием Aruco-меток:
  - Детекция меток и локализация по ним
  - Публикация карты меток
  - Оценка позы дрона
- [vpe](odometry/vpe/px4_local_pose_publisher/README.md) — визуальная одометрия и интеграция с PX4:
  - Публикация локальной позиции от PX4 в ROS
  - Библиотека для отправки одометрии и команд в PX4

### Управление полётом (`offboard`)

- [offboard_control](offboard/offboard_control/Readme.md) — основной пакет для автономного полёта:
  - Примеры скриптов на Python для offboard-полётов
  - Управление позицией, скоростью, ориентацией
- [fmu_calibration_control](offboard/fmu_calibration_control/README.md) — управление полётным контроллером:
  - Дизарм, force_disarm, kill switch
  - Калибровки датчиков (гироскоп, магнитометр, барометр, акселерометр)
- **`offboard_interfaces`** — сообщения для offboard-режима

### Периферийные устройства (`peripheral`)

- [camera_ros](peripheral/camera_ros/README.md) — получение изображений с камеры (подмодуль)
- [camera_calibration](peripheral/camera_calibration/README.md) — калибровка камеры
- [led](peripheral/led/led_control/ReadMe.md) — управление светодиодной лентой:
  - Эффекты и цвет всей ленты
  - Управление отдельными светодиодами
- [sensors_broadcaster](peripheral/sensors_broadcaster/Readme.md) — публикация данных с лазерных датчиков

### Интеграция с PX4 (`px4`)

Подмодули для связи с автопилотом PX4:
- [px4_msgs](px4/px4_msgs/README.md) — сообщения ROS2 для PX4
- [px4_ros_com](px4/px4_ros_com/README.md) — мост между ROS2 и PX4
- [Micro-XRCE-DDS-Agent](px4/Micro-XRCE-DDS-Agent/README.md) — агент для связи PX4 с ROS2 через DDS

### Высокоуровневый API (`sverk_interfaces`)

Python-библиотека для управления дроном через ROS 2 сервисы:
- **`offboard_control`** — полёт, позиции, скорости, флип, телеметрия
- **`fmu_calibration_control`** — дизарм, force_disarm, kill switch, калибровки
- **`led_control`** — управление светодиодной лентой

Подробнее см. [README пакета](sverk_interfaces/README.md).

### Веб-интерфейсы (`web`)

- [rosboard](web/rosboard/README.md) — веб-интерфейс для мониторинга ROS (подмодуль)
- **`blockly`** — визуальное программирование через Blockly
- **`ros_services_bridge`** — мост для веб-сервисов ROS

## Документация

Подробная документация находится в директории [`docs/`](docs/).
