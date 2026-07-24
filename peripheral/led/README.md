# ROS2 workspace for led_control

Пакеты для управления лентой WS2812 из ROS2 Humble (сервисы set_effect/set_leds, топик /led_control/state, визуализация событий PX4).

## Пакеты

- **led_interfaces** — сообщения и сервисы (`LEDState`, `LEDStateArray`, `SetLEDEffect`, `SetLEDs`).
- **led_control** — нода с эффектами и сервисами `/led_control/set_effect`, `/led_control/set_leds` и топиком `/led_control/state`.

## Зависимости

- ROS2 Humble
- Python 3.10+
- `pip install rpi5-ws2812` (до сборки или в venv)

## Сборка

```bash
cd ros2
colcon build --packages-select led_interfaces led_control
source install/setup.bash
```

## Запуск

```bash
ros2 launch led_control led.launch.py
```

Параметры в `src/led_control/config/led_params.yaml` (или свой файл через аргумент `config:=`).

Документация: [docs/ReadMe.md](led_control/ReadMe.md).
