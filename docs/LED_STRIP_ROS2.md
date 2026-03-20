# Работа со светодиодной лентой (ROS2)

Документация по управлению адресуемой лентой WS2812 через ноду **led_control** в ROS2 Humble.

Лента подключается к Raspberry Pi 5 по SPI (пин 19 или 38). Каждый светодиод может принимать один из 16 млн цветов (24 бита). Это позволяет отображать режимы полёта, этапы программы или уведомления.

---

## Подготовка

1. **Подключение ленты**  
   - **DIN** ленты → **MOSI** Raspberry Pi (физический пин **19** для SPI0 или **38** для SPI1).  
   - **GND** и **5 V** — к питанию (для длинных лент рекомендуется внешний блок питания 5 V и общая земля с Pi).

2. **Включение SPI**  
   Включите SPI в `raspi-config` → Interfacing Options → SPI. Для второго канала (пин 38) добавьте в `/boot/firmware/config.txt` строку `dtoverlay=spi1-1cs` и перезагрузитесь.

3. **Установка**  
   - Установите пакет драйвера: `pip install rpi5-ws2812`.  
   - Соберите ROS2-пакеты в workspace (см. ниже).

> **Важно:** Лента может потреблять большой ток. Не питайте длинную ленту только от Raspberry Pi — используйте отдельный BEC или блок питания 5 V.

---

## Запуск ноды

Из workspace с собранными пакетами:

```bash
ros2 launch led_control led.launch.py
```

Параметры задаются в `config/led_params.yaml` или при запуске (файл конфига можно переопределить):

```bash
ros2 launch led_control led.launch.py config:=/path/to/your/led_params.yaml
```

Подробная справка по launch-файлу, всем параметрам, сервисам, топикам и событиям — в разделе [Полная справка: launch, параметры, сервисы, топики, Events](#полная-справка-launch-параметры-сервисы-топики-events).

---

## Полная справка: launch, параметры, сервисы, топики, Events

### Launch-файл `led.launch.py`

**Расположение:** пакет `led_control`, путь при установке: `share/led_control/launch/led.launch.py`.

**Запуск:**
```bash
ros2 launch led_control led.launch.py [config:=<путь к YAML>]
```

#### Аргументы launch (DeclareLaunchArgument)

| Имя аргумента | Тип | Значение по умолчанию | Описание |
|---------------|-----|------------------------|----------|
| `config` | строка | путь к `config/led_params.yaml` в установленном пакете (например `.../share/led_control/config/led_params.yaml`) | Полный путь к файлу параметров ноды. Все параметры ноды (включая `led_count`, `spi_bus`, `spi_device`, `led_notify`, `events` и т.д.) загружаются из этого YAML. Переопределение через `config:=/path/to/your/led_params.yaml` заменяет весь набор параметров. |

#### Запускаемый узел (Node)

| Свойство | Значение |
|----------|----------|
| `package` | `led_control` |
| `executable` | `led_node` (entry point из `setup.py`: `led_node = led_control.led_node:main`) |
| `name` | `led_node` (имя узла в графе ROS2) |
| `output` | `screen` (логи в консоль) |
| `parameters` | Список из одного элемента: значение аргумента `config` (файл YAML). Параметры должны быть под ключом `led_node.ros__parameters` (см. пример `config/led_params.yaml`). |

---

### Параметры ноды (config/led_params.yaml)

Файл должен содержать секцию `led_node.ros__parameters`. Все параметры опциональны при объявлении в коде; ниже приведены используемые ключи и их значения по умолчанию.

#### Параметры ленты и SPI

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `led_count` | int | `58` | Количество светодиодов в ленте. |
| `spi_bus` | int | `0` | Номер шины SPI: `0` — SPI0 (физический пин 19, GPIO10), `1` — SPI1 (физический пин 38, GPIO20). |
| `spi_device` | int | `0` | Номер устройства на шине (Chip Select): обычно `0` (`/dev/spidev<bus>.<device>`). |
| `state_publish_rate` | float | `10.0` | Частота публикации топика `/led/state` (Гц). |
| `animation_rate` | float | `30.0` | Частота обновления анимации эффектов (Гц). |

#### Параметры визуализации событий (Events)

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `led_notify` | bool | `true` | Включить автоматическую смену эффекта/цвета по событиям (подписка на топики PX4 и `/rosout`). При `false` события не подписываются, применяется только ручной вызов сервисов. |
| `fmu_out_prefix` | строка | `"/fmu/out"` | Префикс топиков PX4 (публикации FMU). Подписки: `{fmu_out_prefix}/vehicle_status`, `{fmu_out_prefix}/battery_status`. См. [dds_topics](https://docs.px4.io/main/en/middleware/dds_topics). |
| `vehicle_status_timeout` | float | `2.0` | Таймаут (сек) без сообщений `vehicle_status`, после которого считается потеря связи и применяется событие `disconnected`. |
| `battery_low_threshold` | float | `0.15` | Порог остатка батареи (0.0–1.0). При `remaining <= battery_low_threshold` в сообщении `BatteryStatus` применяется событие `low_battery`. |
| `events` | строка (YAML) | см. пример ниже | Одна строка или многострочная YAML-строка — словарь «имя события → настройка». Настройка: `effect` (строка, опционально, по умолчанию `fill`), `r`, `g`, `b` (целые 0–255). |

---

### Сервисы

#### 1. `/led/set_effect`

**Тип:** `led_interfaces/srv/SetLEDEffect`

**Назначение:** переключить текущий эффект и цвет ленты (заливка, мигание, радуга и т.д.).

**Request:**

| Поле | Тип | Описание |
|------|-----|----------|
| `effect` | `string` | Имя эффекта: `fill`, `blink`, `blink_fast`, `fade`, `wipe`, `flash`, `rainbow`, `rainbow_fill`. Пустая строка трактуется как `fill`. |
| `r` | `uint8` | Красная компонента цвета (0–255). |
| `g` | `uint8` | Зелёная компонента (0–255). |
| `b` | `uint8` | Синяя компонента (0–255). |

**Response:**

| Поле | Тип | Описание |
|------|-----|----------|
| `success` | `bool` | Всегда `true` при успешной обработке. |

**Пример вызова:**
```bash
ros2 service call /led/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'fill', r: 255, g: 0, b: 0}"
```

---

#### 2. `/led/set_leds`

**Тип:** `led_interfaces/srv/SetLEDs`

**Назначение:** задать цвет отдельных светодиодов по индексу (низкоуровневое управление).

**Request:**

| Поле | Тип | Описание |
|------|-----|----------|
| `leds` | `led_interfaces/msg/LEDState[]` | Массив состояний. Каждый элемент: `index` (uint32) — индекс светодиода (0 … led_count-1), `r`, `g`, `b` (uint8) — цвет. |

**Response:**

| Поле | Тип | Описание |
|------|-----|----------|
| `success` | `bool` | Всегда `true` при успешной обработке. |

**Пример вызова:**
```bash
ros2 service call /led/set_leds led_interfaces/srv/SetLEDs "{leds: [{index: 0, r: 255, g: 0, b: 0}, {index: 1, r: 0, g: 255, b: 0}]}"
```

---

### Топики

#### Публикуемые нодой

| Топик | Тип | Частота | Описание |
|-------|-----|---------|----------|
| `/led/state` | `led_interfaces/msg/LEDStateArray` | По параметру `state_publish_rate` (по умолчанию 10 Гц) | Текущее состояние ленты: массив сообщений `LEDState` с полями `index`, `r`, `g`, `b` для каждого светодиода. Длина массива равна `led_count`. |

**Структура сообщения `LEDStateArray`:**
- `leds` — массив `LEDState[]`.
- Каждый `LEDState`: `index` (uint32), `r` (uint8), `g` (uint8), `b` (uint8).

#### Топики, на которые подписывается нода (при `led_notify: true`)

| Топик | Тип | Описание |
|-------|-----|----------|
| `{fmu_out_prefix}/vehicle_status` | `px4_msgs/msg/VehicleStatus` | Состояние аппарата PX4: арм, режим полёта (`nav_state`). Используется для событий `connected`, `disconnected`, `armed`, `disarmed` и режимов (см. таблицу Events). Требуется пакет `px4_msgs` в workspace. |
| `{fmu_out_prefix}/battery_status` | `px4_msgs/msg/BatteryStatus` | Статус батареи. Поле `remaining` (0.0–1.0) сравнивается с `battery_low_threshold` → событие `low_battery`. |
| `/rosout` | `rcl_interfaces/msg/Log` | Логи ROS2. При уровне `ERROR` применяется событие `error`. |

---

### События (Events) — полная таблица

Настройка событий задаётся в параметре **`events`** в виде YAML-словаря. Формат записи для каждого события: `имя_события: { effect?: "<имя_эффекта>", r: <0–255>, g: <0–255>, b: <0–255> }`. Поле `effect` необязательно; если не указано, используется `fill`.

**Доступные эффекты:** `fill`, `blink`, `blink_fast`, `fade`, `wipe`, `flash`, `rainbow`, `rainbow_fill`.

| Имя события | Описание | Условие срабатывания | Источник данных |
|-------------|----------|----------------------|-----------------|
| `startup` | Старт ноды | Один раз при запуске узла (если `led_notify: true` и в `events` есть `startup`). | Внутреннее состояние ноды. |
| `connected` | Связь с полётным контроллером установлена | Первое полученное сообщение `VehicleStatus` от PX4. | Топик `{fmu_out_prefix}/vehicle_status`. |
| `disconnected` | Связь с полётным контроллером потеряна | Нет сообщений `vehicle_status` в течение `vehicle_status_timeout` секунд (после того как ранее было `connected`). | Таймер-watchdog по подписке на `vehicle_status`. |
| `armed` | Двигатели взведены | В `VehicleStatus` поле `arming_state == ARMING_STATE_ARMED` (2). | `vehicle_status.arming_state`. |
| `disarmed` | Двигатели сняты с взвода | В `VehicleStatus` поле `arming_state == ARMING_STATE_DISARMED` (1). | `vehicle_status.arming_state`. |
| `stabilized` | Режим стабилизации (ручной по крену/тангажу) | `nav_state == NAVIGATION_STATE_STAB` (15) или `NAVIGATION_STATE_MANUAL` (0). | `vehicle_status.nav_state`. |
| `acro` | Acro-режим | `nav_state == NAVIGATION_STATE_ACRO` (10). | `vehicle_status.nav_state`. |
| `altctl` | Удержание высоты | `nav_state == NAVIGATION_STATE_ALTCTL` (1). | `vehicle_status.nav_state`. |
| `posctl` | Позиционный режим | `nav_state == NAVIGATION_STATE_POSCTL` (2). | `vehicle_status.nav_state`. |
| `offboard` | Offboard (внешнее управление) | `nav_state == NAVIGATION_STATE_OFFBOARD` (14). | `vehicle_status.nav_state`. |
| `mission` | Автомиссия / Loiter | `nav_state == NAVIGATION_STATE_AUTO_MISSION` (3) или `NAVIGATION_STATE_AUTO_LOITER` (4). | `vehicle_status.nav_state`. |
| `rtl` | Возврат на точку старта | `nav_state == NAVIGATION_STATE_AUTO_RTL` (5). | `vehicle_status.nav_state`. |
| `land` | Посадка | `nav_state == NAVIGATION_STATE_AUTO_LAND` (18). | `vehicle_status.nav_state`. |
| `orbit` | Орбита | `nav_state == NAVIGATION_STATE_ORBIT` (21). | `vehicle_status.nav_state`. |
| `takeoff` | Взлёт по команде | `nav_state == NAVIGATION_STATE_AUTO_TAKEOFF` (17). | `vehicle_status.nav_state`. |
| `low_battery` | Низкий заряд батареи | В `BatteryStatus` поле `remaining <= battery_low_threshold`. | Топик `{fmu_out_prefix}/battery_status`, поле `remaining`. |
| `error` | Ошибка в ROS | В топике `/rosout` получено сообщение с уровнем `level == 40` (ERROR). | Топик `/rosout`, тип `rcl_interfaces/msg/Log`. |

**Пример полного блока `events` в led_params.yaml:**

```yaml
led_node:
  ros__parameters:
    led_notify: true
    fmu_out_prefix: "/fmu/out"
    vehicle_status_timeout: 2.0
    battery_low_threshold: 0.15
    events: |
      startup:        { r: 255, g: 255, b: 255 }
      connected:      { effect: rainbow }
      disconnected:   { effect: blink, r: 255, g: 50, b: 50 }
      armed:          { r: 0, g: 255, b: 0 }
      disarmed:       { r: 255, g: 255, b: 255 }
      acro:           { r: 255, g: 165, b: 0 }
      stabilized:     { r: 0, g: 255, b: 0 }
      altctl:         { r: 255, g: 255, b: 0 }
      posctl:         { r: 0, g: 0, b: 255 }
      offboard:       { r: 255, g: 0, b: 255 }
      mission:        { r: 0, g: 255, b: 255 }
      rtl:            { r: 255, g: 140, b: 0 }
      land:           { r: 255, g: 200, b: 0 }
      orbit:          { r: 100, g: 149, b: 237 }
      takeoff:        { r: 0, g: 255, b: 128 }
      error:          { effect: flash, r: 255, g: 0, b: 0 }
      low_battery:    { effect: blink_fast, r: 255, g: 0, b: 0 }
```

Отключение визуализации событий: в `led_params.yaml` задать `led_notify: false`. Для работы событий от PX4 в workspace должен быть собран пакет **px4_msgs** (версии сообщений — как в прошивке PX4). См. [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide).

---

## Настройка визуализации событий (по топикам PX4)

Лента может автоматически показывать состояние полётного контроллера: подключение, арм, режим полёта, низкий заряд батареи, ошибки. Источники данных — топики PX4 через uXRCE-DDS ([ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide), [dds_topics](https://docs.px4.io/main/en/middleware/dds_topics)).

Вся настройка цветов и эффектов по событиям задаётся в **`config/led_params.yaml`** в параметре **`events`** (YAML-строка). Полный список параметров, все события с условиями срабатывания и пример блока `events` приведены в разделе [Полная справка: launch, параметры, сервисы, топики, Events](#полная-справка-launch-параметры-сервисы-топики-events). Чтобы отключить подсветку по событиям, задайте в конфиге `led_notify: false`. Для событий от PX4 в workspace должен быть собран пакет **px4_msgs**.

---

## Высокоуровневое управление (эффекты)

Сервис **`/led/set_effect`** переключает эффект и цвет ленты.

**Аргументы:**

- `effect` — название эффекта (строка).
- `r`, `g`, `b` — компоненты RGB (0–255).

**Доступные эффекты:**

| Эффект          | Описание |
|-----------------|----------|
| `fill` (или `""`) | Вся лента одним цветом. |
| `blink`         | Мигание заданным цветом. |
| `blink_fast`    | Быстрое мигание. |
| `fade`          | Плавный переход к заданному цвету. |
| `wipe`          | Заполнение ленты по одному светодиоду. |
| `flash`         | Два коротких вспышки, затем возврат к предыдущему эффекту. |
| `rainbow`       | Бегущая радуга по ленте. |
| `rainbow_fill`  | Вся лента одним цветом, цвет плавно меняется по радуге. |

### Пример из командной строки

```bash
# Заливка красным
ros2 service call /led/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'fill', r: 255, g: 0, b: 0}"

# Плавный переход к синему
ros2 service call /led/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'fade', r: 0, g: 0, b: 255}"

# Радуга
ros2 service call /led/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'rainbow', r: 0, g: 0, b: 0}"
```

### Пример на Python

```python
import rclpy
from rclpy.node import Node
from led_interfaces.srv import SetLEDEffect


class LedClient(Node):
    def __init__(self):
        super().__init__("led_client")
        self.client = self.create_client(SetLEDEffect, "led/set_effect")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /led/set_effect...")

    def set_effect(self, effect="fill", r=255, g=0, b=0):
        req = SetLEDEffect.Request()
        req.effect = effect
        req.r = r
        req.g = g
        req.b = b
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = LedClient()
    node.set_effect(r=255, g=0, b=0)           # красный
    node.set_effect(effect="fade", r=0, g=100, b=0)  # плавно к зелёному
    node.set_effect(effect="rainbow")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

## Низкоуровневое управление (отдельные светодиоды)

Сервис **`/led/set_leds`** задаёт цвет выбранных светодиодов по индексу и RGB.

**Запрос:** массив сообщений с полями `index`, `r`, `g`, `b`.

### Пример из командной строки

```bash
ros2 service call /led/set_leds led_interfaces/srv/SetLEDs "{
  leds: [
    {index: 0, r: 255, g: 0, b: 0},
    {index: 1, r: 0, g: 255, b: 0},
    {index: 2, r: 0, g: 0, b: 255}
  ]
}"
```

### Пример на Python

```python
import rclpy
from rclpy.node import Node
from led_interfaces.msg import LEDState
from led_interfaces.srv import SetLEDs


class LedClient(Node):
    def __init__(self):
        super().__init__("led_client")
        self.client = self.create_client(SetLEDs, "led/set_leds")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /led/set_leds...")

    def set_leds(self, leds_list):
        req = SetLEDs.Request()
        req.leds = leds_list
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main():
    rclpy.init()
    node = LedClient()
    node.set_leds([
        LEDState(index=0, r=255, g=0, b=0),
        LEDState(index=1, r=0, g=255, b=0),
        LEDState(index=2, r=0, g=0, b=255),
    ])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

После вызова `set_leds` нода переходит в режим «ручного» управления: заданные индексы обновляются, остальные сохраняют текущее состояние. Новый вызов `set_effect` снова включает эффекты и обновляет всю ленту.

---

## Топик состояния ленты

Текущее состояние ленты публикуется в топике **`/led/state`** (тип `led_interfaces/msg/LEDStateArray`): массив сообщений с полями `index`, `r`, `g`, `b` для каждого светодиода.

Просмотр из терминала:

```bash
ros2 topic echo /led/state
```

Получение количества светодиодов в Python:

```python
from led_interfaces.msg import LEDStateArray

# В ноде или подписчике:
msg = node.create_subscription(
    LEDStateArray, "led/state", callback, 10
)
# В callback или через wait_for_message:
# led_count = len(msg.leds)
```

---

## Сборка ROS2-пакетов

В workspace с исходниками (например `ros2/src`):

```bash
cd /path/to/rpi5-ws2812/ros2
colcon build --packages-up-to led_control
source install/setup.bash
```

Перед сборкой установите зависимости:

- `pip install rpi5-ws2812`
- ROS2 Humble: `ros-humble-rclpy`, пакеты из `package.xml` (интерфейсы, launch и т.д.)

---

## Использование в Docker

Для работы ноды в контейнере нужен доступ к SPI-устройству и привилегированный режим:

```bash
docker run --device /dev/spidev0.0 --privileged --network host YOUR_ROS2_IMAGE ros2 launch led_control led.launch.py
```

В `docker-compose`:

```yaml
services:
  led_node:
    image: YOUR_ROS2_IMAGE
    privileged: true
    devices:
      - /dev/spidev0.0
    command: ros2 launch led_control led.launch.py
```

Для ленты на втором SPI (пин 38) используйте `/dev/spidev1.0` и в конфиге задайте `spi_bus: 1`.

