# Работа со светодиодной лентой (ROS2)

Документация по управлению адресуемой лентой **WS2812** через ноду **led_control** в ROS2 Humble.

Тайминги WS2812 кодируются потоком байт на **SPI MOSI**. В текущей runtime-конфигурации нода использует **`ws2812_spidev`** и при его недоступности — **`rpi5_ws2812`** (см. `led_node.py`).

Ветви **`ws2812_mraa`** и **`ws2812_uart`** сохранены в проекте и документации для дальнейшей разработки, но их использование в основной ветке запуска сейчас закомментировано.

Каждый светодиод — 24 бита цвета (GRB на шине). Лента может отображать режимы полёта, этапы программы и уведомления.

---

## Raspberry Pi (кратко)

1. **Подключение:** DIN → MOSI (часто пин **19** для SPI0 или **38** для SPI1), GND и 5 V — см. выше про ток.
2. **SPI:** `raspi-config` → Interfacing Options → SPI; для второго контроллера — overlay в `/boot/firmware/config.txt` (например `dtoverlay=spi1-1cs`), перезагрузка.
3. **Запасной драйвер:** при необходимости `pip install rpi5-ws2812` (если не используются `ws2812_mraa` и `ws2812_spidev`).

> **Важно:** Лента может потреблять большой ток. Не питайте длинную ленту только от вывода 5 V платы — используйте отдельный BEC или блок питания 5 V и общую землю.

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
| `spi_bus` | int | `0` | С **MRAA**: логический номер SPI (часто `0`). С **spidev**: индекс **X** в `/dev/spidevX.Y` (`ls /dev/spidev*`). |
| `spi_device` | int | `0` | С **spidev**: **Y** в `/dev/spidevX.Y`. С **MRAA**: не переключает CS (поле для совместимости YAML). |
| `brightness` | int/float | `100` | Яркость 0–100 %; внутри `set_brightness(0.0–1.0)` у драйвера ленты. |
| `state_publish_rate` | float | `10.0` | Частота публикации топика `/led_control/state` (Гц). |
| `animation_rate` | float | `30.0` | Частота обновления анимации эффектов (Гц). |

#### Параметры визуализации событий (Events)

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `led_notify` | bool | `true` | Включить автоматическую смену эффекта/цвета по событиям (подписка на топики PX4 и `/rosout`). При `false` события не подписываются, применяется только ручной вызов сервисов. |
| `fmu_out_prefix` | строка | `"/fmu/out"` | Префикс топиков PX4 (публикации FMU). Подписки: `{fmu_out_prefix}/vehicle_status`, `{fmu_out_prefix}/battery_status`. См. [dds_topics](https://docs.px4.io/main/en/middleware/dds_topics). |
| `vehicle_status_timeout` | float | `2.0` | Таймаут (сек) без сообщений `vehicle_status`, после которого считается потеря связи и применяется событие `disconnected`. |
| `battery_min_voltage_per_cell` | float | `3.5` | Минимальное напряжение на одну банку (В). Расчёт: `voltage_v / cell_count` из [BatteryStatus](https://docs.px4.io/main/en/msg_docs/BatteryStatus). Если напряжение на банку ниже этого порога — событие `low_battery`. При невалидных `voltage_v` или `cell_count` событие не применяется. |
| `events` | строка (YAML) | см. пример ниже | Словарь «имя события → настройка». Настройка: `effect` (строка, опционально, по умолчанию `fill`), `r`, `g`, `b` (целые 0–255). Для **любого** события можно указать `effect: none` — тогда оно отключено, на него ничего не назначается (полная кастомизация набора событий). |

---

### Сервисы

#### 1. `/led_control/set_effect`

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
ros2 service call /led_control/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'fill', r: 255, g: 0, b: 0}"
```

---

#### 2. `/led_control/set_leds`

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
ros2 service call /led_control/set_leds led_interfaces/srv/SetLEDs "{leds: [{index: 0, r: 255, g: 0, b: 0}, {index: 1, r: 0, g: 255, b: 0}]}"
```

---

### Топики

#### Публикуемые нодой

| Топик | Тип | Частота | Описание |
|-------|-----|---------|----------|
| `/led_control/state` | `led_interfaces/msg/LEDStateArray` | По параметру `state_publish_rate` (по умолчанию 10 Гц) | Текущее состояние ленты: массив сообщений `LEDState` с полями `index`, `r`, `g`, `b` для каждого светодиода. Длина массива равна `led_count`. |

**Структура сообщения `LEDStateArray`:**
- `leds` — массив `LEDState[]`.
- Каждый `LEDState`: `index` (uint32), `r` (uint8), `g` (uint8), `b` (uint8).

#### Топики, на которые подписывается нода (при `led_notify: true`)

| Топик | Тип | Описание |
|-------|-----|----------|
| `{fmu_out_prefix}/vehicle_status` | `px4_msgs/msg/VehicleStatus` | Состояние аппарата PX4: арм, режим полёта (`nav_state`). Используется для событий `connected`, `disconnected`, `armed`, `disarmed` и режимов (см. таблицу Events). Требуется пакет `px4_msgs` в workspace. |
| `{fmu_out_prefix}/battery_status` | `px4_msgs/msg/BatteryStatus` | Статус батареи. Индикация по напряжению на банку: `voltage_v / cell_count` < `battery_min_voltage_per_cell` → событие `low_battery`. См. [BatteryStatus (PX4)](https://docs.px4.io/main/en/msg_docs/BatteryStatus). |
| `/rosout` | `rcl_interfaces/msg/Log` | Логи ROS2. При уровне `ERROR` применяется событие `error`. |

---

### События (Events) — полная таблица

События по полётному контроллеру соответствуют сообщению PX4 **VehicleStatus** (UORB): режимы полёта — полю `nav_state`, арм — `arming_state`. См. [VehicleStatus (PX4)](https://docs.px4.io/main/en/msg_docs/VehicleStatus).

Настройка событий задаётся в параметре **`events`** в виде YAML-словаря. Формат записи для каждого события: `имя_события: { effect?: "<имя_эффекта>", r: <0–255>, g: <0–255>, b: <0–255> }`. Поле `effect` необязательно; если не указано, используется `fill`. Для **любого** события можно указать **`effect: none`** — тогда это событие отключено: на него ничего не назначается и оно не учитывается. Так можно максимально кастомизировать набор событий (например, отключить только `startup`, или `acro`, или `orbit`, оставив остальные).

**Доступные эффекты:** `fill`, `blink`, `blink_fast`, `fade`, `wipe`, `flash`, `rainbow`, `rainbow_fill`. Специальное значение **`none`** — данное событие отключено (применимо к любому имени события в `events`).

| Имя события | Описание | Условие срабатывания | Источник данных |
|-------------|----------|----------------------|-----------------|
| `startup` | Старт ноды | Один раз при запуске узла (если `led_notify: true` и в `events` есть `startup` и у него не указано `effect: none`). | Внутреннее состояние ноды. |
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
| `position_slow` | Позиционный режим (медленный) | `nav_state == NAVIGATION_STATE_POSITION_SLOW` (6). | [VehicleStatus](https://docs.px4.io/main/en/msg_docs/VehicleStatus). |
| `altitude_cruise` | Высота с крейсерским режимом | `nav_state == NAVIGATION_STATE_ALTITUDE_CRUISE` (8). | `vehicle_status.nav_state`. |
| `descend` | Снижение (без позиционного контроля) | `nav_state == NAVIGATION_STATE_DESCEND` (12). | `vehicle_status.nav_state`. |
| `termination` | Режим аварийного завершения | `nav_state == NAVIGATION_STATE_TERMINATION` (13). | `vehicle_status.nav_state`. |
| `follow_target` | Авто-следование за целью | `nav_state == NAVIGATION_STATE_AUTO_FOLLOW_TARGET` (19). | `vehicle_status.nav_state`. |
| `precland` | Точная посадка по посадочной метке | `nav_state == NAVIGATION_STATE_AUTO_PRECLAND` (20). | `vehicle_status.nav_state`. |
| `vtol_takeoff` | Взлёт VTOL (переход, выход в loiter) | `nav_state == NAVIGATION_STATE_AUTO_VTOL_TAKEOFF` (22). | `vehicle_status.nav_state`. |
| `low_battery` | Низкое напряжение на банку | В `BatteryStatus`: `voltage_v / cell_count` < `battery_min_voltage_per_cell` (В). При невалидных `voltage_v` или `cell_count` событие не применяется. | Топик `{fmu_out_prefix}/battery_status` ([BatteryStatus](https://docs.px4.io/main/en/msg_docs/BatteryStatus)). |
| `error` | Ошибка в ROS | В топике `/rosout` получено сообщение с уровнем `level == 40` (ERROR). | Топик `/rosout`, тип `rcl_interfaces/msg/Log`. |

**Итоговая настройка конфигурации (config/led_params.yaml):**

- Все параметры ноды задаются в секции `led_node.ros__parameters`.
- Параметр **`events`** — многострочная YAML-строка (после `|`): словарь «событие → настройка». Для **любого** события можно указать **`effect: none`** — тогда оно отключено и не применяется (полная кастомизация: можно отключить startup, acro, orbit, error и т.д. по желанию).
- Цвета и эффекты для включённых событий задаются как `effect` (опционально, по умолчанию `fill`) и `r`, `g`, `b` (0–255).

**Пример полного блока `events` в led_params.yaml:**

```yaml
led_node:
  ros__parameters:
    led_notify: true
    fmu_out_prefix: "/fmu/out"
    vehicle_status_timeout: 2.0
    battery_min_voltage_per_cell: 3.5
    events: |
      startup:        { effect: rainbow }
      connected:      { effect: rainbow }
      disconnected:   { effect: blink, r: 255, g: 50, b: 50 }
      armed:          { r: 0, g: 255, b: 0 }
      disarmed:       { r: 255, g: 255, b: 255 }
      acro:           { r: 255, g: 165, b: 0 }   # для любого события: { effect: none } — отключить
      stabilized:     { r: 0, g: 255, b: 0 }
      altctl:         { r: 255, g: 255, b: 0 }
      posctl:         { r: 0, g: 0, b: 255 }
      offboard:       { r: 255, g: 0, b: 255 }
      mission:        { r: 0, g: 255, b: 255 }
      rtl:            { r: 255, g: 140, b: 0 }
      land:           { r: 255, g: 200, b: 0 }
      orbit:          { r: 100, g: 149, b: 237 }
      takeoff:        { r: 0, g: 255, b: 128 }
      position_slow:  { r: 200, g: 200, b: 255 }
      altitude_cruise:{ r: 255, g: 218, b: 185 }
      descend:        { r: 255, g: 180, b: 100 }
      termination:    { effect: blink, r: 255, g: 0, b: 0 }
      follow_target:  { r: 147, g: 112, b: 219 }
      precland:       { r: 255, g: 228, b: 196 }
      vtol_takeoff:   { r: 0, g: 206, b: 209 }
      error:          { effect: flash, r: 255, g: 0, b: 0 }
      low_battery:    { effect: blink_fast, r: 255, g: 0, b: 0 }
```

Соответствие событий полям сообщения PX4 **VehicleStatus** (UORB): события по режиму полёта берутся из поля `nav_state`, по арму — из `arming_state`. Полный список констант `nav_state` см. в [VehicleStatus (PX4)](https://docs.px4.io/main/en/msg_docs/VehicleStatus).

Отключение визуализации событий: в `led_params.yaml` задать `led_notify: false`. Отключение одного или нескольких событий: в блоке `events` для любого события указать `effect: none` (например `startup: { effect: none }`, `orbit: { effect: none }`). Для работы событий от PX4 в workspace должен быть собран пакет **px4_msgs** (версии сообщений — как в прошивке PX4). См. [PX4 ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide).

---

## Настройка визуализации событий (по топикам PX4)

Лента может автоматически показывать состояние полётного контроллера: подключение, арм, режим полёта, низкий заряд батареи, ошибки. Источники данных — топики PX4 через uXRCE-DDS ([ROS 2 User Guide](https://docs.px4.io/main/en/ros2/user_guide), [dds_topics](https://docs.px4.io/main/en/middleware/dds_topics)).

Вся настройка цветов и эффектов по событиям задаётся в **`config/led_params.yaml`** в параметре **`events`** (YAML-строка). Полный список параметров, все события с условиями срабатывания и пример блока `events` приведены в разделе [Полная справка: launch, параметры, сервисы, топики, Events](#полная-справка-launch-параметры-сервисы-топики-events). Чтобы отключить подсветку по событиям, задайте в конфиге `led_notify: false`. Для событий от PX4 в workspace должен быть собран пакет **px4_msgs**.

---

## Высокоуровневое управление (эффекты)

Сервис **`/led_control/set_effect`** переключает эффект и цвет ленты.

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
ros2 service call /led_control/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'fill', r: 255, g: 0, b: 0}"

# Плавный переход к синему
ros2 service call /led_control/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'fade', r: 0, g: 0, b: 255}"

# Радуга
ros2 service call /led_control/set_effect led_interfaces/srv/SetLEDEffect "{effect: 'rainbow', r: 0, g: 0, b: 0}"
```

### Пример на Python

```python
import rclpy
from rclpy.node import Node
from led_interfaces.srv import SetLEDEffect


class LedClient(Node):
    def __init__(self):
        super().__init__("led_client")
        self.client = self.create_client(SetLEDEffect, "/led_control/set_effect")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /led_control/set_effect...")

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

Сервис **`/led_control/set_leds`** задаёт цвет выбранных светодиодов по индексу и RGB.

**Запрос:** массив сообщений с полями `index`, `r`, `g`, `b`.

### Пример из командной строки

```bash
ros2 service call /led_control/set_leds led_interfaces/srv/SetLEDs "{
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
        self.client = self.create_client(SetLEDs, "/led_control/set_leds")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /led_control/set_leds...")

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

Текущее состояние ленты публикуется в топике **`/led_control/state`** (тип `led_interfaces/msg/LEDStateArray`): массив сообщений с полями `index`, `r`, `g`, `b` для каждого светодиода.

Просмотр из терминала:

```bash
ros2 topic echo /led_control/state
```

Получение количества светодиодов в Python:

```python
from led_interfaces.msg import LEDStateArray

# В ноде или подписчике:
msg = node.create_subscription(
    LEDStateArray, "/led_control/state", callback, 10
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

Перед сборкой установите зависимости на **целевой** машине:

- **MRAA + Python**: см. [установку MRAA для RK3588 / Bookworm](#mraa-install-rk3588) (модуль `mraa` для того же `python3`, что запускает ROS).
- Опционально **`spidev`**: `pip install spidev` или `pip install .[spidev]` из пакета, если нужен запасной путь `ws2812_spidev`.
- Опционально **`rpi5-ws2812`**: последний запасной импорт в `led_node.py`.
- ROS2 Humble: `ros-humble-rclpy`, пакеты из `package.xml` (интерфейсы, launch и т.д.).

---

## Использование в Docker

Для работы ноды в контейнере нужен доступ к **SPI** на хосте (те же `/dev/spidev*`, что видит MRAA/spidev), и обычно повышенные права:

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

Подставьте вместо `/dev/spidev0.0` фактический путь (например `/dev/spidev1.0`) и синхронизируйте `spi_bus` / `spi_device` в YAML с этими номерами.

