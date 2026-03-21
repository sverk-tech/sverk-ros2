# sensors_broadcaster

ROS 2 пакет для работы с группой датчиков расстояния **VL53L1X** через интерфейс I2C. Пакет позволяет динамически настраивать количество подключенных датчиков и используемые пины управления (XSHUT) через параметры.

## Предварительная настройка (Hardware)

### 1. Включение интерфейса I2C

Для работы с датчиками на Raspberry Pi необходимо активировать шину I2C.

1. Откройте конфигурационный файл:
```bash
sudo nano /boot/firmware/config.txt

```

2. Найдите и **раскомментируйте** (или добавьте) следующую строку:
```text
dtparam=i2c_arm=on

```


3. Сохраните файл и **перезагрузите** Raspberry Pi.

4. Откройте параметры Raspberry:
```bash
sudo raspi-config
```
5. Разблокируйте i2c:
- Перейдите в Interface Options → I2C.

- Если там написано "Would you like the ARM I2C interface to be enabled?", выберите Yes.




### 2. Схема подключения

Каждый датчик подключается к общим линиям `SDA` и `SCL`. Пин `XSHUT` каждого датчика должен быть подключен к отдельному GPIO пину Raspberry Pi для обеспечения возможности смены I2C адресов при инициализации.

---

## Параметры

| Параметр | Тип | Описание | Значение по умолчанию |
| --- | --- | --- | --- |
| `xshut_pins` | `int_array` | Список GPIO пинов, к которым подключены выходы XSHUT датчиков. | `[4, 17, 27]` |

---

## Запуск

### Команда запуска (через ros2 run):

Вы можете передать список пинов напрямую в терминале:

```bash
ros2 run sensors_broadcaster multi_vl53_node --ros-args -p xshut_pins:="[4, 17, 27]"

```

### Использование в Launch-файле:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensors_broadcaster',
            executable='multi_vl53_node',
            name='vl53l1x_sensors',
            parameters=[{'xshut_pins': [4, 17, 27]}]
        ),
    ])

```

---

## Публикуемые топики

Нода автоматически создает топики в зависимости от количества переданных пинов:

* `sensor_1/range` ([std_msgs/Float32]) — Расстояние с первого датчика (в метрах).
* `sensor_2/range` ([std_msgs/Float32]) — Расстояние со второго датчика.
* ...и так далее.

---
