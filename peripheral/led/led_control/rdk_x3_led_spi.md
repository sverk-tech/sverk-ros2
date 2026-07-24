# WS2812 и `led_control` на RDK X3 Module

Краткая привязка к официальной документации D-Robotics по SPI на 40-pin:

- [Using SPI (RDK DOC)](https://developer.d-robotics.cc/rdk_doc/en/Basic_Application/01_40pin_user_sample/spi)

## Что говорит документация RDK про X3

На **RDK X3** к разъёму 40-pin выведена шина **SPI2** чипа Sunrise X3M на физических контактах **19, 21, 23, 24**, один chip select, **логика 3.3 V**. Пример кода и loopback-тест — в образе: `python3 /app/40pin_samples/test_spi.py` (перед использованием ленты полезно убедиться, что SPI и `spidev` работают).

Для **WS2812** линию **DIN** подключают к **MOSI** (на типичной разводке 40-pin это часто **пин 19** для данных вместе с SCLK/CS согласно вашей схеме платы). **GND** общий с модулем. Питание **5 V** ленты — с отдельного источника при длинной ленте; при необходимости — согласование уровня 3.3 V → 5 V на данных.

## Связь с `led_control`

Нода шлёт тайминги WS2812 как поток байт по **SPI** (в коде используется драйвер из пакета `rpi5_ws2812`, который открывает **`/dev/spidevX.Y`**). Параметры **`spi_bus`** и **`spi_device`** должны совпадать с именем узла:

```bash
ls -l /dev/spidev*
```

Если видите, например, `/dev/spidev0.0`, в YAML задают `spi_bus: 0`, `spi_device: 0`. Номер шины в имени файла **не обязан** совпадать с «SPI2» в маркетинговом описании — ориентируйтесь только на фактический вывод `ls`.

Права:

```bash
groups   # желательна группа spi
sudo usermod -aG spi "$USER"
```

После смены группы — перелогиниться или новая сессия.

## Конфигурация в репозитории

- Файл **`config/led_params_rdkx3.yaml`** — стартовые значения под X3; обязательно поправьте **`led_count`** под вашу ленту и **`spi_bus` / `spi_device`** под ваш `ls /dev/spidev*`.
- В **`full_system_real.launch.py`** при `MK_NAME == "RDX"` и `ENABLE_LED` launch подключает `led.launch.py` с аргументом **`config:=…/led_params_rdkx3.yaml`**.

Ручной запуск без полного стека:

```bash
source /path/to/sverk_ws/install/setup.bash
ros2 launch led_control led.launch.py \
  config:=$(ros2 pkg prefix led_control)/share/led_control/config/led_params_rdkx3.yaml
```

## Длинная лента и размер транзакции SPI

Большой кадр WS2812 даёт длинный буфер байт. Если кадр «обрезается», см. комментарии в `led_control/ws2812_spidev.py` про размер чанка и `bufsiz` в sysfs (при переходе на чистый `spidev`-бэкенд).

## Сводка

| Шаг | Действие |
|-----|----------|
| 1 | Проверить SPI по [доке SPI](https://developer.d-robotics.cc/rdk_doc/en/Basic_Application/01_40pin_user_sample/spi) и `/app/40pin_samples/test_spi.py` |
| 2 | `ls /dev/spidev*`, выставить `spi_bus` / `spi_device` в `led_params_rdkx3.yaml` |
| 3 | DIN → MOSI (пин 19 в описании X3 для SPI2), GND, 5 V |
| 4 | Группа `spi`, пересборка `led_control` после добавления YAML в `setup.py` |
| 5 | Запуск через `full_system_real.launch.py` или `ros2 launch led_control led.launch.py config:=…` |
