## Radxa CM5 (Radxa OS): полный путь — SPI, проверка, программирование

Ниже — рабочий порядок действий, согласованный с официальными материалами Radxa. Конкретные **имена overlay** и **номера пинов** зависят от образа Radxa OS и **носителя (IO board)** для CM5; их нужно выбрать в интерфейсе и в [репозитории overlay](https://github.com/radxa/overlays), а не угадывать «по аналогии с другой платой».

### 1. Включить SPI (device tree overlay)

Инструмент конфигурации системы — **`rsetup`** ([документация rsetup](https://docs.radxa.com/en/som/cm/cm5/radxa-os/sys-config/rsetup)):

1. Запуск (нужны права администратора): `sudo rsetup` или через меню системы.
2. Главное меню → **Overlays** → **Manage overlays**.
3. Клавишей **Space** включите overlay, который **поднимает spidev** на нужном контроллере SPI (в списке обычно есть формулировки вроде *spidev on SPI…*; набор отличается от продукта к продукту, см. подсказку в [rsetup → Overlays](https://docs.radxa.com/en/som/cm/cm5/radxa-os/sys-config/rsetup)).
4. Подтвердите сохранение; **перезагрузите** плату — overlay применяется после reboot.

Дополнительно:

- Меню **Hardware → 40-pin GPIO** в `rsetup` позволяет настроить конфигурацию 40-pin разъёма, если это предусмотрено для вашей платы ([тот же раздел rsetup](https://docs.radxa.com/en/som/cm/cm5/radxa-os/sys-config/rsetup)).
- Параметры загрузки и overlay можно сверять с конфигурацией загрузчика, например: `cat /boot/extlinux/extlinux.conf` (упоминается в [rsetup](https://docs.radxa.com/en/som/cm/cm5/radxa-os/sys-config/rsetup)).
- Сторонние или свои DTBO: **Overlays → Install 3rd party overlay**; исходники Rockchip: `arch/arm64/boot/dts/rockchip/overlays/*.dts` в [radxa/overlays](https://github.com/radxa/overlays) ([инструкция в rsetup](https://docs.radxa.com/en/som/cm/cm5/radxa-os/sys-config/rsetup)).

Общая идея разъёма и периферии (SPI, пины) на примере документации по GPIO-разъёму — в [GPIO headers (Rock 4 и др.)](https://docs.radxa.com/en/rock4/rock4ab-se/getting-started/interface-usage/gpio-headers#gpio); **фактическую распиновку MOSI для CM5** возьмите из документа **вашего** IO-кабеля / модуля CM5.

### 2. Проверить, что SPI (spidev) появился

После перезагрузки:

```bash
ls -l /dev/spidev*
```

Должны появиться узлы вида `/dev/spidevB.D` (например `spidev0.0`). Числа **B** и **D** — это как раз **`spi_bus`** и **`spi_device`** в `config/led_params.yaml` и в конструкторе `WS2812SpiDriver`.

Права доступа:

```bash
groups   # желательно видеть группу spi
sudo usermod -aG spi "$USER"   # затем перелогиниться
```

Проверка на уровне стека SPI (по желанию):

- Утилиты **spi-tools** (если есть в репозитории образа): пакет часто называется `spi-tools`, тест `spidev_test` — пример использования и кода теста приведён в документации Radxa по [GPIO / SPI](https://docs.radxa.com/en/rock4/rock4ab-se/getting-started/interface-usage/gpio-headers#spi) (раздел про включение spidev overlay и SPI testing utility).
- **MRAA** ([Mraa на Radxa CM5](https://docs.radxa.com/en/som/cm/cm5/radxa-os/app-dev/mraa)): в доке приведён тест на C (`mraa_spi_init`); для ноды используется **Python** через модуль `mraa` и драйвер **`ws2812_mraa.py`**. Overlay **spidev** для выбранного SPI по-прежнему нужен — MRAA на Linux ходит к тому же `/dev/spidev*`.

<a id="mraa-install-rk3588"></a>

#### Установка MRAA на Debian Bookworm / Radxa OS (RK3588, CM5, Python 3.11)

**Почему не ставится apt `python3-mraa`:** пакет в репозитории может быть собран под другую ABI Python (в логах — `libpython3.9`), а в Bookworm системный интерпретатор — **3.11**.

**Почему после `cmake && make install` бывает `NameError` / `uint8Array` / `_uint8Array__mraa`:** в каталоге с модулями Python лежат **несогласованные** файлы — `mraa.py` и `_mraa*.so` из **разных** сборок, либо остался старый `mraa.py` без нового `.so`. Они должны ставиться **одной** сборкой.

**Шаг 0 — убрать сломанную установку в `/usr/local`:**

```bash
python3 -c "import site; print(site.getsitepackages())"
# удалите оттуда mraa.py и _mraa*.so, например:
sudo rm -f /usr/local/lib/python3.11/dist-packages/mraa.py \
             /usr/local/lib/python3.11/dist-packages/_mraa*.so
sudo ldconfig
```

---

##### Рекомендуется: сборка `.deb` с [radxa-pkg/mraa](https://github.com/radxa-pkg/mraa)

Это **официальный репозиторий исходников пакетов** `libmraa2` / `python3-mraa` для инфраструктуры Radxa (ветка `master`, каталог `debian/`). Сборка **на самой CM5 (RK3588)** даёт `python3-mraa`, слинкованный с **вашим** `python3-dev` (3.11).

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential devscripts debhelper cmake swig \
  python3-dev libjson-c-dev pkg-config git \
  default-jdk-headless

cd /tmp
git clone https://github.com/radxa-pkg/mraa.git
cd mraa
dpkg-buildpackage -us -uc -b -j"$(nproc)"
cd ..
sudo apt install ./libmraa2_*.deb ./python3-mraa_*.deb ./libmraa-dev_*.deb ./mraa-tools_*.deb
```

При конфликте с уже установленными пакетами mraa из apt: `sudo apt remove libmraa2 python3-mraa libmraa-dev mraa-tools` (учтите зависимости других пакетов), затем снова `sudo apt install ./…deb`.

Проверка:

```bash
python3 -c "import _mraa, mraa; from mraa import Spi; print(mraa.getVersion()); print(_mraa.__file__); Spi(0)"
```

---

##### Альтернатива: [nascs/mraa](https://github.com/nascs/mraa) (как в [доке Radxa](https://docs.radxa.com/en/som/cm/cm5/radxa-os/app-dev/mraa))

В доке ошибочно указано `git checkout -b undefined origin/undefined` — **не выполняйте**; клонируйте `master`/`main`. Ручной `cmake install` возможен, но чаще даёт ту же проблему SWIG, если смешать файлы из разных установок. Минимально безопасный вариант:

```bash
sudo apt-get install -y git cmake build-essential swig python3-dev libjson-c-dev pkg-config
cd /tmp && rm -rf mraa && git clone https://github.com/nascs/mraa.git && cd mraa
rm -rf build && mkdir build && cd build
cmake .. \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILDSWIGPYTHON=ON \
  -DBUILDSWIGNODE=OFF \
  -DBUILDSWIGJAVA=OFF \
  -DPYTHON_EXECUTABLE=/usr/bin/python3
make -j"$(nproc)"
sudo make install
sudo ldconfig
```

Если снова появится ошибка уровня **`uint8Array` / `_mraa`**, используйте сборку **`dpkg-buildpackage` из radxa-pkg/mraa** (блок выше) — она совпадает с тем, как Radxa собирает пакеты для репозитория.

### 3. Подключение ленты WS2812

- **DIN** (данные) → линия **MOSI** того SPI, для которого включили spidev.
- **GND** — общая земля с платой.
- **5 V** ленты — от адекватного источника 5 V (не от 3.3 V логики). Для длинных лент — отдельный БП 5 V, земля с SBC общая.

Уровни: выход SoC обычно **3.3 V**; многие ленты 5 V нормально читают 3.3 V на DIN, но при сбоях используйте преобразователь уровня на линии данных.

#### Radxa CM5 IO Board: три пункта SPI в `rsetup` и куда вешать DIN

В **Manage overlays** у вас доступны (как минимум):

| Overlay | Смысл |
|--------|--------|
| **Enable MCP2515 … on SPI0-M2 over CS0** | CAN-контроллер на SPI0 в режиме **M2**, chip select **0**. Не включайте вместе с spidev на тот же CS, если нужен только spidev. |
| **Enable spidev on SPI0-M2 over CS0** | Пользовательский **spidev** на SPI0-**M2**, **CS0** — обычно появляется **`/dev/spidev0.0`**. |
| **Enable spidev on SPI0-M2 over CS1** | Тот же SPI0-**M2** (те же **SCLK / MOSI / MISO**), другой **CS** — чаще всего второй узел, например **`/dev/spidev0.1`**. |

Важно:

- **CS0 и CS1 не меняют линию данных для WS2812**: лента слушает только **MOSI**. Разница только в номере устройства в ОС (`…0.0` или `…0.1`) и в том, какая линия CS задействована в железе.
- Для **`/dev/spidev0.0`** при одном включённом spidev-overlay логично выбрать вариант **… over CS0** и в YAML задать `spi_bus: 0`, `spi_device: 0`.

**Какой это сигнал на кристалле:** overlay **SPI0-M2** подключает к SPI0 пины из группы `spi0m2_*` в pinctrl Rockchip. Для **MOSI** это линия **`spi0_mosi_m2` → GPIO `GPIO1_B2`** (см. `spi0m2_pins` в `rk3588s-pinctrl.dtsi` в ядре Rockchip/Radxa). Исходник overlay: [rk3588-spi0-m2-cs0-spidev.dts](https://github.com/radxa/overlays/blob/main/arch/arm64/boot/dts/rockchip/overlays/rk3588-spi0-m2-cs0-spidev.dts) (в `metadata` перечислены занятые GPIO, включая **GPIO1_B2**).

**Какой физический контакт на разъёме 40 pin (CM5 IO):** в веб-таблице [Hardware Interface → 40 PIN GPIO](https://docs.radxa.com/en/som/cm/cm5/hardware/hw-interface) для CM5 IO чаще явно перечислен вариант **SPI0_MOSI_M1** (другой mux, не M2). Для режима **M2** ориентируйтесь на то, что на разъём должен выходить именно **`GPIO1_B2`** / **`SPI0_MOSI_M2`**:

1. Скачайте официальный **[Radxa CM5 V2.21 Pinout (xlsx)](https://dl.radxa.com/cm5/v2210/radxa_cm5_v2210_pinout.xlsx)** ([страница загрузок CM5](https://docs.radxa.com/en/som/cm/cm5/download)): на листе **U13-A** у контакта модуля **pin 44** в списке функций RK3588S указаны **SPI0_MOSI_M2** и **GPIO1_B2_***. Дальше по **схеме Radxa CM5 IO** ([например schematic в разделе Hardware Design](https://docs.radxa.com/en/som/cm/cm5/download)) проследите, на какой номер контакта **40-pin GPIO** выведена эта цепь — это и есть точный ответ для вашей платы.
2. Ориентир с **другой** платы Radxa с тем же overlay: у **ROCK 5A** в [Hardware interface → 40-Pin GPIO](https://docs.radxa.com/en/rock5/rock5a/hardware-design/hardware-interface) в таблице указано: **пин 42** — **SPI0_MOSI_M2** (сигнал **GPIO1_B2**), **пин 41** — **SPI0_MISO_M2** (**GPIO1_B1**). Разводка **CM5 IO может отличаться** от ROCK 5A; используйте это только как подсказку, если нет под рукой xlsx/schematic.

**Проверка на работающей плате:** после включения overlay и перезагрузки можно посмотреть, какие пины заняты SPI (зависит от ядра и debugfs), например:

```bash
sudo gpioinfo | grep -iE 'spi|gpio1'
```

или раздел pinctrl в debugfs, если он смонтирован. Надёжнее всего для «куда паять DIN» — свериться с **pinout xlsx + schematic** выше.

### 4. Минимальный тест без ROS (Python)

**Вариант MRAA** (после успешного `import mraa`, SPI overlay включён):

```python
#!/usr/bin/env python3
from led_control.ws2812_mraa import Color, WS2812SpiDriver

BUS, N = 0, 8   # BUS — логический SPI в MRAA (как в примере Radxa SPI_BUS 0)
drv = WS2812SpiDriver(spi_bus=BUS, spi_device=0, led_count=N)
strip = drv.get_strip()
strip.set_pixel_color(0, Color(0, 64, 0))
strip.show()
drv.close()
```

**Вариант spidev** (`pip install spidev` при необходимости):

```python
#!/usr/bin/env python3
from led_control.ws2812_spidev import Color, WS2812SpiDriver

BUS, DEV, N = 0, 0, 8  # из ls /dev/spidev*
drv = WS2812SpiDriver(spi_bus=BUS, spi_device=DEV, led_count=N)
strip = drv.get_strip()
strip.set_pixel_color(0, Color(0, 64, 0))
strip.show()
drv.close()
```

Запуск с правами на SPI (часто root или группа `spi`).

### 5. ROS2: параметры `spi_bus` / `spi_device`

- При работе через **MRAA** (`ws2812_mraa`): **`spi_bus`** — логический номер SPI в описании платы MRAA (часто **0** для первой шины, как в примере Radxa). **`spi_device`** в коде MRAA-драйвера не переключает CS (оставлено для совместимости YAML).
- При работе через **spidev**: задайте **B** и **D** из имени `/dev/spidevB.D`.

По умолчанию в ноде — `spi_bus: 0`, `spi_device: 0`.

### 6. Увеличение буфера spidev (длинная лента)

Если ядро обрезает передачу (~4096 байт), см. комментарии в `ws2812_spidev.py` и параметр `spi_chunk_size` / `bufsiz` в sysfs.

---