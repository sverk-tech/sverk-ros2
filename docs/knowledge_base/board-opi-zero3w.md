# Orange Pi Zero 3W (Allwinner A733)

Этот раздел описывает настройку бортового компьютера на базе Orange Pi Zero 3W: включать UART, SPI, I2C, PWM, подключать камеру IMX219, устанавливать Docker и настраивать режим максимальной производительности.

---

## Конфигурация системы

| Параметр     | Значение                       |
|--------------|--------------------------------|
| Плата        | Orange Pi Zero 3W              |
| SoC          | Allwinner A733                 |
| Ядро         | 6.6.98-sun60iw2                |
| ОС           | Debian Bookworm / Ubuntu 22.04 |
| Пользователь | `orangepi`                     |

> **Примечание** Главное отличие от Raspberry Pi: на Orange Pi интерфейсы включаются через файл `/boot/orangepiEnv.txt`, а не через `/boot/firmware/config.txt`. Утилита `orangepi-config` редактирует именно этот файл — всё что она делает, можно сделать вручную.

---

## 1. Установка операционной системы

### 1.1. Скачайте образ

Официальные образы доступны на сайте [orangepi.org](http://www.orangepi.org).

Рекомендуется:
- **Debian Bookworm Server** — минимальный образ без графического интерфейса, стабильная основа
- **Ubuntu Jammy Server** — альтернатива, если нужны пакеты Ubuntu

### 1.2. Запись образа на SD-карту

Используйте **balenaEtcher** (Windows / macOS / Linux):

1. Скачайте образ с сайта Orange Pi.
2. Откройте balenaEtcher, выберите образ и SD-карту.
3. Нажмите **Flash**.
4. После записи вставьте карту в плату и подайте питание.

### 1.3. Первый вход в систему

**Логины по умолчанию:**

| Пользователь | Пароль     |
|--------------|------------|
| `root`       | `orangepi` |
| `orangepi`   | `orangepi` |

**Что такое SSH?** SSH — это способ управлять компьютером по сети, как будто вы сидите прямо за ним. Вы набираете команды на своём ноутбуке, а выполняются они на Orange Pi.

Подключитесь по SSH:
```bash
ssh orangepi@<IP-адрес-платы>
```

---

## 2. Аппаратные интерфейсы

### 2.1. Как работают overlays на Orange Pi

На Raspberry Pi интерфейсы включаются в `config.txt`:
```bash
dtoverlay=uart2
dtparam=spi=on
```

На Orange Pi Zero 3W — через файл `/boot/orangepiEnv.txt`:
```bash
overlays=uart2 spi3-cs0-cs1-spidev i2c1
```

Несколько интерфейсов перечисляются через **пробел** в одной строке `overlays=`.

Посмотреть все доступные overlays для своей платы:
```bash
ls /boot/dtb/allwinner/overlay/sun60i-a733-*.dtbo | sed 's/.*sun60i-a733-//;s/\.dtbo//'
```

**Таблица доступных интерфейсов:**

| Интерфейс | Overlay                  | Устройство          | Пины (40-pin)          |
|-----------|--------------------------|---------------------|------------------------|
| UART2     | `uart2`                  | `/dev/ttyS2`        | TX=Pin11, RX=Pin13     |
| UART6     | `uart6`                  | `/dev/ttyS6`        | TX=Pin24, RX=Pin23     |
| UART7     | `uart7`                  | `/dev/ttyS7`        | TX=Pin16, RX=Pin18     |
| UART8     | `uart8`                  | `/dev/ttyS8`        | нет                    |
| SPI3      | `spi3-cs0-cs1-spidev`   | `/dev/spidev3.0/.1` | нет                    |
| I2C0      | `i2c0`                   | `/dev/i2c-0`        | SDA=Pin3, SCL=Pin5     |
| I2C1      | `i2c1`                   | `/dev/i2c-1`        | SDA=Pin38, SCL=Pin40   |
| I2C2      | `i2c2`                   | `/dev/i2c-2`        | SDA=Pin19, SCL=Pin23   |
| I2C3      | `i2c3`                   | `/dev/i2c-3`        | SDA=Pin26, SCL=Pin21   |
| PWM       | `pwm4` / `pwm8` / `pwm9` | `/sys/class/pwm/`  | Pin37 / Pin35 / Pin40  |

---

### 2.2. Включение UART

UART нужен для подключения полётного контроллера.

#### Физическое подключение (UART2)

- TX устройства → Pin 13 (RX платы)
- RX устройства → Pin 11 (TX платы)
- GND устройства → GND платы

> **Примечание** TX одного устройства всегда подключается к RX другого, и наоборот.

#### Способ 1: через orangepi-config

```bash
sudo orangepi-config
```

1. Выберите **System -> Hardware**
2. Стрелками найдите нужный UART (например, `uart2`), выберите его **пробелом**
3. **Save -> Back -> Reboot**

#### Способ 2: вручную

```bash
sudo nano /boot/orangepiEnv.txt
```

Найдите строку `overlays=` и добавьте нужный UART:
```bash
overlays=uart2
```

Если строки нет — добавьте в конец файла. Если уже есть другие — допиши через пробел:
```bash
overlays=uart2 spi3-cs0-cs1-spidev
```

Перезагрузитесь:
```bash
sudo reboot
```

#### Проверка

```bash
ls /dev/ttyS*
# Ожидается: /dev/ttyS0  /dev/ttyS2
```

Тест loopback (замкни TX и RX пины проводом, затем выполните):
```bash
gpio serial /dev/ttyS2
# Ожидается: Out: 0: -> 0, Out: 1: -> 1, ...
```

---

### 2.3. Включение SPI

#### Способ 1: через orangepi-config

```bash
sudo orangepi-config
```

1. **System -> Hardware**
2. Отметьте `spi3-cs0-cs1-spidev` пробелом
3. **Save -> Back -> Reboot**

#### Способ 2: вручную

```bash
sudo nano /boot/orangepiEnv.txt
```

Добавьте в строку `overlays=`:
```bash
overlays=spi3-cs0-cs1-spidev
```

Перезагрузитесь и проверьте:
```bash
ls /dev/spidev*
# Ожидается: /dev/spidev3.0  /dev/spidev3.1
```

---

### 2.4. Включение I2C

#### Способ 1: через orangepi-config

```bash
sudo orangepi-config
```

1. **System -> Hardware**
2. Отметьте нужный I2C, например `i2c1`
3. **Save -> Back -> Reboot**

#### Способ 2: вручную

```bash
sudo nano /boot/orangepiEnv.txt
```

Добавьте:
```bash
overlays=i2c1
```

Проверьте:
```bash
ls /dev/i2c-*
# Ожидается: /dev/i2c-1

sudo i2cdetect -y -r 1
# Покажет таблицу адресов подключённых I2C устройств
```

---

### 2.5. Включение PWM

**Что такое PWM?** Широтно-импульсная модуляция — это способ управлять яркостью светодиодов, скоростью моторов и другими устройствами, быстро включая и выключая сигнал.

#### Способ 1: через orangepi-config

```bash
sudo orangepi-config
```

1. **System -> Hardware**
2. Отметьте нужный PWM (например, `pwm4` для Pin 37)
3. **Save -> Back -> Reboot**

#### Способ 2: вручную

```bash
sudo nano /boot/orangepiEnv.txt
```

Добавьте:
```bash
overlays=pwm4
```

#### Ручное управление PWM (тест)

Выполняйте от root:
```bash
sudo su

# Открыть канал PWM
echo 0 > /sys/class/pwm/pwmchip0/export

# Установить период 20 мс (50 Гц)
echo 20000000 > /sys/class/pwm/pwmchip0/pwm0/period

# Установить скважность 1 мс (5%)
echo 1000000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle

# Включить PWM
echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable
```

- `period` — полный период сигнала в наносекундах (20 000 000 нс = 20 мс = 50 Гц)
- `duty_cycle` — длительность высокого уровня (должна быть меньше или равна period)
- `enable` — 1 включает, 0 выключает

---

### 2.6. Включение нескольких интерфейсов одновременно

Все нужные overlays перечисляются через пробел в одной строке:

```bash
sudo nano /boot/orangepiEnv.txt
```

Пример для UART2 + SPI3 + I2C1 + PWM4:
```bash
overlays=uart2 spi3-cs0-cs1-spidev i2c1 pwm4
```

После каждого изменения файла нужна перезагрузка. Изменения не применяются сразу.

---

## 3. Камера MIPI CSI (IMX219)

### 3.1. Физическое подключение

На Orange Pi Zero 3W есть два разъёма камеры: **CAM1** и **CAM2** (22-пиновые, шаг 0.5 мм).

1. Обесточьте плату перед подключением.
2. Используйте переходной шлейф **22PIN -> 15PIN** (входит в комплект камеры или покупается отдельно).
3. Вставьте шлейф в разъём CAM1 контактами к плате, защёлкни фиксатор.
4. Убедитесь, что шлейф не перекручен.

CAM1 даёт устройство `/dev/video0`, CAM2 — `/dev/video8`.

### 3.2. Настройка камеры

В отличие от Raspberry Pi, на Orange Pi Zero 3W камера IMX219 работает без device tree overlay. Драйвер `vin_v4l2` встроен в ядро.

#### Шаг 1: Загрузите драйвер

```bash
sudo modprobe vin_v4l2
```

#### Шаг 2: Проверьте, что устройства появились

```bash
ls /dev/video*
# Ожидается: /dev/video0  /dev/video8
```

#### Шаг 3: Добавьте автозагрузку драйвера

```bash
echo "vin_v4l2" | sudo tee -a /etc/modules
```

Теперь драйвер будет загружаться автоматически при каждом включении платы.

#### Шаг 4: Тест камеры

```bash
# Встроенный программа (открывает превью при подключённом мониторе)
test_camera.sh /dev/video0    # CAM1
test_camera.sh /dev/video8    # CAM2
```

Для работы без монитора:
```bash
sudo apt install v4l-utils fswebcam

# Информация о камере
v4l2-ctl --device=/dev/video0 --list-formats-ext

# Захват кадра
sudo fswebcam -d /dev/video0 --no-banner -r 1920x1080 ./image.jpg
```

#### Шаг 5: Использование OpenCV в Python

```bash
sudo apt install python3-opencv libopencv-dev python3-pip
sudo apt install python3-pybind11 python3-dev
```

Пример кода:
```python
import cv2

cap = cv2.VideoCapture('/dev/video0')
ret, frame = cap.read()
if ret:
    cv2.imwrite('frame.jpg', frame)
cap.release()
```

---

## 4. Режим максимальной производительности CPU

По умолчанию все ядра работают в режиме `schedutil` — частота меняется автоматически по нагрузке. Для стабильной производительности нужен режим `performance`, при котором процессор всегда работает на максимальной частоте.

**Топология процессора:**

| Ядра   | Тип        | Максимальная частота |
|--------|------------|----------------------|
| cpu0–5 | Cortex-A55 | 1794 МГц             |
| cpu6–7 | Cortex-A76 | 2002 МГц             |

### 4.1. Временное переключение (до перезагрузки)

```bash
# A55 кластер (cpu0-5)
for cpu in 0 1 2 3 4 5; do
  echo performance | sudo tee /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
done

# A76 кластер (cpu6-7) — сначала governor, потом max_freq
for cpu in 6 7; do
  echo performance | sudo tee /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
  echo 2002000 | sudo tee /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_max_freq
done
```

> **Примечание** Для A76 нужно сначала выставить governor, и только потом изменять `scaling_max_freq`. Смена governor сбрасывает max_freq обратно.

### 4.2. Постоянная настройка через systemd

Чтобы режим performance включался автоматически при каждом запуске, создадим сервис.

#### Шаг 1: Создайте программа

```bash
sudo nano /usr/local/bin/cpu-performance.sh
```

Содержимое:
```bash
#!/bin/bash
for cpu in 0 1 2 3 4 5; do
    if [[ -f /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor ]]; then
        echo performance > /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
    fi
done

for cpu in 6 7; do
    if [[ -f /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor ]]; then
        echo performance > /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_governor
        echo 2002000     > /sys/devices/system/cpu/cpu${cpu}/cpufreq/scaling_max_freq
    fi
done

exit 0
```

Сделайте программа исполняемым:
```bash
sudo chmod +x /usr/local/bin/cpu-performance.sh
```

#### Шаг 2: Создайте сервис systemd

```bash
sudo nano /etc/systemd/system/cpu-performance.service
```

Содержимое:
```ini
[Unit]
Description=Orange Pi Zero 3W - CPU Performance Mode
After=multi-user.target
DefaultDependencies=no

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/cpu-performance.sh

[Install]
WantedBy=multi-user.target
```

#### Шаг 3: Включите и запустите сервис

```bash
sudo systemctl daemon-reload
sudo systemctl enable cpu-performance.service
sudo systemctl start cpu-performance.service
```

#### Шаг 4: Проверьте

```bash
systemctl status cpu-performance.service
# Ожидается: Active: active (exited)

for cpu in /sys/devices/system/cpu/cpu*/cpufreq; do
  echo "$cpu: $(cat $cpu/scaling_governor) @ $(( $(cat $cpu/scaling_max_freq) / 1000 )) MHz"
done
```

---

## 5. Вентилятор охлаждения

Вентилятор подключается к разъёму **FAN** (2-пиновый, 0.8 мм). Система автоматически управляет им по температуре, поэтому он может останавливаться при низкой нагрузке.

### 5.1. Принудительно включить на максимум (временно)

```bash
# Отключить термальное управление
sudo bash -c 'echo disabled > /sys/class/thermal/thermal_zone0/mode'
sudo bash -c 'echo disabled > /sys/class/thermal/thermal_zone3/mode'

# Установить максимальную скорость (4/4)
sudo bash -c 'echo 4 > /sys/class/thermal/cooling_device9/cur_state'
```

Проверить состояние:
```bash
cat /sys/class/thermal/cooling_device9/cur_state
# Ожидается: 4
```

### 5.2. Постоянная настройка через systemd

```bash
sudo nano /etc/systemd/system/fan-always-on.service
```

Содержимое:
```ini
[Unit]
Description=Orange Pi Zero 3W - Fan always on at maximum speed
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '\
  echo disabled > /sys/class/thermal/thermal_zone0/mode; \
  echo disabled > /sys/class/thermal/thermal_zone3/mode; \
  echo 4 > /sys/class/thermal/cooling_device9/cur_state'

[Install]
WantedBy=multi-user.target
```

Включите и запустите:
```bash
sudo systemctl daemon-reload
sudo systemctl enable fan-always-on.service
sudo systemctl start fan-always-on.service
```

Проверьте:
```bash
systemctl status fan-always-on.service
cat /sys/class/thermal/cooling_device9/cur_state
# Ожидается: 4
```

### 5.3. Управление сервисом

```bash
systemctl stop    fan-always-on    # выключить вентилятор
systemctl start   fan-always-on    # включить вентилятор
systemctl disable fan-always-on    # убрать из автозапуска
```

### 5.4. Вернуть автоматическое управление

```bash
sudo systemctl disable fan-always-on
sudo systemctl stop fan-always-on
sudo bash -c 'echo enabled > /sys/class/thermal/thermal_zone0/mode'
sudo bash -c 'echo enabled > /sys/class/thermal/thermal_zone3/mode'
```

---

## 6. Установка Docker

```bash
sudo apt update
sudo apt install ca-certificates curl
```

Добавьте GPG-ключ:
```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg \
    -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

Добавьте репозиторий:
```bash
sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null << EOF
Types: deb
URIs: https://download.docker.com/linux/debian
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF
```

Установите Docker:
```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io \
    docker-buildx-plugin docker-compose-plugin
sudo systemctl enable docker
sudo systemctl start docker
```

Проверьте:
```bash
sudo systemctl status docker
docker --version
```

Настройте права:
```bash
sudo usermod -aG docker $USER
sudo reboot
```

Документация: [Официальная инструкция по установке Docker](https://docs.docker.com/engine/install/debian/)

---

## 7. GPIO, I2C и SPI инструменты

### Установка пакетов

```bash
sudo apt update
sudo apt install gpiod libgpiod-dev python3-libgpiod
sudo apt install i2c-tools libi2c-dev
sudo apt install v4l-utils
```

### Добавьте пользователя в системные группы

```bash
sudo usermod -aG docker   $USER
sudo usermod -aG dialout  $USER
sudo usermod -aG tty      $USER
sudo usermod -aG i2c      $USER
sudo usermod -aG video    $USER
sudo usermod -aG gpio     $USER
sudo usermod -aG spi      $USER
```

Изменения применяются после перезагрузки или повторного входа в систему.

### Проверка GPIO

```bash
gpiodetect                        # GPIO контроллеры
gpioinfo                          # состояние всех GPIO линий
gpioget gpiochip0 <номер_линии>   # прочитать пин
gpioset gpiochip0 <номер_линии>=1 # установить пин
```

### Проверка I2C

```bash
# Сканировать шину I2C 1
sudo i2cdetect -y -r 1

# Прочитать регистр устройства (адрес 0x48, регистр 0x00)
sudo i2cget -y 1 0x48 0x00

# Записать в регистр
sudo i2cset -y 1 0x48 0x01 0xFF
```

---

## 8. Настройка SSH на порт 2222

Если нужно освободить порт 22 для Docker-контейнера:

```bash
sudo nano /etc/ssh/sshd_config
```

Найдите строку `#Port 22` и замените:
```bash
Port 2222
```

Перезапустите SSH:
```bash
sudo systemctl restart ssh
```

Откройте порт в файрволе (если включён ufw):
```bash
sudo ufw allow 2222/tcp
sudo ufw reload
```

Подключение после смены порта:
```bash
ssh -p 2222 orangepi@<IP-адрес-платы>
```

> **Внимание** Не закрывайте текущее SSH-соединение до проверки нового подключения по порту 2222. Если что-то пойдёт не так, вы потеряете доступ к плате.

---

## 9. Подключение к Wi-Fi через nmcli

`nmcli` — это утилита управления сетью в командной строке.

Просмотр доступных сетей:
```bash
sudo nmcli device wifi list
```

Подключение к сети:
```bash
sudo nmcli connection add type wifi con-name "Router" \
  ifname <имя_интерфейса> ssid "Router_name" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "Router_password" \
  connection.autoconnect yes
```

Создание нескольких профилей с приоритетами:
```bash
# Домашняя сеть (высший приоритет)
sudo nmcli connection add type wifi con-name "Wifi" \
  ifname <имя_интерфейса> ssid "Wifi" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "11111111" \
  connection.autoconnect yes connection.autoconnect-priority 100

# Сеть "Poletka" (средний приоритет)
sudo nmcli connection add type wifi con-name "Poletka" \
  ifname <имя_интерфейса> ssid "Poletka" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "sosatusa" \
  connection.autoconnect yes connection.autoconnect-priority 50
```

Управление профилями:
```bash
nmcli connection show             # список профилей
sudo nmcli connection up "Имя"    # активировать
sudo nmcli connection down "Имя"  # деактивировать
sudo nmcli connection delete "Имя" # удалить
```

---

## 10. Мониторинг системы

### Температуры

```bash
for zone in /sys/class/thermal/thermal_zone*; do
  type=$(cat $zone/type)
  temp=$(cat $zone/temp)
  echo "$type: $(( temp / 1000 )).$(( (temp % 1000) / 100 ))°C"
done
```

Нормальные значения:
- `cpub_thermal_zone` ниже 70°C — нормально
- `cpub_thermal_zone` выше 85°C — процессор начинает снижать частоту

### Частоты CPU в реальном времени

```bash
watch -n 1 'for cpu in /sys/devices/system/cpu/cpu*/cpufreq; do
  echo "$cpu: $(cat $cpu/scaling_governor) @ $(cat $cpu/scaling_cur_freq) Hz"
done'
```

### Нагрузочный тест CPU

```bash
sudo apt install stress-ng

# Нагрузить все 8 ядер на 30 секунд
stress-ng --cpu 8 --timeout 30s
```

### Проверка текущих настроек

```bash
cat /boot/orangepiEnv.txt

ls /dev/ttyS*    # UART
ls /dev/spidev*  # SPI
ls /dev/i2c-*    # I2C
ls /dev/video*   # Камера

systemctl status fan-always-on cpu-performance
```

---

## 11. Сравнение с Raspberry Pi

| Задача                    | Raspberry Pi                        | Orange Pi Zero 3W                        |
|---------------------------|-------------------------------------|------------------------------------------|
| Файл конфигурации         | `/boot/firmware/config.txt`         | `/boot/orangepiEnv.txt`                  |
| Включение UART            | `enable_uart=1` + `dtoverlay=uart2` | `overlays=uart2`                         |
| Включение SPI             | `dtoverlay=spi1-1cs`                | `overlays=spi3-cs0-cs1-spidev`           |
| Включение I2C             | `dtparam=i2c_arm=on`                | `overlays=i2c1`                          |
| Камера IMX219             | `dtoverlay=imx219,cam0`             | `modprobe vin_v4l2` (overlay не нужен)   |
| Несколько интерфейсов     | Отдельные строки `dtoverlay=`       | Один `overlays=` через пробел            |
| Утилита настройки         | `raspi-config`                      | `orangepi-config`                        |
| Управление вентилятором   | Нет встроенного                     | `cooling_device9` через thermal sysfs    |

---

## 12. Восстановление системы

### Безопасное выключение

```bash
sudo shutdown -h now    # выключить
sudo reboot             # перезагрузить
```

Всегда используйте программное выключение вместо прямого отключения питания. Это защитит SD-карту от повреждений.

### Восстановление файла orangepiEnv.txt

Резервные копии создаются автоматически программой `setup.sh`:
```bash
ls /boot/orangepiEnv.txt.backup-*
```

Восстановить из резервной копии:
```bash
sudo cp /boot/orangepiEnv.txt.backup-20250609-120000 /boot/orangepiEnv.txt
sudo reboot
```

---

*Orange Pi Zero 3W | Allwinner A733 | kernel 6.6.98-sun60iw2*
