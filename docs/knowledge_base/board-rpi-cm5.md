# Raspberry Pi CM5 с платой Waveshare CM5-NANO-A/B

Этот раздел описывает настройку бортового компьютера на базе Raspberry Pi Compute Module 5: установить операционную систему, подключить камеру, настроить Docker и соединить с полётным контроллером.

---

## 1. Установка операционной системы

### 1.1. Какой образ выбрать

Для работы с камерой Raspberry Pi рекомендуется использовать:

**Raspberry Pi OS (Legacy) Lite**
- Основа: Debian Bookworm
- Без графического интерфейса (Lite — значит только командная строка)
- Размер: 422 МБ (64-bit)
- Ядро: версии 6.12

> **Внимание** Ubuntu 24.04 Server не поддерживает камеру Raspberry Pi в полном объёме. Если нужна камера — используйте Raspberry Pi OS.

Скачать образ можно здесь:
- [Официальные образы Raspberry Pi OS](https://www.raspberrypi.com/software/operating-systems/)
- [Документация по Waveshare CM5-NANO-A/B](https://www.waveshare.com/wiki/CM5-NANO-A/B)

---

## 2. Настройка камеры (Raspberry Pi OS)

> **Примечание** Что такое сенсор камеры. Внутри любой камеры есть светочувствительный чип — сенсор. У разных камер разные сенсоры. Нужно знать, какой сенсор в вашей камере, чтобы загрузить правильный драйвер.
>
> **Примечание** Поддерживаются: `ov5647` и `imx219`
> - Rpi Camera (G) = `ov5647`
> - Raspberry Pi Camera Module 2 = `imx219`

### 2.1. Физическое подключение камеры

Сначала физически подключите камеру к плате. Делайте это при выключенном питании.

1. Полностью обесточьте плату (отключите питание).
2. На плате Waveshare CM5-NANO-A/B найдите разъёмы **CAM0/DSI0** и **CAM1/DSI1**. Вам нужен **CAM0/DSI0** — он ближе к углу платы (схема в [документации Waveshare](https://www.waveshare.com/wiki/CM5-NANO-A/B#Hardware)).
3. Подключите шлейф камеры к разъёму CAM0/DSI0:
   - Вставляйте шлейф контактами (металлическими дорожками) в сторону платы.
   - Вставьте до упора и аккуратно защёлкни фиксатор.
4. Убедитесь, что шлейф не перекручен и не повреждён.

> **Примечание** Если после всех настроек камера не определяется — чаще всего это проблема физического контакта. Попробуйте переподключить шлейф или заменить его.

### 2.2. Настройка файла конфигурации

**Что такое config.txt?** Это файл настроек, который Raspberry Pi читает при каждом включении — до загрузки Linux. Через него включаются и выключаются различные аппаратные функции.

#### Шаг 1: Откройте файл конфигурации

```bash
sudo nano /boot/firmware/config.txt
```

#### Шаг 2: Добавьте настройки камеры

Найдите или создайте секцию `[cm5]` и добавьте строки.

Для сенсора `ov5647`:
```bash
[cm5]
dtoverlay=dwc2,dr_mode=host
dtoverlay=ov5647,cam0
start_x=1
gpu_mem=256
```

Для сенсора `imx219`:
```bash
[cm5]
dtoverlay=dwc2,dr_mode=host
dtoverlay=imx219,cam0
start_x=1
gpu_mem=256
```

Для `imx219` также добавьте строку в секцию `[all]` (или в начало файла):
```bash
camera_auto_detect=0
```

Для постоянного максимального вентилятора Raspberry Pi 5/CM5 добавь строки в секцию `[all]`:
```bash
[all]
dtparam=fan_temp0=1000
dtparam=fan_temp0_speed=255
dtparam=fan_temp1_speed=255
dtparam=fan_temp2_speed=255
dtparam=fan_temp3_speed=255
```

`fan_temp0=1000` включает первую ступень уже при 1°C, а все ступени скорости выставлены в `255`, то есть на максимальный PWM.

Что означают эти строки:
- `dtoverlay=dwc2,dr_mode=host` — включает USB-порт в режиме хоста
- `dtoverlay=...,cam0` — загружает драйвер для камеры
- `start_x=1` — включает поддержку камеры
- `gpu_mem=256` — выделяет 256 МБ памяти для обработки видео

#### Шаг 3: Перезагрузите систему

```bash
sudo reboot
```

#### Шаг 4: Установите нужные пакеты

```bash
sudo apt update
sudo apt install libcamera-apps
```

#### Шаг 5: Проверьте, работает ли камера

```bash
rpicam-hello
```

Откроется окно предварительного просмотра на 5 секунд. Если видите картинку — камера работает.

```bash
rpicam-hello --list-cameras
```

Покажет список камер, максимальное разрешение и FPS.

Для бесконечного просмотра:
```bash
rpicam-hello --timeout 0
```

Дополнительная информация:
- [Документация по ПО камеры Raspberry Pi](https://www.raspberrypi.com/documentation/computers/camera_software.html)

---

## 3. Установка Docker

**Что такое Docker?** Docker — это изолированная среда, в которой собраны программа и все её зависимости. Программа в Docker-контейнере работает одинаково на любом компьютере, независимо от того, что на нём установлено. ROS 2 на Обрике запущен именно в Docker-контейнере.

### Шаг 1: Подготовка системы

```bash
sudo apt update
sudo apt install ca-certificates curl
```

### Шаг 2: Добавьте GPG-ключ Docker

Это цифровая подпись, которая гарантирует, что вы скачиваете настоящий Docker, а не поддельный.

```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

### Шаг 3: Добавьте репозиторий Docker

```bash
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/debian
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF
```

### Шаг 4: Установите Docker

```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl start docker
```

### Шаг 5: Проверьте, что Docker работает

```bash
sudo systemctl status docker
```

Строка `active (running)` означает, что Docker запущен.

### Шаг 6: Настройте права доступа

Чтобы запускать Docker без sudo:

```bash
sudo usermod -aG docker $USER
sudo reboot
```

Документация: [Официальная инструкция по установке Docker](https://docs.docker.com/engine/install/debian/)

---

## 4. Подключение полётного контроллера (Matek H743 Mini V3)

### 4.1. Физическое подключение

Полётный контроллер и бортовой компьютер соединяются по интерфейсу UART (последовательная передача данных).

Схема подключения:
- **RX4** (Matek) → **GPIO14** (CM5-NANO-A/B)
- **TX4** (Matek) → **GPIO15** (CM5-NANO-A/B)
- **GND** (Matek) → **GND** (CM5-NANO-A/B)

### 4.2. Добавьте пользователя в нужные группы

Группы в Linux — это способ управлять доступом к устройствам. Чтобы программы могли работать с UART, камерой и GPIO без прав администратора, добавьте своего пользователя в соответствующие группы:

```bash
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER
sudo usermod -aG i2c $USER
sudo usermod -aG video $USER
sudo usermod -aG gpio $USER
```

- `dialout`, `tty` — доступ к последовательным портам (UART)
- `i2c` — доступ к шине I2C для дополнительных сенсоров
- `video` — доступ к камере
- `gpio` — доступ к GPIO-пинам

### 4.3. Установите инструменты для работы с GPIO

```bash
sudo apt update
sudo apt install gpiod libgpiod-dev python3-libgpiod
```

### 4.4. Проверьте доступ к GPIO

```bash
sudo gpiodetect
sudo gpioinfo
```

Команды покажут доступные GPIO-чипы и их состояние.

### 4.5. Включите UART

Для Raspberry Pi OS добавьте в `/boot/firmware/config.txt`:
```bash
enable_uart=1
```

Для Ubuntu 24.04 Server:
```bash
enable_uart=1
dtoverlay=uart4
dtoverlay=uart3
dtoverlay=uart2
```

После изменения конфигурации перезагрузите систему.

### 4.6. Отключите консольный UART [только Raspberry Pi OS]

По умолчанию UART используется как консоль для отладки. Нам нужно переключить его в режим связи с полётным контроллером:

```bash
sudo raspi-config
# -> Interface Options -> Serial Port
# -> Would you like a login shell...? -> NO
# -> Would you like the serial port...? -> YES
```

---

## 5. Подключение к Wi-Fi через nmcli

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

## 6. Настройка SSH на порт 2222

**Что такое SSH?** SSH — это способ управлять компьютером дрона с ноутбука по сети, как будто вы сидите прямо за ним. Вы набираете команды на своём ноутбуке, а выполняются они на Обрике.

По умолчанию SSH работает на порту 22. Нам нужно перенести SSH хоста на порт 2222, чтобы порт 22 мог занять Docker-контейнер. Тогда:
- Подключение к контейнеру: `ssh sverk@DRONE_IP` (порт 22)
- Подключение к Raspberry Pi: `ssh -p 2222 pi@DRONE_IP` (порт 2222)

### Шаг 1: Отредактируйте конфигурацию SSH

```bash
sudo nano /etc/ssh/sshd_config
```

Найдите строку `#Port 22` и замените на:
```bash
Port 2222
```

### Шаг 2: Перезапустите SSH

```bash
sudo systemctl restart ssh
```

На некоторых системах сервис называется `sshd`:
```bash
sudo systemctl restart sshd
```

### Шаг 3: Откройте порт в файрволе (если включён ufw)

```bash
sudo ufw allow 2222/tcp
sudo ufw reload
```

---

## 7. Настройка SPI1 для адресной LED-ленты (пин 38)

Если вы хотите подключить адресную LED-ленту WS2812, используйте SPI1.

### Физическое подключение WS2812

1. Обесточьте плату.
2. Подключите ленту к 40-пиновому разъёму:
   - **DIN** (Data In) ленты → **физический пин 38** (GPIO20, SPI1 MOSI)
   - **GND** ленты → **GND** (например, пин 39)
   - **5V** ленты → внешний источник питания 5В (общая земля с платой обязательна)

### Включите SPI0 (если ещё не включён)

```bash
sudo raspi-config
# -> Interfacing Options -> SPI -> Enable
```

### Включите SPI1 в config.txt

```bash
sudo nano /boot/firmware/config.txt
```

Добавьте строку (в секцию `[all]` или в конец файла):
```bash
dtoverlay=spi1-1cs
```

Это создаёт устройство `/dev/spidev1.0`.

### Добавьте пользователя в группу spidev

```bash
sudo usermod -aG spi $USER
```

### Перезагрузите и проверьте

```bash
sudo reboot
```

После загрузки проверьте, что устройство появилось:
```bash
ls -la /dev/spidev1.0
```

Если файл есть — SPI1 работает.

---

## 8. Системное администрирование

### Мониторинг температуры CPU

Создайте файл `temp.sh`:
```bash
nano temp.sh
```

Вставьте в него:
```bash
#!/bin/bash
cpuTemp0=$(cat /sys/class/thermal/thermal_zone0/temp)
cpuTemp1=$(($cpuTemp0/1000))
cpuTemp2=$(($cpuTemp0/100))
cpuTempM=$(($cpuTemp2 % $cpuTemp1))

echo CPU Temp: $cpuTemp1"."$cpuTempM"°C"
```

Запустите:
```bash
bash temp.sh
```

### Сетевые настройки (Ubuntu)

Если сетевой интерфейс не включается автоматически:
```bash
sudo ip link set eth0 up
sudo dhclient -v eth0
```

### Безопасное выключение и перезагрузка

```bash
sudo shutdown -h now   # выключить
sudo reboot            # перезагрузить
```

Всегда используйте эти команды вместо прямого отключения питания: резкое обесточивание может испортить файловую систему на SD-карте.

### Восстановление файловой системы

Если при загрузке появляются сообщения об ошибках файловой системы (режим initramfs):
```bash
fsck -y /dev/sda1 ; reboot -f
```

> **Примечание** Выполняйте эту команду только из режима восстановления (initramfs). В обычном режиме работы сначала отмонтируйте раздел.

[Подробное руководство по восстановлению файловой системы](https://ru.stackoverflow.com/questions/765130/)

---

## 9. Настройка производительности (Raspberry Pi OS)

Чтобы компьютер не перегревался и работал стабильно, добавьте ограничения частоты процессора.

```bash
sudo nano /boot/firmware/config.txt
```

Добавьте или отредактируйте секцию `[all]`:
```bash
[all]
arm_freq=1500
arm_freq_min=600
temp_soft_limit=65000
```

Перезагрузите систему:
```bash
sudo reboot
```
