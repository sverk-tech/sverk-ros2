# Horizon RDK X3 с платой Waveshare CM4-NANO-A/B/C

Этот раздел описывает настройку бортового компьютера на базе Horizon RDK X3: установите операционную систему, Docker, исправите настройки cgroups для Docker на старом ядре, подключите полётный контроллер и запустите стек sverk-ros2 в контейнере.

**Официальная документация D-Robotics (RDK X3):** [RDK DOC](https://developer.d-robotics.cc/rdk_doc/en/)

---

## Что такое BPU в RDK X3

RDK X3 основан на процессоре Sunrise X3. Его особенность — встроенный BPU (Brain Processing Unit), специальный блок для нейросетевых вычислений. Это значит, что X3 умеет быстро обрабатывать нейросети прямо на борту дрона без подключения к интернету.

---

## 1. Образ ОС и запись на карту

Для полноценной работы рекомендуется **Ubuntu 22.04 Server** из линейки образов RDK X3.

Источники:
- [Архив образов RDK X3](https://archive.d-robotics.cc/downloads/os_images/rdk_x3/rdk_os_3.0.3-2025-09-08/rdk-x3-ubuntu22-preinstalled-server-3.0.3-arm64.img.xz) (пример версии)
- [Waveshare CM4-NANO-A](https://www.waveshare.com/wiki/CM4-NANO-A)
- [Установка ОС RDK X3](https://developer.d-robotics.cc/rdk_doc/en/Quick_start/install_os/rdk_x3_module/system_burn)

---

## 2. Первичная настройка на хосте RDK

### 2.1. Вход в систему и переключение на SD-карту

Для первого входа обычно используется пользователь **`sunrise`** с паролем **`sunrise`** (смотрите выпускные заметки к своему образу).

Переключение загрузки с eMMC на SD-карту ([документация](https://developer.d-robotics.cc/rdk_doc/en/Quick_start/install_os/rdk_x3_module/boot_system)):

```bash
sudo parted /dev/mmcblk0 set 2 boot off
```

### 2.2. Обновление системы

Перед установкой Docker и запуском tros-пакетов обновите систему:

```bash
sudo apt update && sudo apt upgrade -y
```

Это часто устраняет конфликты с устаревшими зависимостями после свежей прошивки образа.

Рекомендация D-Robotics: [FAQ по устранению проблем с tros](https://developer.d-robotics.cc/rdk_doc/en/FAQ/tros_ros?_highlight=ros#q1-what-are-the-recommended-troubleshooting-steps-if-tros-related-packages-fail-to-run)

### 2.3. Установка Docker

```bash
sudo apt update
sudo apt install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
```

Проверьте:
```bash
sudo systemctl status docker
```

Ожидается статус `active (running)`.

Настройте права:
```bash
sudo usermod -aG docker "$USER"
```

После добавления в группу нужно выйти из сессии и войти снова (или перезагрузиться).

Справка: [Установка Docker Engine на Ubuntu](https://docs.docker.com/engine/install/ubuntu/)

### 2.4. Исправление проблемы с Docker и старым ядром (cgroups)

RDK X3 использует ядро **4.14.x**. На таких ядрах Docker может не запускать контейнеры из-за несовместимости с новым способом управления ресурсами (cgroups v2).

**Что такое cgroups?** Это механизм Linux, который контролирует, сколько процессора, памяти и других ресурсов может потратить каждая программа. Docker активно использует cgroups. Ядро 4.14 поддерживает только старую версию (cgroups v1), поэтому нужно явно сказать системе использовать именно её.

Загрузчик на RDK X3 — **U-Boot**. Параметры ядра хранятся в файле `/boot/boot.cmd`, из которого собирается `/boot/boot.scr`.

Сначала сделайте резервные копии:
```bash
sudo cp /boot/boot.cmd /boot/boot.cmd.bak
sudo cp /boot/boot.scr /boot/boot.scr.bak
```

Откройте файл для редактирования:
```bash
sudo nano /boot/boot.cmd
```

Найдите строку вида:
```text
setenv bootargs "console=tty1 console=ttyS0,921600 video=hobot:x3sdb-hdmi ${rootfs_args} ${flash_partitions}"
```

Добавьте параметр в конец строки (только внутри кавычек):
```text
setenv bootargs "console=tty1 console=ttyS0,921600 video=hobot:x3sdb-hdmi ${rootfs_args} ${flash_partitions} systemd.unified_cgroup_hierarchy=0"
```

Пересоберите `boot.scr`:
```bash
sudo mkimage -C none -A arm64 -T script -d /boot/boot.cmd /boot/boot.scr
```

Если команды `mkimage` нет:
```bash
sudo apt install -y u-boot-tools
sudo mkimage -C none -A arm64 -T script -d /boot/boot.cmd /boot/boot.scr
```

Перезагрузитесь и проверьте результат:
```bash
sudo reboot
```

После перезагрузки:
```bash
cat /proc/cmdline | tr ' ' '\n' | grep cgroup
stat -fc %T /sys/fs/cgroup/
sudo docker run --rm hello-world
```

### 2.5. Мониторинг CPU и BPU

```bash
sudo hrut_somstatus
```

Подробнее в [документации RDK DOC](https://developer.d-robotics.cc/rdk_doc/en/).

### 2.6. Подключение полётного контроллера (Matek H743 Mini V3)

#### Физическое подключение

Для платы Waveshare CM4-NANO-A/B/C (разъём 40-pin):

- Припаяйте провода к **RX4 / TX4** на Matek.
- Подключите:
  - **RX4** (Matek) → **GPIO14** (хост)
  - **TX4** (Matek) → **GPIO15** (хост)
  - **GND** (Matek) → **GND** (хост)

Точные пины UART уточняйте в Wiki Waveshare для своей платы.

#### Настройка групп пользователя

```bash
sudo usermod -aG dialout,tty,i2c,video,gpio "$USER"
```

- `dialout`, `tty` — последовательные порты (UART)
- `i2c` — шина I2C
- `video` — доступ к видеоподсистеме
- `gpio` — GPIO

Перелогинься после изменения групп.

### 2.7. Перенос SSH хоста на порт 2222

**Что такое SSH?** SSH — это способ управлять компьютером по сети, как будто вы сидите прямо за ним.

Нам нужно, чтобы порт 22 на IP дрона оставался для SSH внутри контейнера, а к хосту RDK подключаться по порту 2222.

```bash
sudo nano /etc/ssh/sshd_config
```

Укажите строку:
```text
Port 2222
```

Перезапустите SSH:
```bash
sudo systemctl restart ssh
```

Если сервис называется `sshd`:
```bash
sudo systemctl restart sshd
```

Если используется файрвол ufw:
```bash
sudo ufw allow 2222/tcp
sudo ufw reload
```

После настройки:

| Куда подключаться | Команда |
|-------------------|---------|
| Контейнер `sverk_ros2` | `ssh sverk@DRONE_IP` (порт 22) |
| Хост RDK | `ssh -p 2222 sunrise@DRONE_IP` |

### 2.8. SPI и адресная LED-лента (40-pin)

Официальная документация: [Using SPI (40-pin)](https://developer.d-robotics.cc/rdk_doc/en/Basic_Application/01_40pin_user_sample/spi)

Для параметров `led_control` смотрите файл `sverk_ws/src/sverk_drone/peripheral/led/led_control/rdk_x3_led_spi.md` в репозитории.

Краткая инструкция:

1. Проверьте доступные SPI-устройства:
   ```bash
   ls -l /dev/spidev*
   ```

2. При необходимости добавьте пользователя в группу `spi`:
   ```bash
   sudo usermod -aG spi "$USER"
   ```
   После смены группы нужна новая сессия или перезагрузка.

3. В `config/led_params_rdkx3.yaml` укажите `spi_bus` и `spi_device` в соответствии с именем устройства (например, `spidev0.0` → `spi_bus: 0`, `spi_device: 0`).

4. Тест SPI на образе RDK:
   ```bash
   python3 /app/40pin_samples/test_spi.py
   ```

---

## 3. Запуск sverk-ros2 в Docker на RDK X3

Для RDK X3 используется override `scripts/docker/docker-compose.rdkx3.yml` и образ на базе `scripts/docker/Dockerfile.rdk`.

### 3.1. Запуск контейнера

Из каталога репозитория на хосте RDK:

```bash
docker compose -f docker-compose.yml -f scripts/docker/docker-compose.rdkx3.yml up -d
```

Остановка:
```bash
docker compose -f docker-compose.yml -f scripts/docker/docker-compose.rdkx3.yml down
```

### 3.2. Настройка ROS 2 и Fast DDS

На RDK в контейнере при Fast DDS и shared memory часто возникают ошибки вида `RTPS_TRANSPORT_SHM` / `open_and_lock_file failed`. Из-за них топики видны в списке, но `ros2 topic hz` и `ros2 topic echo` не получают данные.

В `scripts/docker/docker-compose.rdkx3.yml` и `scripts/start.sh` задано использование только UDP-транспорта:
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`

Также задаются:
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DOMAIN_ID=0`
- `ROS_LOCALHOST_ONLY=0`

Проверьте с хоста после старта:
```bash
docker exec -it sverk_ros2 bash -lc 'source /opt/ros/humble/setup.bash && ros2 topic list'
```

Если нужно отладить вручную:
```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
ros2 daemon stop
```

> **Примечание** Значение должно быть `UDPv4`, без опечаток вроде `UDPv4v4`.

### 3.3. Структура файлов в контейнере RDK

- `/opt/tros` — монтируется с хоста (read-only), vendor-стек tros.b
- `/usr/lib/hbmedia`, `/usr/lib/hbbpu`, `/usr/lib/sensorlib` — аппаратные библиотеки X3
- `scripts/start.sh` — точка запуска: настраивает Fast DDS, загружает ROS 2, workspace и tros, затем запускает `ros2 launch launch_system full_system_real.launch.py`

---

## 4. Ссылки

| Тема | URL |
|------|-----|
| RDK DOC (корень) | https://developer.d-robotics.cc/rdk_doc/en/ |
| Запись ОС / модуль X3 | https://developer.d-robotics.cc/rdk_doc/en/Quick_start/install_os/rdk_x3_module/system_burn |
| Загрузка с SD / eMMC | https://developer.d-robotics.cc/rdk_doc/en/Quick_start/install_os/rdk_x3_module/boot_system |
| SPI 40-pin | https://developer.d-robotics.cc/rdk_doc/en/Basic_Application/01_40pin_user_sample/spi |
| Docker в Ubuntu | https://docs.docker.com/engine/install/ubuntu/ |

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