# Orange Pi 5 Pro (Rockchip RK3588S)

Этот раздел описывает настройку бортового компьютера на базе Orange Pi 5 Pro: установите операционную систему, подключите камеру, настроите Docker, UART, SPI и Wi-Fi.

---

## 1. Установка операционной системы

### 1.1. Скачайте образ

1. Перейдите на официальную страницу продукта: [Orange Pi 5 Pro](http://www.orangepi.org/html/hardWare/computerAndMicrocontrolers/details/Orange-Pi-5-Pro.html)
2. Перейдите во вкладку **Service and Support -> Download**
3. Скачайте образ **Orangepi5pro_X.X.X_debian_bookworm_server_linux6.X.XX.img.xz**
4. Официальная инструкция и дополнительные материалы: [Google Drive](https://drive.google.com/drive/folders/1j3gmf31XBuKPBeNIQOqqh9X_7SFCOv0s)

### 1.2. Запись образа

1. Отформатируйте карту microSD через **SD Memory Card Formatter** ([скачать](https://files.waveshare.com/upload/d/d7/Panasonic_SDFormatter.zip)).
2. Распакуйте образ:
   ```bash
   unxz Orangepi5pro_X.X.X_debian_bookworm_server_linux6.X.XX.img.xz
   ```
3. Запустите **balenaEtcher** от имени администратора.
4. Выберите распакованный файл `.img`, укажите карту microSD и нажмите Flash.
5. Дождитесь завершения записи и верификации.

### 1.3. Первый запуск

Вставьте карту в плату, подключите питание и сетевой кабель.

**Что такое SSH?** SSH — это способ управлять компьютером по сети, как будто вы сидите прямо за ним. Узнайте IP-адрес платы через интерфейс роутера, затем подключитесь:

```bash
ssh orangepi@<IP_АДРЕС_ПЛАТЫ>
```

**Логин:** `orangepi`, **Пароль:** `orangepi`

### 1.4. Обновление системы

```bash
sudo orangepi-config
```

В меню: **System -> System Update -> Yes**. После завершения перезагрузитесь:

```bash
sudo reboot
```

---

## 2. Настройка камеры

Поддерживаемые сенсоры:
- **IMX219** (Raspberry Pi Camera v2)
- **OV5647** (Raspberry Pi Camera v1)

### 2.1. Активация драйвера камеры

```bash
sudo orangepi-config
```

1. **System -> Hardware**
2. Найдите нужный сенсор:
   - Для IMX219: `imx219-camera-on-csi1`
   - Для OV5647: `ov5647-camera-on-csi1`
3. Нажмите **ПРОБЕЛ** для отметки `[*]`
4. **Save -> Exit -> Reboot**:
   ```bash
   sudo reboot
   ```

### 2.2. Установка инструментов

```bash
sudo apt-get update
sudo apt-get install -y v4l-utils gstreamer1.0-tools \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav ffmpeg
```

### 2.3. Проверка камеры

После перезагрузки камера доступна как `/dev/video11`.

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video11 --list-formats-ext
```

Запись тестового видео для IMX219 (640x480):
```bash
v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl exposure=2000
v4l2-ctl -d /dev/video11 --set-fmt-video=width=640,height=480,pixelformat=NV12
gst-launch-1.0 -q v4l2src device=/dev/video11 num-buffers=900 ! \
    video/x-raw,format=NV12,width=640,height=480 ! \
    videoconvert ! jpegenc ! avimux ! filesink location="test_imx219.avi"
```

Запись тестового видео для OV5647 (1920x1080):
```bash
gst-launch-1.0 v4l2src device=/dev/video11 io-mode=4 ! \
    videoconvert ! video/x-raw,format=NV12,width=1920,height=1080 ! \
    jpegenc ! avimux ! filesink location="test_ov5647.avi"
```

---

## 3. Установка Docker

**Что такое Docker?** Docker — это изолированная среда, в которой собраны программа и все её зависимости. ROS 2 на Обрике запущен именно в Docker-контейнере.

Orange Pi OS поставляется с предустановленным Docker, но он отключён по умолчанию.

### 3.1. Активация предустановленного Docker

```bash
enable_docker.sh
```

Проверьте:
```bash
docker run hello-world
```

### 3.2. Установка docker-compose-plugin

```bash
sudo apt update
sudo apt install -y docker-compose-plugin
```

### 3.3. Проверка

```bash
sudo systemctl status docker
```

Строка `active (running)` — Docker работает.

### 3.4. Настройка прав доступа

```bash
sudo usermod -aG docker $USER
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER
sudo usermod -aG i2c $USER
sudo usermod -aG video $USER
sudo usermod -aG gpio $USER
sudo reboot
```

---

## 4. Подключение полётного контроллера (Matek H743 Mini V3)

### 4.1. Физическое подключение

| Контакт на Orange Pi 5 Pro | Контакт на Matek H743 Mini V3 |
|----------------------------|-------------------------------|
| TXD (UART0_M2, Pin 36)     | RX4                           |
| RXD (UART0_M2, Pin 38)     | TX4                           |
| Любой GND                  | GND                           |

### 4.2. Активация UART

```bash
sudo orangepi-config
```

- **System -> Hardware**
- Найдите и выберите пробелом `uart0-m2`
- **Save -> Exit -> Reboot**

### 4.3. Проверка

```bash
ls -la /dev/ttyS0
sudo usermod -aG dialout $USER
```

Для применения изменений группы нужно переподключиться по SSH.

### 4.4. Тестирование связи (опционально)

```bash
sudo apt install minicom
sudo minicom -D /dev/ttyS0 -b 921600
```

---

## 5. Настройка SPI

SPI0 доступен на контактах 40-pin разъёма:

| Функция   | Pin |
|-----------|-----|
| SPI0 CLK  | 23  |
| SPI0 MOSI | 19  |
| SPI0 MISO | 21  |
| SPI0 CS0  | 24  |

### 5.1. Активация SPI

```bash
sudo orangepi-config
```

- **System -> Hardware**
- Найдите и выберите пробелом `spi0-m2-cs0-spidev`
- **Save -> Exit -> Reboot**

### 5.2. Проверка

```bash
ls /dev/spidev*
```

---

## 6. Настройка Wi-Fi

### 6.1. Просмотр доступных сетей

```bash
sudo nmcli device wifi list
```

### 6.2. Подключение к сети

```bash
sudo nmcli connection add type wifi con-name "Router" \
  ifname <имя_интерфейса> ssid "Router_name" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "Router_password" \
  connection.autoconnect yes
```

### 6.3. Создание нескольких профилей с приоритетами

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

# Сеть "Sverk_5G" (низкий приоритет)
sudo nmcli connection add type wifi con-name "Sverk_5G" \
  ifname <имя_интерфейса> ssid "Sverk_5G" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "avtonomkarulit" \
  connection.autoconnect yes connection.autoconnect-priority 10
```

### 6.4. Управление профилями

```bash
nmcli connection show                       # список профилей
sudo nmcli connection up "Имя_Профиля"      # активировать
sudo nmcli connection down "Имя_Профиля"    # деактивировать
sudo nmcli connection delete "Имя_Профиля"  # удалить
```

### 6.5. Графический интерфейс (альтернатива)

```bash
sudo nmtui
```

1. **Activate a connection**
2. Выберите сеть из списка
3. Введите пароль
4. Нажмите **OK**

---

## 7. Работа с GPIO

```bash
sudo apt install gpiod libgpiod-dev python3-libgpiod
```

Основные команды:
```bash
gpiodetect                           # список GPIO-чипов
gpioinfo                             # состояние всех GPIO линий
gpioget gpiochip0 <номер_линии>      # прочитать пин
gpioset gpiochip0 <номер_линии>=1    # установить пин
gpiomon gpiochip0 <номер_линии>      # отслеживать события на пине
```

---

## 8. Системное администрирование

### Мониторинг температуры

```bash
#!/bin/bash
cpu_temp_raw=$(cat /sys/class/thermal/thermal_zone0/temp)
cpu_temp_c=$(echo "scale=1; $cpu_temp_raw / 1000" | bc)
echo "CPU Temperature: $cpu_temp_c°C"
```

Сохраните как `temp.sh`, сделайте исполняемым и запустите:
```bash
chmod +x temp.sh
./temp.sh
```

### Базовые команды

- `sudo shutdown -h now` — безопасное выключение
- `sudo reboot` — перезагрузка
- `sudo systemctl status <имя_сервиса>` — состояние сервиса

Всегда выключайте через команду, а не прямым отключением питания — это предотвращает повреждение SD-карты.

---

## 9. Сборка пакетов ROS 2

```bash
# Редактирование launch-файла
nano sverk_ws/src/sverk_drone/main_package/launch_system/launch/full_system_real.launch.py

# Сборка нужных пакетов
colcon build --packages-select mipi_csi_cam_ros2 launch_system
```

---

## 10. Полезные ресурсы

- [Orange Pi 5 Pro — официальная страница](http://www.orangepi.org/html/hardWare/computerAndMicrocontrolers/details/Orange-Pi-5-Pro.html)
- [Orange Pi 5 Pro — загрузки и документация](https://drive.google.com/drive/folders/1j3gmf31XBuKPBeNIQOqqh9X_7SFCOv0s)
- [Orange Pi Wiki](http://www.orangepi.org/orangepiwiki/)
- [Форум Orange Pi](http://www.orangepi.org/orangepibbsen/)
