# Настройка системы на базе Radxa CM5 и Waveshare CM4-NANO-B/C

## Введение
Данная документация описывает полный процесс настройки системы на базе **Radxa Compute Module 5 (CM5)** с использованием платы-носителя **Waveshare CM4-NANO-B** или **CM4-NANO-C**. Руководство охватывает установку операционной системы, настройку камеры, развертывание Docker и подключение полетного контроллера через последовательный интерфейс.

---

## 1. Установка операционной системы

Перед началом настройки необходимо подготовить загрузочный носитель с операционной системой Radxa OS.

### 1.1. Подготовка загрузочного образа
Следуйте приведенным ниже шагам для записи образа на карту памяти.

1.  **Загрузка образа ОС**
    *   Посетите страницу загрузок Radxa: [https://docs.radxa.com/en/som/cm/cm5/download](https://docs.radxa.com/en/som/cm/cm5/download).
    *   Скачайте рекомендованный образ **Radxa OS (Debian Bookworm CLI)**. Пример актуальной ссылки:
        > `https://github.com/radxa-build/radxa-cm5-rpi-cm4-io/releases/download/rsdk-b3/radxa-cm5-rpi-cm4-io_bookworm_cli_b3.output.img.xz`

2.  **Подготовка карты памяти**
    *   Полностью отформатируйте карту microSD с помощью инструмента **SD Memory Card Formatter** ([скачать](https://files.waveshare.com/upload/d/d7/Panasonic_SDFormatter.zip)).
    *   Распакуйте скачанный сжатый образ в терминале:
        ```bash
        unxz radxa-cm5-rpi-cm4-io_bookworm_cli_b3.output.img.xz
        ```

3.  **Запись образа на носитель**
    *   Запустите **balenaEtcher** от имени администратора.
    *   Выберите распакованный файл (`.img`), укажите карту microSD в качестве целевого устройства и начните запись.
    *   Дождитесь завершения процесса и успешной верификации данных.

### 1.2. Первый запуск и базовая настройка
После записи образа выполните первоначальную настройку для удаленного доступа.

1.  **Включение SSH-доступа**
    *   Снова подключите записанную карту памяти к компьютеру.
    *   В корневом разделе (`boot`) найдите файл `before.txt` и откройте его для редактирования.
    *   Закомментируйте строки, отключающие SSH, и принудительно включите сервис:
        ```bash
        # disable_service ssh
        # disable_service ssh.socket
        # if headless enable_service ssh
        enable_service ssh
        ```

2.  **Подключение к плате**
    *   Вставьте карту в плату Waveshare, подключите питание и сетевой кабель.
    *   Определите IP-адрес платы в вашей локальной сети (через интерфейс роутера или утилиты для сканирования).
    *   Подключитесь к плате по SSH, используя учетные данные по умолчанию:
        ```bash
        ssh rock@<IP_АДРЕС_ПЛАТЫ>
        ```
        **Логин:** `rock`
        **Пароль:** `rock`

3.  **Обновление системы**
    *   После первого входа рекомендуется выполнить обновление системы для получения последних обновлений безопасности и драйверов.
        ```bash
        sudo rsetup
        ```
        В открывшемся меню утилиты:
        *   С помощью стрелок выберите **System -> System Update** и нажмите `Enter`.
        *   Подтвердите обновление, выбрав **Yes**.
        *   После завершения нажмите `Enter`, затем `Esc` для выхода.
    *   Примените обновления, перезагрузив систему:
        ```bash
        sudo reboot
        ```

---

## 2. Настройка камеры

**Внимание:** Настройка камеры на Radxa CM5 выполняется **исключительно** через официальную утилиту `rsetup`. Ручное редактирование конфигурационных файлов не поддерживается.

**Подтвержденные рабочие конфигурации**
*   Сенсор **IMX219** (встроен в плату **Waveshare CM4-NANO-C**)
*   Сенсор **OV5647** (например, модуль Raspberry Pi Camera (G) на плате **Waveshare CM4-NANO-B**)

### 2.1. Активация драйвера камеры
Активируйте необходимый драйвер (оверлей) через утилиту `rsetup`:

```bash
sudo rsetup
```

1.  В главном меню выберите: **Overlays -> (Yes) -> Manage overlays**.
2.  Найдите в списке конфигурацию, соответствующую вашему сенсору:
    *   Для **IMX219**: `Enable Raspberry Pi Camera V2 on CAM0`
    *   Для **OV5647**: `Enable Raspberry Pi Camera V1.3 on CAM0`
3.  Нажмите **ПРОБЕЛ**, чтобы отметить пункт галочкой `[*]`.
4.  Нажмите `Enter`, затем `Ok`.
5.  Выберите **Rebuild overlays** и подтвердите (`Yes`).
6.  Выйдите из меню клавишей `Esc` и перезагрузите систему:
    ```bash
    sudo reboot
    ```

### 2.2. Установка программных инструментов
Установите необходимые утилиты для работы с видео:

```bash
sudo apt-get update
sudo apt-get install -y v4l-utils gstreamer1.0-tools \
    gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly gstreamer1.0-libav ffmpeg
```

### 2.3. Проверка и использование камеры
После перезагрузки устройство камеры будет доступно как `/dev/video11`.

*   **Просмотр доступных режимов:**
    ```bash
    v4l2-ctl -d /dev/video11 --list-formats-ext
    ```

*   **Пример записи тестового видео:**
    *   **Для IMX219** (разрешение 640x480):
        ```bash
        v4l2-ctl -d /dev/v4l-subdev2 --set-ctrl exposure=2000
        v4l2-ctl -d /dev/video11 --set-fmt-video=width=640,height=480,pixelformat=NV12
        gst-launch-1.0 -q v4l2src device=/dev/video11 num-buffers=900 ! \
            video/x-raw,format=NV12,width=640,height=480 ! \
            videoconvert ! jpegenc ! avimux ! filesink location="test_imx219.avi"
        ```
    *   **Для OV5647** (разрешение 1920x1080):
        ```bash
        gst-launch-1.0 v4l2src device=/dev/video11 io-mode=4 ! \
            videoconvert ! video/x-raw,format=NV12,width=1920,height=1080 ! \
            jpegenc ! avimux ! filesink location="test_ov5647.avi"
        ```

**Поддерживаемые разрешения (частота ~21 FPS)**

| Сенсор IMX219(~21 FPS) | Сенсор OV5647(~15 FPS) |
|---------------|---------------|
| 640x480       | 640x480       |
| 800x600       | 800x600       |
| 1024x768      | 1024x768      |
| 1280x720      | 1280x720      |
| 1280x960      | 1280x960      |
| 1600x1200     | 1600x1200     |
| 1920x1080     | 1920x1080     |
| 2592x1944     | 2592x1944     |
| 3280x2464     | –             |

## 3. Установка Docker
### 3.1. Подготовка системы
```bash
# Обновление списка пакетов и установка необходимых зависимостей
sudo apt update
sudo apt install ca-certificates curl
```

### 3.2. Добавление GPG ключа Docker
```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

**Пояснение:**
- `install -m 0755 -d /etc/apt/keyrings` - создает директорию с правами 755
- `curl -fsSL` - загружает файл без вывода прогресса, проверяя SSL сертификат
- `chmod a+r` - делает ключ доступным для чтения всеми пользователями

### 3.3. Добавление репозитория Docker
```bash
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/debian  
Suites: $(. /etc/os-release && echo "$VERSION_CODENAME")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF
```

**Пояснение параметров:**
- `tee` - записывает вывод в файл и выводит на экран
- `$(. /etc/os-release && echo "$VERSION_CODENAME")` - автоматически определяет кодовое имя дистрибутива (bookworm для Raspberry Pi OS Legacy)
- `Signed-By` - указывает путь к GPG ключу для проверки подписи пакетов

### 3.4. Обновление пакетов и установка Docker
```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 3.5. Проверка установки
```bash
sudo systemctl status docker
```

**Что проверить:** Статус службы должен быть `active (running)`

### 3.6. Настройка прав доступа
```bash
sudo usermod -aG docker $USER
sudo usermod -aG dialout $USER
sudo usermod -aG tty $USER  
sudo usermod -aG i2c $USER
sudo usermod -aG video $USER
sudo usermod -aG gpio $USER
sudo reboot
```

**Пояснение:**
- `usermod -aG docker $USER` - добавляет текущего пользователя в группу docker для запуска контейнеров без sudo
- Требуется перезагрузка для применения изменений в группах

**Документация:** [Официальная инструкция по установке Docker](https://docs.docker.com/engine/install/debian/)

## 4. Подключение полетного контроллера (Matek H743 Mini V3)

### 4.1. Физическое подключение
Соедините платы согласно следующей схеме:

| Контакт на Waveshare CM4-NANO-B/C | Контакт на Matek H743 Mini V3 |
|-----------------------------------|-------------------------------|
| TXD (UART2_TX_M0, Pin 8)               | RX4                           |
| RXD (UART2_RX_M0, Pin 10)              | TX4                           |
| Любой контакт GND                 | GND                           |

[Распиновка CM5](https://docs.radxa.com/en/som/cm/cm5/hardware/hw-interface#gpio-interface)

### 4.2. Настройка UART в системе
1.  Активируйте последовательный порт через `rsetup`:
    ```bash
    sudo rsetup
    ```
    *   Выберите: **Overlays -> (Yes) -> Manage overlays**.
    *   Найдите и выберите пробелом пункт `Enable UART2-M0`.
    *   Пересоберите оверлеи и перезагрузите систему.

2.  **Проверка доступности порта:**
    После перезагрузки проверьте наличие устройства UART2 и настройте права:
    ```bash
    ls -la /dev/ttyS2
    sudo usermod -aG dialout $USER  # Добавление пользователя в группу dialout
    ```
    *Для применения изменения группы требуется переподключение по SSH.*

### 4.3. Тестирование связи (опционально)
Для проверки соединения установите терминальную программу `minicom`:
```bash
sudo apt install minicom
sudo minicom -D /dev/ttyS2 -b 921600
```

---

## 5. Настройка Wi-Fi подключения
### 5.1. Установка драйвера для адаптера TP-LINK TL-WN725N

Wi-Fi адаптер **TP-LINK TL-WN725N** на чипе **Realtek RTL8188EUS** требует установки дополнительных драйверов в современных дистрибутивах Linux, таких как Radxa OS (Debian Bookworm).

#### 5.1.1. Проверка адаптера
Перед установкой убедитесь, что система распознает ваш адаптер:

```bash
lsusb | grep -i realtek
```

Ожидаемый вывод для TP-LINK TL-WN725N:
```
Bus 001 Device 003: ID 0bda:8179 Realtek Semiconductor Corp. RTL8188EUS 802.11n Wireless Network Adapter
```

#### 5.1.2. Установка драйвера
Выполните следующие команды для установки совместимого драйвера:

```bash
# Обновление системы и установка зависимостей
sudo apt update
sudo apt install -y git build-essential dkms linux-headers-$(uname -r)

# Клонирование и установка драйвера
git clone https://github.com/petayyyy/rtl8188eus.git
cd rtl8188eus

# Сборка и установка драйвера
sudo make clean
sudo make all -j$(nproc)
sudo make install

# Загрузка драйвера в ядро
sudo modprobe 8188eu
```

#### 5.1.3. Проверка установки
Убедитесь, что драйвер загружен корректно:

```bash
# Проверка загруженного модуля
lsmod | grep 8188

# Проверка сетевого интерфейса
ip a | grep -A 2 wl
```

После успешной установки появится новый сетевой интерфейс с именем вида `wlxXXXXXXXXXXXX` (основанным на MAC-адресе адаптера).

**Важно:** Имя интерфейса может отличаться от стандартного `wlan0`. В дальнейших командах используйте фактическое имя вашего интерфейса.

### 5.2. Настройка Wi-Fi подключения с помощью `nmcli`

`nmcli` (Network Manager Command Line) — утилита для управления сетевыми подключениями из командной строки.

#### 5.2.1. Поиск доступных сетей
```bash
# Просмотр всех доступных Wi-Fi сетей
sudo nmcli device wifi list
```
#### 5.2.2. Подключение к Wi-Fi сети
```bash
# Создание профиля с автоматическим подключением
sudo nmcli connection add type wifi con-name "Router" \
  ifname <имя_интерфейса> ssid "Router_name" \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk "Router_password" \
  connection.autoconnect yes
```

#### 5.2.3. Пример создания нескольких профилей с приоритетами
Вы можете создать несколько профилей для разных сетей с разными приоритетами автоматического подключения:

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

#### 5.2.4. Управление подключениями
**Просмотр всех профилей:**
```bash
nmcli connection show
```

**Активация/деактивация подключения:**
```bash
# Активация профиля
sudo nmcli connection up "Имя_Профиля"

# Деактивация профиля
sudo nmcli connection down "Имя_Профиля"
```

**Удаление профиля:**
```bash
sudo nmcli connection delete "Имя_Профиля"
```

### 5.3. Графический интерфейс (альтернатива)
Если вы предпочитаете графический интерфейс (в терминале), используйте `nmtui`:

```bash
sudo nmtui
```

В меню `nmtui`:
1. Выберите **Activate a connection**
2. Выберите вашу Wi-Fi сеть из списка
3. Введите пароль при запросе
4. Нажмите **OK** для сохранения настроек

### 5.4. Устранение неполадок
Если адаптер не появляется после установки драйвера
```bash
# Перезагрузка модуля ядра
sudo rmmod 8188eu
sudo modprobe 8188eu
# Принудительная перезагрузка службы NetworkManager
sudo systemctl restart NetworkManager
```
---

## 6. Работа с GPIO
Для управления GPIO используйте библиотеку `libgpiod`:
```bash
sudo apt install gpiod libgpiod-dev python3-libgpiod
```
**Основные команды:**
*   `gpiodetect` – список доступных GPIO-чипов.
*   `gpioinfo` – информация о состоянии линий GPIO.
*   `gpioset`, `gpioget` – установка и чтение состояния линий.

[Распиновка CM5](https://docs.radxa.com/en/som/cm/cm5/hardware/hw-interface#gpio-interface)  

---
## 7. Системное администрирование

### 7.1. Мониторинг температуры
Для контроля температуры процессора используйте стандартный интерфейс Linux.

**Пример простого скрипта `temp.sh`:**
```bash
#!/bin/bash
cpu_temp_raw=$(cat /sys/class/thermal/thermal_zone0/temp)
cpu_temp_c=$(echo "scale=1; $cpu_temp_raw / 1000" | bc)
echo "CPU Temperature: $cpu_temp_c°C"
```
Сделайте скрипт исполняемым и запустите:
```bash
chmod +x temp.sh
./temp.sh
```

### 7.2. Базовые команды управления
*   **Безопасное выключение:** `sudo shutdown -h now`
*   **Перезагрузка системы:** `sudo reboot`
*   **Проверка состояния сервиса:** `sudo systemctl status <имя_сервиса>`

> **Рекомендация:** Всегда используйте программное выключение. Прямое отключение питания может привести к повреждению файловой системы.

---

## 8. Полезные ресурсы
*   **Radxa CM5 Официальная документация:** [https://docs.radxa.com/en/som/cm/cm5](https://docs.radxa.com/en/som/cm/cm5)
*   **Waveshare CM4-NANO-C:** [https://www.waveshare.com/wiki/CM4-NANO-C](https://www.waveshare.com/wiki/CM4-NANO-C)
*   **Waveshare CM4-NANO-B:** [https://www.waveshare.com/wiki/CM4-NANO-B](https://www.waveshare.com/wiki/CM4-NANO-B)
*   **Форум поддержки Radxa:** [https://forum.radxa.com/](https://forum.radxa.com/)
*   **Репозиторий Radxa на GitHub:** [https://github.com/radxa/](https://github.com/radxa/)
