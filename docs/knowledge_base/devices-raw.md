# Файлы устройств /dev

На Linux всё оборудование доступно через файлы в `/dev/`. Здесь объясняется, как найти нужное устройство и что значат имена вроде `/dev/ttyS2` или `/dev/spidev0.0`.

---

## Какие устройства используются в Обрике

| Файл | Что это | Для чего |
|------|---------|---------|
| `/dev/gpiochip0` | GPIO-контроллер | Управление пинами (кнопки, светодиоды, электромагнит) |
| `/dev/ttyS2` | UART (аппаратный) | Связь с полётным контроллером PX4 |
| `/dev/ttyUSB0` | USB-UART адаптер | Arduino, лидар LD19 через USB |
| `/dev/ttyACM0` | USB-CDC (Arduino) | Arduino Uno/Nano по USB |
| `/dev/video0` | V4L2 камера | Прямой доступ к камере без libcamera |
| `/dev/video11` | libcamera ISP | Аппаратный ISP Raspberry Pi |
| `/dev/spidev0.0` | SPI шина 0, устройство 0 | LED-лента WS2812B |
| `/dev/i2c-1` | I2C шина 1 | Датчик VL53L1X |

---

## Как проверить, что устройство подключено

```bash
# GPIO-чипы
gpiodetect

# UART-порты
ls /dev/ttyS* /dev/ttyAMA* 2>/dev/null

# USB-устройства (Arduino, лидар)
ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Камера (V4L2)
ls /dev/video*
v4l2-ctl --list-devices

# SPI
ls /dev/spidev*

# I2C
ls /dev/i2c-*
# сканировать устройства на I2C-1 (адрес VL53L1X = 0x29)
i2cdetect -y 1

# показать всю информацию о подключённом USB-устройстве
lsusb
udevadm info /dev/ttyUSB0
```

---

## Различия между платформами

Один и тот же периферийный интерфейс (например, UART) имеет разные имена файлов на разных бортовых компьютерах.

| Интерфейс | Raspberry Pi CM5 | Radxa CM5 | Orange Pi Zero 3W | Orange Pi 5 Pro | RDK X3 |
|-----------|-----------------|-----------|-------------------|-----------------|--------|
| UART (FMU) | `/dev/ttyAMA2` | `/dev/ttyS2` | `/dev/ttyS5` | `/dev/ttyS2` | `/dev/ttyS0` |
| GPIO чип | `/dev/gpiochip0` | `/dev/gpiochip0` | `/dev/gpiochip0` | `/dev/gpiochip0` | `/dev/gpiochip0` |
| SPI | `/dev/spidev0.0` | `/dev/spidev0.0` | — | `/dev/spidev0.0` | — |
| I2C | `/dev/i2c-1` | `/dev/i2c-1` | `/dev/i2c-3` | `/dev/i2c-2` | `/dev/i2c-1` |
| ISP | `/dev/video11` | `/dev/video11` | `/dev/video0` | `/dev/video0` | `/dev/video8` |

> **Примечание** Конкретное имя порта можно посмотреть в конфиге Docker-образа для своей платформы: `sverk_ws/scripts/docker/`.

---

## Права доступа к устройствам

Docker-контейнер Обрика настроен так, что нужные устройства уже доступны. Если вы работаете вне контейнера (например, тестируете на своём компьютере), нужно добавить пользователя в группы:

```bash
# группы для доступа к устройствам
sudo usermod -aG dialout $USER  # UART, USB-Serial (/dev/ttyS*, /dev/ttyUSB*)
sudo usermod -aG gpio $USER     # GPIO (/dev/gpiochip*)
sudo usermod -aG spi $USER      # SPI (/dev/spidev*)
sudo usermod -aG i2c $USER      # I2C (/dev/i2c-*)
sudo usermod -aG video $USER    # камера (/dev/video*)

# применить без перезапуска
newgrp dialout
```

Или дать временный доступ:
```bash
sudo chmod a+rw /dev/ttyUSB0
```

---

## udev — постоянные имена устройств

Проблема: `/dev/ttyUSB0` может стать `/dev/ttyUSB1` после перезапуска, если порядок подключения изменился. Решение — udev-правило с постоянным именем:

```bash
# посмотреть уникальный идентификатор устройства
udevadm info /dev/ttyUSB0 | grep -E "ID_SERIAL|ID_VENDOR|ID_MODEL"

# создать правило (пример для Arduino)
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduino"' \
  | sudo tee /etc/udev/rules.d/99-arduino.rules

# применить
sudo udevadm control --reload-rules && sudo udevadm trigger
# теперь Arduino всегда доступен как /dev/arduino
```
