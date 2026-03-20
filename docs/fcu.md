## Настройка MAVLink Router

MAVLink Router автоматически запускается при старте контейнера и настраивается через переменные окружения в файле 'docker-compose.yml':

### Переменные окружения

| Переменная | Значение по умолчанию | Описание |
|------------|----------------------|----------|
| `ON_MAVLINK_ROUTER` | `true` | Включение/отключение MAVLink Router (`true`/`false`) |
| `USB_DEVICE_MR` | `/dev/ttyACM0` | Путь к устройству полетного контроллера |
| `USB_DEVICE_BAUD` | `57600` | Скорость обмена для последовательного порта |

### Подключение QGroundControl
После успешного запуска MAVLink Router вы можете подключить QGroundControl:

1. Получите IP-адрес из логов контейнера или с помощью `ip add`
2. В QGroundControl: Application Settings → Comm Links
3. Добавьте UDP соединение:
   - Port: 14550
   - Address: IP контейнера/дрона
4. Сохраните и подключитесь
---
## Запуск моста между PX4 и ROS 2
Через Usb адаптер 
```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```
Через UART 
```bash
MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600
```
Для проверки посмотрите даные с топика IMU
```bash
ros2 topic echo --once /fmu/out/sensor_combined
# или его hz
ros2 topic hz /fmu/out/sensor_combined
```