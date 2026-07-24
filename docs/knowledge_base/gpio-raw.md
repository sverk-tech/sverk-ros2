# Как работает GPIO

Описание внутреннего устройства GPIO: как библиотека `sverk_interfaces` работает с `/dev/gpiochip0` через libgpiod.

---

## Что такое /dev/gpiochip0

На Linux каждый GPIO-контроллер представлен файлом `/dev/gpiochip0`, `/dev/gpiochip1` и т.д. Это не обычный файл — это интерфейс к драйверу ядра, который управляет физическими пинами.

```bash
# посмотреть, какие GPIO-чипы есть в системе
gpiodetect
# Пример вывода: gpiochip0 [pinctrl-bcm2712] (54 lines)

# посмотреть все пины чипа 0 (имена, направление, текущий статус)
gpioinfo gpiochip0

# установить пин 17 в HIGH из терминала
gpioset gpiochip0 17=1

# установить пин 17 в LOW из терминала
gpioset gpiochip0 17=0

# прочитать значение пина 17
gpioget gpiochip0 17
```

---

## Управление из Python через gpiod

### Вывод (output) — включить/выключить

```python
# Что мы делаем: мигаем светодиодом на GPIO 17 через прямое API gpiod
import time
import gpiod   # установить: sudo apt install python3-libgpiod

chip = gpiod.Chip("/dev/gpiochip0")  # открываем чип

line = chip.get_line(17)             # получаем линию по номеру пина
line.request(
    consumer="my_program",           # имя программы (для отладки)
    type=gpiod.LINE_REQ_DIR_OUT,     # выход
    default_vals=[0]                 # начальное значение: LOW
)

for _ in range(5):
    line.set_value(1)   # HIGH — светодиод горит
    time.sleep(0.5)
    line.set_value(0)   # LOW  — светодиод выключен
    time.sleep(0.5)

line.release()          # всегда освобождайте линию по завершении
chip.close()
```

> **Примечание** `drone.gpio.pin_on(17)` делает ровно это: `chip.get_line(17)` → `request(DIR_OUT)` → `set_value(1)`.

---

### Вход (input) — читать кнопку

```python
# Что мы делаем: ждём нажатия кнопки на GPIO 24
import gpiod

chip = gpiod.Chip("/dev/gpiochip0")
line = chip.get_line(24)
line.request(
    consumer="button_reader",
    type=gpiod.LINE_REQ_DIR_IN,   # вход
)

print("Нажмите кнопку...")
while True:
    value = line.get_value()      # читаем состояние: 0 или 1
    if value == 1:
        print("Нажато!")
        break

line.release()
chip.close()
```

> **Примечание** `drone.gpio.pin_read(24)` делает то же самое через `_get_line(24, "input")` → `get_value()`.

---

## Как drone.gpio.* соотносится с gpiod

| drone.gpio.* | gpiod эквивалент |
|-------------|-----------------|
| `drone.gpio.pin_on(pin)` | `get_line(pin).set_value(1)` |
| `drone.gpio.pin_off(pin)` | `get_line(pin).set_value(0)` |
| `drone.gpio.pin_read(pin)` | `get_line(pin).get_value()` |
| `drone.gpio.pin_release(pin)` | `line.release()` |

---

## Важные ограничения GPIO

| Правило | Почему |
|---------|--------|
| Рабочее напряжение **3.3 В** | 5 В сожжёт вход GPIO-контроллера |
| Максимальный ток на пин **~16 мА** | При большем токе напряжение просядет |
| Нельзя управлять силовой нагрузкой напрямую | Нужен транзисторный ключ или реле |
| Один пин — один потребитель | Два процесса не могут запросить один пин одновременно |

---

## Электромагнит через gpiod

```python
# Что мы делаем: включаем электромагнит через транзисторный ключ на GPIO 23
import gpiod
import time

chip = gpiod.Chip("/dev/gpiochip0")
line = chip.get_line(23)
line.request(consumer="magnet", type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

line.set_value(1)   # включить электромагнит (транзистор открыт — ток через катушку)
time.sleep(3.0)     # держим 3 секунды
line.set_value(0)   # выключить (транзистор закрыт)

line.release()
chip.close()
```

> **Примечание** `drone.gpio.magnet_on(23)` — это просто `drone.gpio.pin_on(23)`, то есть тот же `set_value(1)`.
