"""
Что мы делаем: мониторим батарею PX4 через drone.topic.subscribe —
подписываемся на RAW топик PX4 и выводим напряжение, заряд и ток в реальном времени.

Если заряд упадёт ниже 20% — предупреждение.
Остановка: Ctrl+C.
"""
import sverk_interfaces
from px4_msgs.msg import BatteryStatus  # RAW сообщение от PX4

# Пороговые значения
WARN_REMAINING = 0.20   # 20% заряда — предупреждение
CRIT_REMAINING = 0.10   # 10% заряда — критическая ситуация
CELLS = 3               # 3S батарея

drone = sverk_interfaces.init(Nodename="battery_monitor")

def on_battery(msg: BatteryStatus) -> None:
    """Вызывается при каждом новом сообщении о состоянии батареи."""
    voltage = msg.voltage_v              # напряжение всего пакета в вольтах
    remaining = msg.remaining            # заряд от 0.0 до 1.0
    current = msg.current_a             # ток потребления в амперах
    cell_voltage = voltage / CELLS      # напряжение на банку

    status = "OK"
    if remaining < CRIT_REMAINING:
        status = "!!! КРИТИЧНО — САДИСЬ НЕМЕДЛЕННО !!!"
    elif remaining < WARN_REMAINING:
        status = "ВНИМАНИЕ — заряд низкий"

    print(
        f"Батарея: {voltage:.2f} В  ({cell_voltage:.2f} В/банка)  "
        f"заряд: {remaining * 100:.0f}%  "
        f"ток: {current:.1f} А  "
        f"[{status}]"
    )

print("Мониторинг батареи... Ctrl+C для выхода")
print("-" * 60)

# Подписываемся на RAW топик PX4 (напрямую через DDS)
sub = drone.topic.subscribe(
    BatteryStatus,
    "/fmu/out/battery_status",
    on_battery
)

try:
    drone.topic.spin()   # блокируем — ждём сообщений
except KeyboardInterrupt:
    print("\nОстановлено.")
finally:
    drone.close()
