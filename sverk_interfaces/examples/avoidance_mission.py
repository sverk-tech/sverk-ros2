"""
Комплексный демо-полёт (ВЕРСИЯ НА БАЗЕ sverk_interfaces).

Сценарий:
  1. взлёт (navigate, frame_id='body', auto_arm);
  2. navigate к точке A впереди + короткая пауза («подождать немного»);
  3. манёвр обхода через сервис /set_trajectory: удерживаем медленно ползущую
     вперёд точку И подмешиваем боковую скорость + УГЛОВУЮ СКОРОСТЬ рыскания —
     дрон виляет из стороны в сторону и водит носом, визуально как будто
     обходит препятствия во время движения;
  4. navigate_wait обратно к точке взлёта (ждём прибытия);
  5. посадка.

Парная НИЗКОУРОВНЕВАЯ версия (без sverk_interfaces, на голом rclpy):
  offboard/offboard_control/examples/avoidance_mission_raw.py

Запуск (внутри контейнера / sourced Humble + workspace):
  python3 avoidance_mission.py
"""

import math
import time

import sverk_interfaces

# --- параметры демо (манёвр-«змейка» — те же, что в проверочном тесте) ---
TAKEOFF_Z = 1.5      # высота взлёта, м
FORWARD = 3.0        # перелёт к точке A перед манёвром (вдоль +x карты), м
WEAVE_LEG = 2.0      # насколько цель уезжает вперёд ЗА время манёвра, м
WEAVE_T = 12.0       # длительность манёвра-змейки, с
WEAVE_PERIOD = 4.0   # период виляния/рыскания, с
V_LAT = 0.6          # амплитуда боковой скорости (виляние), м/с
YAW_RATE = 0.6       # амплитуда угловой скорости рыскания, рад/с
RATE_HZ = 15.0       # частота обновления цели в манёвре, Гц

drone = sverk_interfaces.init(Nodename="avoidance_mission")
try:
    # 1. Взлёт: набор высоты относительно корпуса, с авто-армингом.
    print("[1] взлёт")
    drone.control.navigate(
        x=0.0, y=0.0, z=TAKEOFF_Z, yaw=0.0,
        speed=0.5, frame_id="body", auto_arm=True,
    )
    time.sleep(10.0)

    # Точка взлёта в карте — сюда вернёмся в конце.
    start = drone.control.get_telemetry(frame_id="map")
    print(f"    старт map=({start.x:.2f}, {start.y:.2f}, {start.z:.2f}) yaw={start.yaw:.2f}")

    # 2. Летим к точке A впереди и ждём прибытия, затем короткая пауза.
    a_x, a_y = start.x + FORWARD, start.y
    print("[2] navigate_wait -> A, пауза")
    drone.control.navigate_wait(
        x=a_x, y=a_y, z=start.z, yaw=start.yaw, speed=0.7, frame_id="map",
    )
    time.sleep(3.0)

    # 3. Манёвр «обхода препятствий»: база — медленно ползущая вперёд точка;
    #    поверх неё feed-forward боковая скорость (виляние) и угловая скорость
    #    рыскания (нос водит). Сервис /set_trajectory вызывается в цикле.
    print("[3] set_trajectory: обход (виляние + рыскание)")
    t0 = time.monotonic()
    dt = 1.0 / RATE_HZ
    while (t := time.monotonic() - t0) < WEAVE_T:
        prog = WEAVE_LEG * min(t / WEAVE_T, 1.0)  # цель уезжает ещё на WEAVE_LEG вперёд
        ph = 2.0 * math.pi * t / WEAVE_PERIOD
        drone.control.set_trajectory(
            x=a_x + prog, y=a_y, z=start.z,               # база: точка (позиция)
            vx=0.0, vy=V_LAT * math.sin(ph), vz=0.0,      # боковое виляние (скорость)
            yaw_rate=YAW_RATE * math.sin(ph),             # рыскание носом (угл. скорость)
            frame_id="map", auto_arm=False,
        )
        time.sleep(dt)

    # 4. Возврат к точке взлёта и ожидание прибытия.
    print("[4] navigate_wait -> точка взлёта")
    drone.control.navigate_wait(
        x=start.x, y=start.y, z=start.z, yaw=start.yaw, speed=0.7, frame_id="map",
    )
    time.sleep(2.0)

    # 5. Посадка.
    print("[5] посадка")
    drone.control.land(timeout=15.0)
    print("готово")
finally:
    drone.close()
