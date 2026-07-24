#!/usr/bin/env python3
"""
Комплексный демо-полёт (НИЗКОУРОВНЕВАЯ версия — БЕЗ sverk_interfaces, голый rclpy).

Сценарий (тот же, что в высокоуровневой версии):
  1. взлёт (navigate, frame_id='body', auto_arm);
  2. navigate к точке A впереди + короткая пауза («подождать немного»);
  3. манёвр обхода через сервис /set_trajectory: удерживаем медленно ползущую
     вперёд точку И подмешиваем боковую скорость + УГЛОВУЮ СКОРОСТЬ рыскания —
     дрон виляет и водит носом, визуально как будто обходит препятствия;
  4. navigate_wait обратно к точке взлёта (ждём прибытия — здесь опросом
     телеметрии до допуска; в sverk_interfaces прибытие считает борт через
     action navigate_to);
  5. посадка.

Парная ВЫСОКОУРОВНЕВАЯ версия (на базе sverk_interfaces):
  sverk_interfaces/examples/avoidance_mission.py

Запуск (внутри контейнера / sourced Humble + workspace):
  python3 avoidance_mission_raw.py
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node

from offboard_interfaces.srv import GetTelemetry, Navigate, SetTrajectory
from std_srvs.srv import Trigger

NAN = float("nan")

# --- параметры демо (манёвр-«змейка» — те же, что в проверочном тесте) ---
TAKEOFF_Z = 1.5      # высота взлёта, м
FORWARD = 3.0        # перелёт к точке A перед манёвром (вдоль +x карты), м
WEAVE_LEG = 2.0      # насколько цель уезжает вперёд ЗА время манёвра, м
WEAVE_T = 12.0       # длительность манёвра-змейки, с
WEAVE_PERIOD = 4.0   # период виляния/рыскания, с
V_LAT = 0.6          # амплитуда боковой скорости (виляние), м/с
YAW_RATE = 0.6       # амплитуда угловой скорости рыскания, рад/с
RATE_HZ = 15.0       # частота обновления цели в манёвре, Гц
REACH_TOL = 0.25     # допуск прибытия для navigate_wait (опрос), м
REACH_TIMEOUT = 40.0 # предел ожидания прибытия, с


class DroneController(Node):
    def __init__(self):
        super().__init__("avoidance_mission_raw")
        self.navigate_client = self.create_client(Navigate, "/navigate")
        self.set_trajectory_client = self.create_client(SetTrajectory, "/set_trajectory")
        self.telemetry_client = self.create_client(GetTelemetry, "/get_telemetry")
        self.land_client = self.create_client(Trigger, "/land")
        self._wait_for_services()
        self.get_logger().info("все сервисы готовы")

    def _wait_for_services(self):
        clients = [
            (self.navigate_client, "navigate"),
            (self.set_trajectory_client, "set_trajectory"),
            (self.telemetry_client, "get_telemetry"),
            (self.land_client, "land"),
        ]
        for client, name in clients:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("ROS2 прерван, выход")
                self.get_logger().info(f"жду сервис {name}...")

    def _call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            self.get_logger().error(f"сервис {client.srv_name} — таймаут")
            return None
        return future.result()

    # --- обёртки сервисов ---

    def navigate(self, x, y, z, yaw, speed, frame_id, auto_arm=False):
        req = Navigate.Request()
        req.x, req.y, req.z = float(x), float(y), float(z)
        req.yaw, req.speed = float(yaw), float(speed)
        req.frame_id, req.auto_arm = str(frame_id), bool(auto_arm)
        resp = self._call(self.navigate_client, req)
        ok = bool(resp and resp.success)
        self.get_logger().info(f"navigate -> success={ok}")
        return ok

    def set_trajectory(self, *, x=NAN, y=NAN, z=NAN, vx=NAN, vy=NAN, vz=NAN,
                       ax=NAN, ay=NAN, az=NAN, yaw=NAN, yaw_rate=NAN,
                       frame_id="map", auto_arm=False):
        req = SetTrajectory.Request()
        req.x, req.y, req.z = float(x), float(y), float(z)
        req.vx, req.vy, req.vz = float(vx), float(vy), float(vz)
        req.ax, req.ay, req.az = float(ax), float(ay), float(az)
        req.yaw, req.yaw_rate = float(yaw), float(yaw_rate)
        req.frame_id, req.auto_arm = str(frame_id), bool(auto_arm)
        return self._call(self.set_trajectory_client, req, timeout=5.0)

    def get_telemetry(self, frame_id="map"):
        req = GetTelemetry.Request()
        req.frame_id = str(frame_id)
        return self._call(self.telemetry_client, req)

    def land(self):
        resp = self._call(self.land_client, Trigger.Request(), timeout=15.0)
        ok = bool(resp and resp.success)
        self.get_logger().info(f"land -> success={ok}")
        return ok

    # --- аналог navigate_wait: команда + опрос телеметрии до допуска ---

    def navigate_wait(self, x, y, z, yaw, speed, frame_id="map"):
        self.navigate(x, y, z, yaw, speed, frame_id)
        t0 = time.monotonic()
        while time.monotonic() - t0 < REACH_TIMEOUT:
            tel = self.get_telemetry("map")
            if tel and tel.connected:
                dist = math.hypot(tel.x - x, tel.y - y, tel.z - z)
                if dist <= REACH_TOL:
                    self.get_logger().info(f"прибыл (d={dist:.2f} м)")
                    return True
            time.sleep(0.3)
        self.get_logger().warn("navigate_wait: не дождался прибытия (таймаут)")
        return False


def main():
    rclpy.init()
    drone = DroneController()
    try:
        # 1. Взлёт.
        drone.get_logger().info("[1] взлёт")
        drone.navigate(0.0, 0.0, TAKEOFF_Z, 0.0, 0.5, "body", auto_arm=True)
        time.sleep(10.0)

        # Точка взлёта в карте.
        start = drone.get_telemetry("map")
        if not (start and start.connected):
            drone.get_logger().error("нет телеметрии — прерываю")
            return
        drone.get_logger().info(
            f"старт map=({start.x:.2f}, {start.y:.2f}, {start.z:.2f}) yaw={start.yaw:.2f}")
        sx, sy, sz, syaw = start.x, start.y, start.z, start.yaw

        # 2. К точке A впереди + пауза.
        a_x, a_y = sx + FORWARD, sy
        drone.get_logger().info("[2] navigate_wait -> A, пауза")
        drone.navigate_wait(a_x, a_y, sz, syaw, 0.7, "map")
        time.sleep(3.0)

        # 3. Манёвр обхода: база (позиция ползёт вперёд) + боковая скорость
        #    (виляние) + угловая скорость рыскания (нос водит).
        drone.get_logger().info("[3] set_trajectory: обход (виляние + рыскание)")
        t0 = time.monotonic()
        dt = 1.0 / RATE_HZ
        while (t := time.monotonic() - t0) < WEAVE_T:
            prog = WEAVE_LEG * min(t / WEAVE_T, 1.0)
            ph = 2.0 * math.pi * t / WEAVE_PERIOD
            drone.set_trajectory(
                x=a_x + prog, y=a_y, z=sz,                 # база: позиция
                vx=0.0, vy=V_LAT * math.sin(ph), vz=0.0,   # боковое виляние
                yaw_rate=YAW_RATE * math.sin(ph),          # рыскание (угл. скорость)
                frame_id="map",
            )
            time.sleep(dt)

        # 4. Возврат к точке взлёта.
        drone.get_logger().info("[4] navigate_wait -> точка взлёта")
        drone.navigate_wait(sx, sy, sz, syaw, 0.7, "map")
        time.sleep(2.0)

        # 5. Посадка.
        drone.get_logger().info("[5] посадка")
        drone.land()
        drone.get_logger().info("готово")
    finally:
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
