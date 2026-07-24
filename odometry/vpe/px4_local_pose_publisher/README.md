# px4_local_pose_publisher

Пакет отправляет в PX4 EKF2 **external local position** через
`px4_ros2::LocalPositionMeasurementInterface` (топик
`fmu/in/vehicle_visual_odometry`, сообщение `px4_msgs/VehicleOdometry`) по
uXRCE-DDS. В нём два независимых узла-потребителя под **два источника** внешней
позы — выбирается тот, что соответствует сценарию борта:

| Узел (executable) | Источник | Вход | Сценарий |
|---|---|---|---|
| `pose_subscriber_tf2` (`VpePublisher`) | ArUco-полигон | `/aruco/world_pose` | обрик / гигаобрик с камерой + маркеры |
| `lidar_odom_publisher` (`LidarOdomPublisher`) | 3D-лидар Unitree L2 + point_lio | `/aft_mapped_to_init` | гигаобрик с 3D-лидаром (аддон `lidar_3d`) |

Оба узла — тонкие: конвертация фреймов ENU/FLU→NED/FRD, честное время выборки
(`timestamp_sample`), предохранитель `allow_send_vio` и watchdog устаревания.
Никакой TF/offset-логики в них нет — она вынесена наружу (ArucoTfNode для ArUco;
`T_mount`/`T_post` из launch для лидара).

---

## Сценарий 1 — ArUco VPE (`pose_subscriber_tf2` / `VpePublisher`)

Берёт готовую позу `base_link` в мировом фрейме (ENU, тело FLU) от внешнего
ArucoTfNode, конвертирует её в NED/FRD и отправляет в PX4 EKF2.

Узел **не** содержит TF/offset-логики — она вынесена в ArucoTfNode. Здесь только
конвертация фреймов, корректное время выборки и отправка в EKF2.

## Зависимости

- ROS 2 Humble
- PX4 v1.15.4 + uXRCE-DDS
- `px4_msgs`, `px4_ros_com` — ветка **release/1.15** (совпадает с прошивкой)
- `px4_ros2_cpp` (форк под 1.15, с `reset_counter` в `VehicleOdometry`)
- `geometry_msgs`, `std_msgs`, `Eigen`

## Запуск

```bash
ros2 launch px4_local_pose_publisher pose_subscriber_tf2.launch.py
# по умолчанию allow_send_vio:=false — в PX4 ничего не уходит.
# после проверки позы/фреймов на земле:
ros2 launch px4_local_pose_publisher pose_subscriber_tf2.launch.py allow_send_vio:=true
```

## Топики

| Направление | Топик | Тип | Описание |
|---|---|---|---|
| Вход | `/aruco/world_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Поза `base_link` в world-фрейме (ENU, FLU), `header.stamp` = время захвата кадра. QoS best_effort. |
| Вход | `/aruco/reset_counter` | `std_msgs/UInt8` | Счётчик discontinuity внешнего estimator (бамп при commit нового offset). QoS reliable + transient_local, KeepLast(1). |
| Выход | `fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | External local position для EKF2 (через px4_ros2). |

## Параметры

| Параметр | Тип | По умолч. | Описание |
|---|---|---|---|
| `world_pose_topic` | string | `/aruco/world_pose` | Входная поза. |
| `reset_counter_topic` | string | `/aruco/reset_counter` | Входной счётчик сброса. |
| `allow_send_vio` | bool | `false` | Предохранитель: пока false — в PX4 ничего не уходит. |
| `watchdog_rate_hz` | double | `10.0` | Частота watchdog-проверки устаревания (НЕ частота отправки — она on-new). |
| `data_timeout_sec` | double | `0.5` | Таймаут без свежей позы → предупреждение, отправка прекращается. |

`allow_send_vio`, `watchdog_rate_hz`, `data_timeout_sec` можно менять на лету
(`on_set_parameters_callback`), напр. включить отправку без перезапуска:
```bash
ros2 param set /pose_subscriber_tf2 allow_send_vio true
```
Топики (`*_topic`) — только через launch/рестарт (подписки создаются в конструкторе).

`transient_local` QoS передаёт последнее значение `reset_counter` поздно
запущенному VPE. Смена счётчика сообщает PX4 о смене системы координат внешней
оценки.

## Поведение

1. На каждое сообщение `/aruco/world_pose` (`worldPoseCallback`):
   позиция конвертируется ENU→NED (`enu_to_ned_local_frame`), ориентация —
   ENU/FLU→NED/FRD (`ros_to_px4_orientation`, делает **оба** поворота: мировой и
   телесный), `timestamp_sample` берётся из `header.stamp` (время захвата кадра),
   диагональ ковариации переносится со свопом осей, прикладывается актуальный
   `reset_counter` — и измерение **сразу** уходит в EKF2 (on-new, один раз).
2. `watchdogCallback` только следит за устареванием данных и пишет предупреждение;
   он **ничего не отправляет**. При дропауте EKF2 коастит на IMU и сам перестаёт
   фьюзить EV по таймауту — повторная отправка старой/нейтральной позы
   дестабилизировала бы фильтр, поэтому её нет.
3. При смене `reset_counter` EKF2 обрабатывает разрыв как релокализацию.
   Потеря и возврат маркеров сохраняют counter вместе с `map→aruco_map` offset.
4. Covariance floors применяются upstream в ArucoTfNode. VPE переставляет X/Y
   variances при ENU→NED conversion и передаёт их в PX4 без повторной оценки.

## Настройка PX4 и отладка (v1.15.4)

Полный справочник по параметрам EKF2 (что выставить до полёта, что тюнить в
полёте, битовые маски), методам проверки валидности данных и командам
MAVLink-консоли — в отдельном документе:

**→ [`PX4_EKF2_TUNING_AND_DEBUG.md`](PX4_EKF2_TUNING_AND_DEBUG.md)**

Кратко: EV-ядро (`EKF2_EV_CTRL=11`, `EV_NOISE_MD=0`, `GPS/MAG/BARO` off,
`UXRCE_DDS_SYNCT=1`) уже корректно. Перед полётом — `EKF2_EV_DELAY=0` (не 50:
`timestamp_sample` = время захвата + timesync), высоту на лидар (`EKF2_HGT_REF=2`),
indoor-лимиты скорости/наклона. **`EKF2_EV_POS_X/Y/Z` оставить 0** — экстринсик
камеры применяется на стороне ArUco/tf, дублировать его здесь нельзя.

Перед `allow_send_vio:=true` проверь `/aruco/world_pose`, фреймы и
`estimator_aid_src_ev_pos` / `estimator_aid_src_ev_yaw` в логе (см. §6–7 дока).

---

## Сценарий 2 — лидарная одометрия (`lidar_odom_publisher` / `LidarOdomPublisher`)

Источник — **3D-лидар Unitree L2** с одометрией [point_lio]. point_lio публикует
позу **лидара** в своём мире одометрии (гравитационно-выровнен, yaw произвольный)
в `nav_msgs/Odometry` на `/aft_mapped_to_init`. Узел пересчитывает её в позу
`base_link` в world ENU по цепочке трансформов `T_final = T_post · T_odom · T_mount`
(где `T_mount` — поза лидара на раме, `T_post` — доворот мира одометрии к ENU),
конвертирует в NED/FRD и шлёт в EKF2 **on-new** с ограничением частоты `max_rate_hz`
(point_lio может отдавать сотни Гц, EKF2 столько не нужно).

Этот сценарий приходит вместе с аддоном 3D-лидара (тип `lidar_3d` в
`image_builder/types.yaml`, `Dockerfile.3d_lidar`, `docker-compose.3d_lidar.yml`).
На борту узел запускается из `full_system_real.launch.py` по флагу
`ENABLE_LIDAR_3D_ODOM` (закрыт предохранителем `LIDAR_3D_ODOM_SEND_VIO`); значения
`LIDAR_3D_MOUNT_RPY` / `LIDAR_3D_MOUNT_XYZ` / `LIDAR_3D_POST_RPY` пробрасываются из
device `.env` в launch-аргументы `mount_rpy` / `mount_xyz` / `post_rpy`.

### Запуск (отдельно, для проверки на земле)

```bash
ros2 launch px4_local_pose_publisher lidar_odom_publisher.launch.py
# по умолчанию allow_send_vio:=false — в PX4 ничего не уходит.
# после проверки одометрии/фреймов на земле:
ros2 launch px4_local_pose_publisher lidar_odom_publisher.launch.py allow_send_vio:=true
```

> **Сначала проверь фреймы на земле.** Дефолты `mount_*`/`post_*` — значения
> тестовой рамы `ros2_uni` (лидар 17 см позади `base_link`, наклон −7° по тангажу,
> мир одометрии довёрнут на 180° по yaw). **Под своё крепление их надо перемерить.**
> Подними и поверни раму руками и сверь знаки осей `/aft_mapped_to_init` до отправки
> в PX4.

### Топики

| Направление | Топик | Тип | Описание |
|---|---|---|---|
| Вход | `/aft_mapped_to_init` | `nav_msgs/Odometry` | Поза лидара в мире одометрии point_lio. QoS best_effort. |
| Выход | `fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | External local position для EKF2 (через px4_ros2). |

### Параметры

| Параметр | Тип | По умолч. | Описание |
|---|---|---|---|
| `odom_topic` | string | `/aft_mapped_to_init` | Входная одометрия point_lio (read-only, через launch). |
| `mount_rpy` | double[3] | `[0.0, -0.122, 0.0]` | RPY (рад) лидара относительно `base_link` (`Rz·Ry·Rx`). Read-only. |
| `mount_xyz` | double[3] | `[-0.170, 0.0, 0.0]` | Смещение (м) лидара относительно `base_link`. Read-only. |
| `post_rpy` | double[3] | `[0.0, 0.0, 3.14159]` | Доворот мира одометрии point_lio к world ENU (RPY, рад). Read-only. |
| `allow_send_vio` | bool | `false` | Предохранитель: пока false — в PX4 ничего не уходит. |
| `max_rate_hz` | double | `30.0` | Ограничение частоты отправки в EKF2 (0 = без лимита). |
| `watchdog_rate_hz` | double | `10.0` | Частота watchdog-проверки устаревания (НЕ частота отправки). |
| `data_timeout_sec` | double | `0.5` | Таймаут без свежей одометрии → предупреждение, отправка прекращается. |

`allow_send_vio`, `max_rate_hz`, `watchdog_rate_hz`, `data_timeout_sec` меняются на
лету (`on_set_parameters_callback`). Геометрия (`odom_topic`, `mount_*`, `post_rpy`)
— только через launch/рестарт (подписка и трансформы создаются в конструкторе).

### Отличия от сценария 1 (ArUco)

- **Частота.** point_lio отдаёт десятки–сотни Гц, поэтому есть `max_rate_hz`
  (у ArUco его нет — маркеры идут с частотой камеры).
- **reset_counter = 0 всегда.** point_lio не сбрасывает свой мир в полёте (в
  отличие от ArUco-offset, где смена counter = релокализация). Разрыва системы
  координат нет, поэтому и `reset_counter_topic` не нужен.
- **Ковариация.** Диагональ `pose.covariance` одометрии со свопом X/Y (ENU→NED);
  point_lio обычно шлёт нули → флор `clampVar`. `T_post` учитывается точно только
  для yaw, кратных 90°; реальный нижний порог всё равно задают
  `EKF2_EVP_NOISE`/`EKF2_EVA_NOISE`.

Настройка EKF2 — та же, что в [`PX4_EKF2_TUNING_AND_DEBUG.md`](PX4_EKF2_TUNING_AND_DEBUG.md)
(EV-ядро `EKF2_EV_CTRL=11`, `EKF2_EV_DELAY=0`, высота на лидар `EKF2_HGT_REF=2`,
`EKF2_EV_POS_X/Y/Z=0` — экстринсик применяется через `T_mount`, дублировать в PX4
нельзя). Перед `allow_send_vio:=true` проверь `estimator_aid_src_ev_pos` /
`estimator_aid_src_ev_yaw` в логе.

[point_lio]: https://github.com/hku-mars/Point-LIO
