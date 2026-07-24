# Camera Calibration

ROS 2-нода для калибровки камеры дрона по калибровочной доске. Поддерживает четыре типа досок (**chessboard, charuco, circles, acircles**) и две модели камеры (**pinhole, fisheye**), онлайн-захват кадров с превью и авто-захватом, калибровку с **авто-отсевом кадров-выбросов**, экспорт в ROS-совместимый YAML и **авто-деплой** результата в config-папки драйвера камеры. Есть офлайн-режим (калибровка по готовой папке кадров) и отдельный инструмент верификации калибровочного файла.

---

## Содержание

- [Требования](#требования)
- [Сборка](#сборка)
- [Запуск](#запуск)
- [Параметры ноды и launch](#параметры-ноды-и-launch)
- [Сервисы](#сервисы)
- [Топики](#топики)
- [Конфигурации калибровочных досок](#конфигурации-калибровочных-досок)
- [Авто-отсев выбросов](#авто-отсев-выбросов)
- [Авто-деплой в драйвер камеры](#авто-деплой-в-драйвер-камеры)
- [Офлайн-калибровка по папке](#офлайн-калибровка-по-папке)
- [Верификация калибровки](#верификация-калибровки)
- [Типичный сценарий](#типичный-сценарий)
- [Инструменты и тесты](#инструменты-и-тесты)

---

## Требования

- ROS 2 (Humble / совместимый)
- OpenCV с модулем `aruco` (в образе — `python3-opencv` 4.5.4), `cv_bridge`, `numpy`, `PyYAML`
- Пакеты: `rclpy`, `sensor_msgs`, `std_msgs`, `std_srvs`, `cv_bridge`
- Источник изображения — топик `sensor_msgs/msg/Image` (драйвер камеры уже запущен)

---

## Сборка

Из корня workspace:

```bash
cd ~/sverk_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_calibration
source install/setup.bash
```

---

## Запуск

### Онлайн-калибровка (захват кадров с живой камеры)

```bash
ros2 launch camera_calibration calibration.launch.py \
  image_topic:=/camera_1/image_raw
```

С конфигурацией доски и авто-деплоем сразу из launch (без сервисных вызовов):

```bash
ros2 launch camera_calibration calibration.launch.py \
  image_topic:=/camera_1/image_raw \
  board_type:=charuco camera_model:=fisheye \
  board_width:=11 board_height:=8 square_size:=0.022 marker_size:=0.016 \
  aruco_dict:=DICT_4X4_50 sensor_name:=imx219 \
  auto_deploy:=true auto_deploy_max_reproj:=1.0
```

### Офлайн-калибровка по папке с готовыми кадрами

```bash
ros2 launch camera_calibration calibrate_from_dir.launch.py \
  input_dir:=/home/sverk/camera_calibrations/sessions/<id>/captures \
  board_type:=charuco camera_model:=fisheye \
  board_width:=11 board_height:=8 square_size:=0.022 marker_size:=0.016 \
  aruco_dict:=DICT_4X4_50 sensor_name:=imx219 \
  deploy_driver_package:=mipi_csi_cam_ros2
```

Нода читает картинки из папки, калибрует, сохраняет YAML, деплоит и завершается. См. [Офлайн-калибровка по папке](#офлайн-калибровка-по-папке).

### Верификация готового YAML

```bash
ros2 run camera_calibration verify_calibration_node.py \
  --file ~/camera_calibrations/latest_calibration.yaml
```

---

## Параметры ноды и launch

| Параметр | Тип | По умолчанию | Описание |
|----------|-----|--------------|----------|
| `image_topic` | string | `/image/raw` | Топик изображения камеры |
| `output_dir` | string | `~/camera_calibrations` | Каталог для сессий и YAML |
| `camera_name` | string | `calibrated_camera` | Имя камеры в YAML |
| `frame_id` | string | `camera_optical_1` | frame_id для превью-изображения |
| `preview_rate_hz` | double | 5.0 | Частота публикации превью (0 — выкл.) |
| `publish_preview_when_idle` | bool | true | Публиковать превью вне активной сессии |
| `min_charuco_corners` | int | 10 | Минимум углов charuco для приёма кадра |
| `board_type` | string | `chessboard` | Тип доски: `chessboard`/`charuco`/`circles`/`acircles` |
| `camera_model` | string | `pinhole` | Модель камеры: `pinhole`/`fisheye` |
| `board_width` | int | 9 | Ширина доски (клетки/углы — см. тип) |
| `board_height` | int | 6 | Высота доски |
| `square_size` | double | 0.024 | Размер клетки / шаг кругов (м) |
| `marker_size` | double | 0.018 | Размер ArUco-маркера (charuco, м) |
| `aruco_dict` | string | `DICT_4X4_50` | Словарь ArUco (charuco) |
| `min_frames` | int | 15 | Порог кадров для остановки авто-захвата |
| `max_frame_reproj` | double | 2.0 | Порог авто-отсева выбросов (px); 0 — выкл. |
| `auto_deploy` | bool | false | Авто-деплой при `reproj <= auto_deploy_max_reproj` |
| `auto_deploy_max_reproj` | double | 1.0 | Порог reproj для авто-деплоя (px) |
| `deploy_driver_package` | string | `a733_csi_cam_ros2` | Пакет драйвера — цели деплоя (install+src) |
| `workspace_src` | string | `/home/sverk/sverk_ws/src` | Корень src для поиска config драйвера |
| `deploy_config_dir` | string | `` | Доп. путь для деплоя (сверх пакета) |
| `sensor_name` | string | `imx219` | Имя сенсора в имени файла `<sensor>_<w>x<h>.yaml` |
| `batch_input_dir` | string | `` | Папка кадров для офлайн-режима (если задана) |
| `batch_shutdown` | bool | true | Выйти после офлайн-калибровки |

---

## Сервисы

Конфигурацию доски можно задать параметрами launch (см. выше), а можно менять на лету сервисом `set_config`.

| Сервис | Тип | Параметры запроса<br>---<br>ответ | Что делает |
|--------|-----|-----------------------------------|------------|
| `/calibration/start_session` | `std_srvs/srv/Trigger` | ---<br>`bool success`, `string message` | Начинает новую сессию (каталог в `output_dir`). |
| `/calibration/reset_session` | `std_srvs/srv/Trigger` | ---<br>`bool success`, `string message` | Сбрасывает сессию и захваченные данные. |
| `/calibration/capture` | `std_srvs/srv/Trigger` | ---<br>`bool success`, `string message` | Захватывает один кадр; `success=false`, если доска не найдена. |
| `/calibration/calibrate` | `std_srvs/srv/Trigger` | ---<br>`bool success`, `string message` | Калибрует по кадрам с авто-отсевом выбросов; при `auto_deploy` — деплоит. |
| `/calibration/save` | `std_srvs/srv/Trigger` | ---<br>`bool success`, `string message` | Сохраняет YAML в `output_dir` + `latest_calibration.yaml`. |
| `/calibration/deploy` | `std_srvs/srv/Trigger` | ---<br>`bool success`, `string message` | Сохраняет и копирует YAML в config-папки драйвера. |
| `/calibration/get_state` | `camera_calibration/srv/GetState` | ---<br>`bool success`, `string state_json` | Полное состояние ноды в JSON. |
| `/calibration/list_captures` | `camera_calibration/srv/ListCaptures` | ---<br>`bool success`, `string[] captures` | Список захваченных кадров (JSON-строки). |
| `/calibration/remove_capture` | `camera_calibration/srv/RemoveCapture` | `int32 index`<br>---<br>`bool success`, `string message` | Удаляет кадр по индексу. |
| `/calibration/set_auto_capture` | `camera_calibration/srv/SetAutoCapture` | `bool enable`, `float32 period_sec`<br>---<br>`bool success`, `string message` | Авто-захват раз в интервал (стоп по `min_frames`). |
| `/calibration/set_config` | `camera_calibration/srv/SetCalibrationConfig` | `string board_type`, `string camera_model`, `int32 board_width`, `int32 board_height`, `float32 square_size`, `float32 marker_size`, `string aruco_dict`, `int32 min_frames`, `bool save_captures`<br>---<br>`bool success`, `string message` | Меняет конфигурацию (сбрасывает захваченные кадры). |

**Примеры:**

```bash
# конфигурация charuco + fisheye
ros2 service call /calibration/set_config camera_calibration/srv/SetCalibrationConfig \
"{board_type: 'charuco', camera_model: 'fisheye', board_width: 11, board_height: 8, square_size: 0.022, marker_size: 0.016, aruco_dict: 'DICT_4X4_50', min_frames: 20, save_captures: true}"

# сессия -> захват -> калибровка
ros2 service call /calibration/start_session std_srvs/srv/Trigger
ros2 service call /calibration/capture std_srvs/srv/Trigger
ros2 service call /calibration/calibrate std_srvs/srv/Trigger

# авто-захват раз в 1.5 с (двигай доску)
ros2 service call /calibration/set_auto_capture camera_calibration/srv/SetAutoCapture "{enable: true, period_sec: 1.5}"

# деплой в config драйвера
ros2 service call /calibration/deploy std_srvs/srv/Trigger
```

---

## Топики

| Топик | Тип | Описание |
|-------|-----|----------|
| `/camera_calibration/status` | `std_msgs/String` | Человекочитаемый статус (результаты вызовов). |
| `/camera_calibration/state` | `std_msgs/String` | Полное состояние ноды в JSON (то же, что `get_state`). |
| `/camera_calibration/processed_image` | `sensor_msgs/Image` | Превью с разметкой детекции доски (зелёная — найдена). |

Для просмотра превью используйте `rqt_image_view /camera_calibration/processed_image`, `web_video_server` или веб-интерфейс Sverk.

---

## Конфигурации калибровочных досок

Значения `board_width`/`board_height` трактуются по типу доски: для `chessboard` — число **внутренних углов**, для `charuco`/`circles`/`acircles` — число **клеток/кругов**. Готовые доски для печати — в `boards_examples_a4/`, свои можно сгенерировать на <https://calib.io/pages/camera-calibration-pattern-generator>.

### Шахматная доска (chessboard)
```json
{"board_type": "chessboard", "camera_model": "pinhole", "board_width": 10, "board_height": 7, "square_size": 0.022, "marker_size": 0, "aruco_dict": "", "min_frames": 5, "save_captures": true}
```
Поля ArUco (`marker_size`, `aruco_dict`) игнорируются.

### Шахматная доска с ArUco (charuco)
```json
{"board_type": "charuco", "camera_model": "fisheye", "board_width": 11, "board_height": 8, "square_size": 0.022, "marker_size": 0.016, "aruco_dict": "DICT_4X4_50", "min_frames": 20, "save_captures": true}
```
`marker_size` должен быть меньше `square_size`.

### Доска с кругами (circles)
```json
{"board_type": "circles", "camera_model": "pinhole", "board_width": 11, "board_height": 8, "square_size": 0.022, "marker_size": 0, "aruco_dict": "", "min_frames": 5, "save_captures": true}
```
`square_size` здесь — расстояние между центрами кругов.

### Асимметричные круги (acircles)
```json
{"board_type": "acircles", "camera_model": "pinhole", "board_width": 4, "board_height": 11, "square_size": 0.021213, "marker_size": 0, "aruco_dict": "", "min_frames": 5, "save_captures": true}
```
`square_size` — расстояние между центрами; диаметр влияет только на надёжность детекции.

---

## Авто-отсев выбросов

При калибровке нода итеративно удаляет кадры, у которых средняя ошибка репроекции превышает `max_frame_reproj` (по умолчанию **2.0 px**), и пересчитывает результат — так один битый кадр не портит всю калибровку. Дополнительно для `fisheye` отбрасываются плохо обусловленные (ill-conditioned) кадры. Исходные захваты не удаляются — отсев работает на подмножестве.

В сообщении калибровки видно, что отсеяно:
```
Calibrated (fisheye). reproj_error=0.209442, rms=0.243262, frames=35/36, dropped 1 outlier frame(s) (>2px)
```

- `max_frame_reproj:=1.5` — строже; `max_frame_reproj:=0` — отключить отсев.
- Есть защита: не опускается ниже `max(3, min_frames/2)` кадров.

В YAML пишутся `num_calibration_frames` (использовано), `num_captured_frames`, `dropped_outliers`, `dropped_illcond`.

---

## Авто-деплой в драйвер камеры

Готовый YAML сохраняется как `<sensor_name>_<width>x<height>.yaml` (например `imx219_1280x960.yaml`) и копируется во **все** найденные config-папки пакета `deploy_driver_package`:

- `install/share/<pkg>/config` — действует сразу, без пересборки;
- `src/**/<pkg>/config` — переживает `colcon build`.

Длинный путь указывать не нужно — достаточно имени пакета драйвера (`a733_csi_cam_ros2` для Orange Pi Zero 3W, `mipi_csi_cam_ros2` для RPi/RK3588S). Проверить реальные цели можно в `/camera_calibration/state` (поле `deploy_targets`).

Авто-деплой срабатывает при `auto_deploy:=true`, если `reproj <= auto_deploy_max_reproj`. Ручной деплой — сервисом `/calibration/deploy`.

> После записи в `install/share` учтите: следующий `colcon build --packages-select <driver>` перезальёт install из src — где файл тоже уже лежит, поэтому калибровка сохранится.

---

## Офлайн-калибровка по папке

Если кадры уже сняты (сессия писала их при `save_captures=true` в `output_dir/sessions/<id>/captures/`), можно откалибровать без камеры одной командой — см. [Запуск](#запуск), `calibrate_from_dir.launch.py`. Полезные аргументы:

- `batch_shutdown:=false` — не выходить после калибровки (остаться с сервисами);
- `max_frame_reproj:=1.5` — строже отсев;
- `min_charuco_corners:=18` — жёстче входной фильтр детекции;
- `deploy_driver_package:=<pkg>` — куда деплоить.

Для тонкого разбора (сравнение pinhole/fisheye, per-frame ошибки) есть скрипт `test/recalibrate_offline.py`.

---

## Верификация калибровки

`verify_calibration_node.py` читает YAML и выводит статистику и вердикт с рекомендацией (оставить как есть / улучшить / переснять):

```bash
# по файлу
ros2 run camera_calibration verify_calibration_node.py --file <path.yaml>
# по последней калибровке (~/camera_calibrations/latest_calibration.yaml)
ros2 run camera_calibration verify_calibration_node.py
# с глубоким per-frame анализом по сохранённым кадрам
ros2 run camera_calibration verify_calibration_node.py --file <yaml> --captures <папка raw_*.jpg>
```

Анализирует: интринсики (fx/fy, соотношение), смещение главной точки, оценку FOV (H/V/D) и её соответствие модели, коэффициенты дисторсии, `reprojection_error` с буквенной оценкой и числом кадров. Код возврата: `0` — калибровка приемлема, `1` — рекомендуется переделать.

---

## Типичный сценарий

1. Запустить драйвер камеры (топик изображения публикуется).
2. Запустить ноду с конфигурацией доски и авто-деплоем (см. [Запуск](#запуск)).
3. `start_session`, затем набрать кадры (`capture` или `set_auto_capture`), двигая доску по всему кадру — для широкого угла обязательно по краям и углам под наклоном.
4. `calibrate` — выбросы отсеются сами; при `reproj <= auto_deploy_max_reproj` результат задеплоится.
5. Проверить результат: `verify_calibration_node.py`.
6. Пересобрать драйвер, если деплой шёл в `install`: `colcon build --packages-select <driver> && source install/setup.bash`.

---

## Инструменты и тесты

В каталоге `test/`:

- **verify_calibration.sh** — интеграционная проверка ноды (сервисы, захват, калибровка, деплой).
- **test_offline_units.py** — юнит-тесты чистых функций (без камеры/ROS).
- **recalibrate_offline.py** — сравнение pinhole/fisheye и отсев выбросов по сохранённым кадрам.
- **VERIFY_AGENT_PROMPT.md** — инструкция для агента-проверяющего.

Инструмент верификации: **src/verify_calibration_node.py** (`ros2 run camera_calibration verify_calibration_node.py`).

---

## Дополнительные сведения

- В `boards_examples_a4/` лежат доски для примеров выше.
- Если вызов сервиса возвращает `timeout` — нода ещё поднимается, попробуйте повторить.

---

*Документация актуальна для пакета `camera_calibration` в составе sverk_ws.*
