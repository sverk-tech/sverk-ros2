# ros_services_bridge

Пакет ROS2, который поднимает HTTP API для получения списка сервисов и вызова любого сервиса. Предназначен для работы в паре с **dotmdVisualiser**: веб-интерфейс обращается к этому мосту по сети (в Docker — по имени контейнера `sitl`).

## Сборка

Из корня workspace:

```bash
cd /home/sverk/sverk_ws
colcon build --packages-select ros_services_bridge
source install/setup.bash
```

## Запуск

В контейнере sitl (или на машине с ROS2):

```bash
source /home/sverk/sverk_ws/install/setup.bash
ros2 run ros_services_bridge ros_services_bridge_node
```

Параметр порта (по умолчанию 9090):

```bash
ros2 run ros_services_bridge ros_services_bridge_node --ros-args -p port:=9090
```

## API

- **GET /api/services** — список сервисов: `{"services": [{"name": "/service_name", "type": "pkg/srv/Type"}, ...]}`
- **POST /api/services/<service_name>/call** — вызов сервиса. Тело запроса: JSON с полями запроса (для пустого запроса: `{}`). Ответ: `{"success": true, "response": {...}}` или `{"success": false, "error": "..."}`

Имя сервиса в URL передаётся с экранированием (например `/my/srv` → `%2Fmy%2Fsrv`).

## Связка с dotmdVisualiser

- В `docker-compose` у сервиса `dotmdviz` задана переменная `ROS_SERVICES_BRIDGE_URL=http://sitl:9090`.
- На главной странице dotmdVisualiser добавлена ссылка «ROS-сервисы» → страница со списком сервисов и кнопкой «Вызвать» с вводом JSON-запроса.

Мост должен работать в том же контейнере (или в той же сети), где запущены остальные узлы ROS2 (контейнер `sitl`).
