# SVERK SITL — симулятор дрона

Симулятор дрона SVERK (PX4 + Gazebo Harmonic + ROS 2) в готовом Docker-образе:
скачиваете и запускаете, ничего собирать не нужно. Внутри — тот же полётный
стек, что на реальном дроне: навигация по ArUco, дальномеры, камера,
взлёт/посадка по сервису.

## Быстрый старт

```bash
cd compose
cp .env.example .env          # Windows: Copy-Item .env.example .env
./sim.sh up novnc             # Windows: .\sim.ps1 up novnc
```

Откройте <http://localhost:6080/vnc.html>, войдите в контейнер
(`./sim.sh shell`) и запустите симулятор:

```bash
ros2 launch launch_system full_system_sitl.launch.py \
  vnc_gui:=true obrik_config:=base gz_world:=obrik_aruco
```

Полное руководство (режимы отображения, модели и миры, навигация, полёт,
проверки, аддоны Web-IDE и MCP, траблшутинг) — в
**[docs/guide.md](docs/guide.md)**.

## Структура

```
simulation/
├── compose/                          # docker compose + скрипты запуска
│   ├── docker-compose.sim.yml        # база (headless)
│   ├── docker-compose.sim.novnc.yml  # noVNC — картинка в браузере (любой ноут)
│   ├── docker-compose.sim.wslg.yml + .wslg.nvidia.yml / .wslg.intel.yml   # Windows GPU-окно
│   ├── docker-compose.sim.linux-host.yml   # нативный Linux X11
│   ├── docker-compose.sim.ide.yml    # Web-IDE (VS Code + терминал в браузере)
│   ├── docker-compose.sim.mcp.yml    # MCP HTTP-сервер :8765
│   ├── mcp_policy.yaml                # права MCP
│   ├── .env.example                  # теги образов
│   ├── sim.ps1                        # обёртка для Windows
│   └── sim.sh                         # обёртка для Linux/WSL/macOS
└── docs/
    └── guide.md                      # руководство пользователя
```

## Режимы одним взглядом

| Режим | Команда (из `compose/`) | Картинка |
|---|---|---|
| noVNC (рекомендуется) | `./sim.sh up novnc` | браузер :6080 |
| WSLg + NVIDIA | `./sim.sh up wslg-nvidia` | окно на рабочем столе |
| WSLg + Intel Arc | `./sim.sh up wslg-intel` | окно (или noVNC) |
| Linux X11 | `./sim.sh up linux-host` | окно на рабочем столе |
| headless | `./sim.sh up headless` | нет окна |

Аддоны (добавляются к режиму): `./sim.sh up novnc mcp`, `./sim.sh up headless ide+mcp`.
