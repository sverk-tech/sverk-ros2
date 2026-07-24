#!/usr/bin/env bash
# SVERK SITL — обёртка над docker compose (Linux / WSL / macOS).
# Запускать из папки compose/.
#
#   ./sim.sh up   novnc          # поднять noVNC-режим (рекомендуется)
#   ./sim.sh up   wslg-nvidia    # WSLg + NVIDIA (только из Ubuntu-24.04!)
#   ./sim.sh up   wslg-intel     # WSLg + Intel Arc
#   ./sim.sh up   linux-host     # нативный Linux X11 (сначала: xhost +local:docker)
#   ./sim.sh up   headless       # без картинки (только физика)
#   ./sim.sh up   headless ide   # + Web-IDE (VS Code :5467 / терминал :8575)
#                                #   аддон ide сочетается со всеми режимами, КРОМЕ novnc
#   ./sim.sh up   novnc   mcp    # + MCP-сервер :8765 (сочетается с любым режимом)
#   ./sim.sh up   headless ide+mcp   # несколько аддонов через +
#   ./sim.sh shell               # войти в контейнер (docker exec bash)
#   ./sim.sh down headless ide   # остановить (режим и аддон — те же, что при up)
#   ./sim.sh pull                # docker pull образов
#   ./sim.sh logs
set -euo pipefail
cd "$(dirname "$0")"

CMD="${1:-}"
MODE="${2:-novnc}"
ADDON="${3:-}"

files() {
  case "$1" in
    novnc)       echo "-f docker-compose.sim.yml -f docker-compose.sim.novnc.yml" ;;
    wslg-nvidia) echo "-f docker-compose.sim.yml -f docker-compose.sim.wslg.yml -f docker-compose.sim.wslg.nvidia.yml" ;;
    wslg-intel)  echo "-f docker-compose.sim.yml -f docker-compose.sim.wslg.yml -f docker-compose.sim.wslg.intel.yml" ;;
    linux-host)  echo "-f docker-compose.sim.yml -f docker-compose.sim.linux-host.yml" ;;
    headless)    echo "-f docker-compose.sim.yml" ;;
    *) echo "Неизвестный режим '$1'. Доступно: novnc | wslg-nvidia | wslg-intel | linux-host | headless" >&2; exit 1 ;;
  esac
}

compose_files() {
  local f; f="$(files "$MODE")"
  local a
  for a in ${ADDON//+/ }; do
    case "$a" in
      ide)
        [ "$MODE" = "novnc" ] && { echo "Аддон ide несовместим с novnc (оба меняют образ). В noVNC терминал открывается прямо в рабочем столе :6080." >&2; exit 1; }
        f="$f -f docker-compose.sim.ide.yml" ;;
      mcp) f="$f -f docker-compose.sim.mcp.yml" ;;
      "" ) ;;
      *) echo "Неизвестный аддон '$a'. Доступно: ide | mcp (или ide+mcp)" >&2; exit 1 ;;
    esac
  done
  echo "$f"
}

[ -f .env ] || { echo "-> .env не найден, копирую из .env.example"; cp .env.example .env; }

case "$CMD" in
  up)    docker compose $(compose_files) up -d
         [ "$MODE" = "novnc" ] && echo "noVNC: http://localhost:6080/vnc.html"
         case "$ADDON" in *ide*) echo "Web-IDE: http://localhost:5467  |  терминал: http://localhost:8575" ;; esac
         case "$ADDON" in *mcp*) echo "MCP: http://localhost:8765/healthz" ;; esac ;;
  down)  docker compose $(compose_files) down ;;
  pull)  docker compose $(compose_files) pull ;;
  logs)  docker compose $(compose_files) logs -f ;;
  shell) docker exec -it sverk_sitl bash ;;
  *) echo "Неизвестная команда '$CMD'. Доступно: up | down | pull | logs | shell" >&2; exit 1 ;;
esac
