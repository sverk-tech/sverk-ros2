# SVERK SITL — обёртка над docker compose (Windows PowerShell).
# Запускать из папки compose/.
#
#   .\sim.ps1 up   novnc          # поднять noVNC-режим (рекомендуется)
#   .\sim.ps1 up   wslg-nvidia    # WSLg + NVIDIA (только из Ubuntu-24.04!)
#   .\sim.ps1 up   wslg-intel     # WSLg + Intel Arc
#   .\sim.ps1 up   headless       # без картинки (только физика)
#   .\sim.ps1 up   headless ide   # + Web-IDE (VS Code :5467 / терминал :8575)
#                                 #   аддон ide сочетается со всеми режимами, КРОМЕ novnc
#   .\sim.ps1 up   novnc   mcp    # + MCP-сервер :8765 (сочетается с любым режимом)
#   .\sim.ps1 up   headless ide+mcp   # несколько аддонов через +
#   .\sim.ps1 shell               # войти в контейнер (docker exec bash)
#   .\sim.ps1 down  headless ide  # остановить (режим и аддон — те же, что при up)
#   .\sim.ps1 pull                # docker pull образов
#   .\sim.ps1 logs
param(
    [Parameter(Mandatory = $true)][string]$Command,
    [string]$Mode = "novnc",
    [string]$Addon = ""
)

$ErrorActionPreference = "Stop"
Set-Location -Path $PSScriptRoot

function Files($m) {
    switch ($m) {
        "novnc"       { @("docker-compose.sim.yml", "docker-compose.sim.novnc.yml") }
        "wslg-nvidia" { @("docker-compose.sim.yml", "docker-compose.sim.wslg.yml", "docker-compose.sim.wslg.nvidia.yml") }
        "wslg-intel"  { @("docker-compose.sim.yml", "docker-compose.sim.wslg.yml", "docker-compose.sim.wslg.intel.yml") }
        "linux-host"  { @("docker-compose.sim.yml", "docker-compose.sim.linux-host.yml") }
        "headless"    { @("docker-compose.sim.yml") }
        default       { throw "Неизвестный режим '$m'. Доступно: novnc | wslg-nvidia | wslg-intel | linux-host | headless" }
    }
}

function ComposeArgs {
    $fs = Files $Mode
    foreach ($ad in ($Addon -split '[+ ]')) {
        switch ($ad) {
            "ide" {
                if ($Mode -eq "novnc") { throw "Аддон ide несовместим с novnc (оба меняют образ). В noVNC терминал открывается прямо в рабочем столе :6080." }
                $fs += "docker-compose.sim.ide.yml"
            }
            "mcp" { $fs += "docker-compose.sim.mcp.yml" }
            "" { }
            default { throw "Неизвестный аддон '$ad'. Доступно: ide | mcp (или ide+mcp)" }
        }
    }
    $a = @("compose")
    foreach ($f in $fs) { $a += @("-f", $f) }
    return $a
}

if (-not (Test-Path ".env")) {
    Write-Host "-> .env не найден, копирую из .env.example" -ForegroundColor Yellow
    Copy-Item ".env.example" ".env"
}

switch ($Command) {
    "up" {
        & docker (ComposeArgs) up -d
        if ($Mode -eq "novnc")   { Write-Host "noVNC: http://localhost:6080/vnc.html" -ForegroundColor Green }
        if ($Addon -like "*ide*") { Write-Host "Web-IDE: http://localhost:5467  |  терминал: http://localhost:8575" -ForegroundColor Green }
        if ($Addon -like "*mcp*") { Write-Host "MCP: http://localhost:8765/healthz" -ForegroundColor Green }
    }
    "down"  { & docker (ComposeArgs) down }
    "pull"  { & docker (ComposeArgs) pull }
    "logs"  { & docker (ComposeArgs) logs -f }
    "shell" { & docker exec -it sverk_sitl bash }
    default { throw "Неизвестная команда '$Command'. Доступно: up | down | pull | logs | shell" }
}
