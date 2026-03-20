# VS Code Server Installer

Быстрая установка и запуск VS Code в браузере на Linux.

## Быстрый старт

```bash
chmod +x install-code-server.sh
./install-code-server.sh
```

Готово! Откройте `http://localhost:8080` в браузере.

---

## Как это работает

1. Скрипт устанавливает code-server
2. Автоматически создаёт конфигурацию
3. Включает автозапуск через systemd

---

## Доступ из другого устройства

```bash
# Узнайте IP
hostname -I

# Откройте в браузере другого устройства:
# http://<ВАШ_IP>:8080
```

---

## Управление

```bash
# Запустить
systemctl --user start code-server

# Остановить
systemctl --user stop code-server

# Статус
systemctl --user status code-server

# Логи
journalctl --user -u code-server -f
```

---

## Настройка пароля

Отредактируйте файл:
```bash
nano ~/.config/code-server/config.yaml
```

Измените строку `password: ` на свой пароль.
