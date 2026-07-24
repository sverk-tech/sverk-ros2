## Диагностика дрона (self_check)

`self_check` — нода-оркестратор диагностики дрона. Раньше это был
одноразовый CLI-скрипт (`selfcheck.py`); теперь это постоянно работающая
ROS-нода с action-API, каталогом из **пассивных** (сами по топикам/
параметрам) и **интерактивных** (с инструкциями пользователю вида «покрутите
дрон», «снимите пропеллеры») проверок и структурированным JSON-отчётом.
Это серверная половина вкладки «Проверка системы» веб-QGC (`web/qgc`) —
веб слой рендерит то, что отдаёт эта нода, вся логика проверок живёт здесь.

Полный контракт интерфейсов и схем — `docs/dev/tz/system_check/00_contract.md`.

### Архитектура

```
self_check_interfaces/           # srv/action (ament_cmake, отдельный пакет)
  srv/ListChecks.srv             # -> список манифестов ИЛИ последний отчёт
  srv/UserInput.srv               # ответ пользователя на WAITING_USER-шаг
  action/RunChecks.action         # запуск прогона + live-feedback

self_check/                       # ament_python
  config/
    default.yaml                  # пороги + топики по умолчанию
    rpi.yaml radxa.yaml rdx.yaml orange.yaml orange_3w.yaml   # оверрайды по платформе
  self_check/
    node.py                       # нода-оркестратор: action + сервисы
    context.py                    # CheckContext — API, которое видит каждая проверка
    report.py                     # CheckResult, сборка report_json
    profile.py                    # deep-merge default.yaml + config/<platform>.yaml
    util.py                       # чистые пороговые функции (кватернионы, банки, VPE...)
    cli.py                        # CLI-клиент action (ros2 run self_check selfcheck)
    checks/
      base.py                     # BaseCheck
      __init__.py                 # REGISTRY = {id: BaseCheck-инстанс}
      link.py power.py sensors.py vision.py motors.py sbc.py prearm.py
  test/                           # pytest на чистую логику (без запущенной ROS-графа)
```

Нода **никогда не армит** дрон и не пишет параметры PX4 (только читает через
`/fmu_control/get_param`).

### ROS API

Нода: `self_check`.

| Интерфейс | Имя | Назначение |
|---|---|---|
| action `RunChecks` | `/self_check/run` | запустить прогон, стримить feedback, вернуть `report_json` |
| srv `ListChecks` | `/self_check/list` | манифесты всех проверок каталога (доступность — по профилю платформы) |
| srv `ListChecks` | `/self_check/report` | последний отчёт в `checks_json` (`"null"`, если прогонов не было) |
| srv `UserInput` | `/self_check/user_input` | ответ на текущий шаг `WAITING_USER` (`ok\|yes\|no\|skip`) |

`RunChecks.Goal`:

- `check_ids` — пусто = все применимые для платформы.
- `interactive` — `false` = только пассивные проверки (быстрый прогон).
- `confirm` — фраза `I_UNDERSTAND_PROPS_ARE_OFF`, обязательна, если в списке
  `motor_spin` (иначе проверка мгновенно завершается `FAIL`, моторы не трогаются).

Один прогон одновременно — второй `goal` во время активного прогона
отклоняется (`GoalResponse.REJECT`). Отмена goal (`cancel`) переводит прогон
в `aborted`: текущая проверка донашивается до своего терминального состояния,
`motor_spin` в любом случае (успех/отмена/исключение) шлёт
`/fmu_control/stop_all_motors`.

Порядок прогона: сначала все пассивные проверки из запрошенных (могут
исполняться параллельно, пул 3–4 потока), затем — если `interactive=true` —
интерактивные строго последовательно.

### Каталог проверок

ID проверок фиксированы контрактом (их знают и нода, и веб-UI) — см. таблицы
в `00_contract.md §4`. Кратко по категориям:

- **link** — `link_fmu`, `offboard_status`, `rc_sticks` (интерактивная)
- **power** — `power_battery`
- **sensors** — `calib_params`, `attitude_level`, `peripherals`,
  `hold_still`, `orientation_tilt`, `yaw_rotation`, `raise_lower` (интерактивные)
- **vision** — `camera_stream`, `aruco_pipeline`, `vpe_consistency`,
  `aruco_show_map`, `move_1m` (интерактивные)
- **motors** — `motor_spin` (интерактивная, **опасная**)
- **sbc** — `sbc_health`, `ros_graph`, `web_services`, `led_test` (интерактивная)
- **prearm** — `prearm_failsafe`, `estimator_flags`

Каждая проверка завершается одним из `PASS | WARN | FAIL | SKIP`. `SKIP`
означает: подсистема отключена в профиле платформы, зависимый сервис/нода
недоступны (например `fmu_control` не запущена — `calib_params` не FAIL, а
SKIP), либо пользователь явно нажал «skip».

### Профиль платформы

`config/default.yaml` задаёт топики/пороги/фичи по умолчанию,
`config/<platform>.yaml` — только реальные отличия (deep-merge поверх
default). Платформа выбирается параметром ноды `platform`
(`RPI | RADXA | RDX | ORANGE | ORANGE_3W`, как в `full_system_real.launch.py`
/ `MK_NAME`); `profile_file` — путь к произвольному YAML вместо файла платформы.

```bash
ros2 run self_check self_check_node --ros-args -p platform:=RPI
```

### CLI-клиент

```bash
# Быстрый прогон (только пассивные проверки)
ros2 run self_check selfcheck

# Конкретные проверки
ros2 run self_check selfcheck --only power_battery,link_fmu,sbc_health

# Полный интерактивный прогон с вопросами через stdin
ros2 run self_check selfcheck --interactive --confirm-props-off

# Сохранить report_json в файл
ros2 run self_check selfcheck --json /tmp/selfcheck_report.json
```

Флаги: `--only id,id`, `--interactive` (включает интерактивные проверки и
задаёт вопросы через stdin), `--confirm-props-off` (подтверждение для
`motor_spin`), `--json out.json`. `WARN`/`FAIL` — красным в выводе, в конце —
сводка `pass/warn/fail/skip` и exit-код (`0`, если нет `FAIL`).

### Как добавить новую проверку

1. Определить класс-наследник `BaseCheck` в подходящем `checks/<категория>.py`
   (`id`, `category`, `title`, `description`, при необходимости `interactive`/
   `dangerous`, метод `available(profile)` и обязательный `run(ctx)`).
2. В `run(ctx)` пользоваться `CheckContext` (`context.py`): `wait_for_message`,
   `wait_for_updates`, `measure_rate`, `collect`, `collect_multi`,
   `subscribe_latest`, `call_service`, `report(level, text)`, `metric(key,
   value)`, для интерактивных — `instruct(text, metrics)` и
   `wait_user(text, options, timeout_s)`. В конце выставить
   `ctx.result.state` в `report.STATE_PASS/WARN/FAIL/SKIP`.
3. Добавить инстанс в список `CHECKS` того же файла — реестр
   (`checks/__init__.py`) подхватит его автоматически (падает с `RuntimeError`
   при дублирующемся `id`).
4. Если проверка вводит новые пороги/топики — добавить их в
   `config/default.yaml`, при необходимости переопределить в платформенных
   YAML.
5. Обновить контракт `docs/dev/tz/system_check/00_contract.md §4`, если id —
   часть межкомандного соглашения (веб-UI и dds-бэкенд его тоже используют).

### Тесты

```bash
cd sverk_ws/src/sverk_drone/main_package/self_check
python3 -m pytest test/
```

`test_util.py`, `test_profile.py`, `test_report.py` не зависят от rclpy/ROS.
`test_registry.py` импортирует сгенерированные типы сообщений (`px4_msgs`,
`offboard_interfaces`, `led_interfaces`, `aruco_det_loc`,
`self_check_interfaces`) и поэтому пропускается (`importorskip`), если
воркспейс не собран colcon-ом.
