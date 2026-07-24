#!/usr/bin/env bash
# ============================================================================
# verify_calibration.sh — интеграционная проверка правок пакета camera_calibration
#
# Условия стенда: доска целиком видна в кадре, НО камера и доска неподвижны.
# Поэтому цель НЕ получить хорошую калибровку, а прогнать все ветки кода и
# убедиться, что нода:
#   - принимает изображение и детектирует мишень (единая детекция, правка №3);
#   - корректно ведёт учёт кадров и remove_capture;
#   - не падает на вырожденной калибровке (статичные кадры);
#   - на fisheye не валится из-за CALIB_CHECK_COND, а отбрасывает кадры (правка №1);
#   - деплоит YAML в config-папку драйвера с именем <sensor>_<w>x<h>.yaml (фича);
#   - отдаёт новые поля в /camera_calibration/state.
#
# Скрипт запускает СВОЙ экземпляр ноды (не трогает автозапуск full_system).
# Нужен только уже публикуемый топик изображения (запусти камеру заранее).
#
# Настройка через переменные окружения (значения по умолчанию — рабочий кейс imx219):
#   IMAGE_TOPIC   (default /camera_1/image_raw)
#   BOARD_TYPE    chessboard|charuco|circles|acircles   (default charuco)
#   BOARD_W BOARD_H SQUARE MARKER ARUCO_DICT SENSOR_NAME
#   N_CAPTURES    сколько кадров захватывать          (default 6)
#
# Запуск:
#   bash verify_calibration.sh
# Код возврата 0 — все проверки пройдены, иначе число проваленных.
# ============================================================================
set -u

IMAGE_TOPIC="${IMAGE_TOPIC:-/camera_1/image_raw}"
BOARD_TYPE="${BOARD_TYPE:-charuco}"
BOARD_W="${BOARD_W:-11}"
BOARD_H="${BOARD_H:-8}"
SQUARE="${SQUARE:-0.021}"
MARKER="${MARKER:-0.015}"
ARUCO_DICT="${ARUCO_DICT:-DICT_4X4_50}"
SENSOR_NAME="${SENSOR_NAME:-imx219}"
N_CAPTURES="${N_CAPTURES:-6}"

DEPLOY_DIR="$(mktemp -d /tmp/calib_deploy_test.XXXXXX)"
OUTPUT_DIR="$(mktemp -d /tmp/calib_out.XXXXXX)"
LOG="$(mktemp /tmp/calib_node.XXXXXX.log)"

PASS=0
FAIL=0
NODE_PID=""

log()  { echo -e "$*"; }
ok()   { PASS=$((PASS+1)); echo -e "  [PASS] $*"; }
bad()  { FAIL=$((FAIL+1)); echo -e "  [FAIL] $*"; }
hdr()  { echo -e "\n=== $* ==="; }

cleanup() {
  [ -n "$NODE_PID" ] && kill "$NODE_PID" 2>/dev/null
  wait "$NODE_PID" 2>/dev/null
}
trap cleanup EXIT

# --- helpers -----------------------------------------------------------------

# Вызов Trigger-сервиса. Возвращает 0, если success=True.
trig() {
  local svc="$1"
  ros2 service call "$svc" std_srvs/srv/Trigger 2>/dev/null | grep -q "success=True"
}

# Полный вывод Trigger-сервиса (для чтения message).
trig_msg() {
  ros2 service call "$1" std_srvs/srv/Trigger 2>/dev/null
}

# Извлечь поле из /camera_calibration/state (JSON в std_msgs/String.data).
state_field() {
  local key="$1"
  timeout 6 ros2 topic echo "/camera_calibration/state" std_msgs/msg/String --once --field data 2>/dev/null \
    | python3 -c "import sys,json
s=sys.stdin.read().strip()
try:
    d=json.loads(s)
except Exception:
    print(''); sys.exit()
v=d.get('$key','')
print(json.dumps(v) if isinstance(v,(dict,list)) else v)"
}

wait_for_service() {
  local svc="$1" i
  for i in $(seq 1 30); do
    ros2 service list 2>/dev/null | grep -q "^$svc$" && return 0
    sleep 1
  done
  return 1
}

# --- 0. предусловия ----------------------------------------------------------
hdr "0. Предусловия"

if ! command -v ros2 >/dev/null; then
  echo "ros2 не найден — source /opt/ros/humble/setup.bash и install/setup.bash"; exit 99
fi

log "Проверка топика изображения: $IMAGE_TOPIC"
if timeout 8 ros2 topic echo "$IMAGE_TOPIC" sensor_msgs/msg/Image --once >/dev/null 2>&1; then
  ok "изображение публикуется на $IMAGE_TOPIC"
else
  bad "нет данных на $IMAGE_TOPIC — запусти драйвер камеры и повтори"
  echo "Итог: предусловие не выполнено, дальше нет смысла."; exit 98
fi

# --- запуск тестового экземпляра ноды ---------------------------------------
hdr "Запуск тестового экземпляра camera_calibration"
ros2 launch camera_calibration calibration.launch.py \
  image_topic:="$IMAGE_TOPIC" \
  output_dir:="$OUTPUT_DIR" \
  sensor_name:="$SENSOR_NAME" \
  auto_deploy:=false \
  deploy_config_dir:="$DEPLOY_DIR" \
  > "$LOG" 2>&1 &
NODE_PID=$!
log "PID=$NODE_PID, лог: $LOG"

if wait_for_service "/calibration/get_state"; then
  ok "нода поднялась, сервисы доступны"
else
  bad "сервисы не появились за 30с — см. $LOG"; exit 97
fi

# --- 1. конфигурация и валидация --------------------------------------------
hdr "1. set_config: валидация входа"

# невалидный board_type должен отклоняться
if ros2 service call /calibration/set_config camera_calibration/srv/SetCalibrationConfig \
   "{board_type: 'nonsense', camera_model: 'pinhole', board_width: 9, board_height: 6, square_size: 0.02, marker_size: 0.0, aruco_dict: '', min_frames: 5, save_captures: true}" \
   2>/dev/null | grep -q "success=False"; then
  ok "невалидный board_type отклонён"
else
  bad "невалидный board_type НЕ отклонён"
fi

# валидная конфигурация под наш стенд
CFG="{board_type: '$BOARD_TYPE', camera_model: 'pinhole', board_width: $BOARD_W, board_height: $BOARD_H, square_size: $SQUARE, marker_size: $MARKER, aruco_dict: '$ARUCO_DICT', min_frames: 5, save_captures: true}"
if ros2 service call /calibration/set_config camera_calibration/srv/SetCalibrationConfig "$CFG" 2>/dev/null | grep -q "success=True"; then
  ok "валидная конфигурация принята ($BOARD_TYPE ${BOARD_W}x${BOARD_H})"
else
  bad "валидная конфигурация отклонена — проверь параметры доски"
fi

# --- 2. сессия и захват (единая детекция, правка №3) ------------------------
hdr "2. Сессия и захват кадров"

trig /calibration/start_session && ok "start_session" || bad "start_session"

captured=0
for i in $(seq 1 "$N_CAPTURES"); do
  if trig /calibration/capture; then
    captured=$((captured+1))
  fi
  sleep 0.4
done
log "успешных capture: $captured из $N_CAPTURES"

frames="$(state_field frames_captured)"
if [ "$frames" = "$captured" ] && [ "$captured" -ge 3 ]; then
  ok "frames_captured=$frames согласован с числом успешных захватов (детекция работает)"
elif [ "$captured" -eq 0 ]; then
  bad "ни один кадр не захвачен — доска не детектируется (проверь тип/размеры доски)"
else
  bad "рассинхрон: frames_captured=$frames, успешных capture=$captured"
fi

# захват без сессии не должен ломать (сначала reset)
trig /calibration/reset_session && ok "reset_session" || bad "reset_session"
if ros2 service call /calibration/capture std_srvs/srv/Trigger 2>/dev/null | grep -q "success=False"; then
  ok "capture без активной сессии корректно отклонён"
else
  bad "capture без сессии должен возвращать success=False"
fi

# --- 3. remove_capture: согласованность списков -----------------------------
hdr "3. remove_capture"
trig /calibration/start_session >/dev/null
for i in $(seq 1 4); do trig /calibration/capture >/dev/null; sleep 0.4; done
before="$(state_field frames_captured)"
if ros2 service call /calibration/remove_capture camera_calibration/srv/RemoveCapture "{index: 0}" 2>/dev/null | grep -q "success=True"; then
  after="$(state_field frames_captured)"
  if [ "$after" = "$((before-1))" ]; then
    ok "remove_capture уменьшил frames_captured $before -> $after"
  else
    bad "remove_capture: ожидалось $((before-1)), получено $after"
  fi
else
  bad "remove_capture не выполнился (before=$before)"
fi
# невалидный индекс
if ros2 service call /calibration/remove_capture camera_calibration/srv/RemoveCapture "{index: 999}" 2>/dev/null | grep -q "success=False"; then
  ok "remove_capture с невалидным индексом отклонён"
else
  bad "remove_capture с индексом 999 должен вернуть success=False"
fi

# --- 4. pinhole калибровка (вырожденная, но не должна ронять ноду) ----------
hdr "4. Калибровка pinhole (статичная сцена — вырожденная)"
# доберём кадры до min_frames
for i in $(seq 1 4); do trig /calibration/capture >/dev/null; sleep 0.4; done
CAL_OUT="$(trig_msg /calibration/calibrate)"
echo "$CAL_OUT" | grep -q "success=" && ok "calibrate вернул корректный ответ (нода не упала)" || bad "calibrate не ответил"
if echo "$CAL_OUT" | grep -q "success=True"; then
  log "  pinhole: $(echo "$CAL_OUT" | grep -o "message='[^']*'")"
  PINHOLE_OK=1
else
  log "  pinhole калибровка не сошлась (ожидаемо для статичной сцены): $(echo "$CAL_OUT" | grep -o "message='[^']*'")"
  PINHOLE_OK=0
fi

# --- 5. manual deploy: имя файла и валидность YAML --------------------------
hdr "5. Ручной деплой /calibration/deploy"
DEP_OUT="$(trig_msg /calibration/deploy)"
if [ "${PINHOLE_OK:-0}" = "1" ]; then
  if echo "$DEP_OUT" | grep -q "success=True"; then
    DEPLOYED="$(ls "$DEPLOY_DIR"/${SENSOR_NAME}_*.yaml 2>/dev/null | head -1)"
    if [ -n "$DEPLOYED" ]; then
      ok "файл задеплоен: $(basename "$DEPLOYED")"
      # имя должно быть <sensor>_<w>x<h>.yaml
      if basename "$DEPLOYED" | grep -Eq "^${SENSOR_NAME}_[0-9]+x[0-9]+\.yaml$"; then
        ok "имя файла соответствует соглашению <sensor>_<w>x<h>.yaml"
      else
        bad "имя файла не по шаблону: $(basename "$DEPLOYED")"
      fi
      # валидность YAML и ключевые поля
      if python3 -c "import yaml,sys
d=yaml.safe_load(open('$DEPLOYED'))
need=['image_width','image_height','camera_matrix','distortion_coefficients','projection_matrix']
assert all(k in d for k in need), 'missing keys'
assert len(d['camera_matrix']['data'])==9, 'K не 3x3'
assert d['image_width']>0 and d['image_height']>0
print('yaml OK', d['image_width'],'x',d['image_height'])" ; then
        ok "YAML валиден и содержит camera_matrix/projection_matrix/размеры"
      else
        bad "YAML невалиден или отсутствуют обязательные поля"
      fi
    else
      bad "деплой сообщил success, но файл в $DEPLOY_DIR не найден"
    fi
  else
    bad "deploy вернул success=False при успешной калибровке: $DEP_OUT"
  fi
else
  # калибровка не прошла — deploy обязан честно отказать
  if echo "$DEP_OUT" | grep -q "success=False"; then
    ok "deploy без валидной калибровки корректно отклонён (Calibration not performed)"
  else
    bad "deploy должен был отказать (калибровки нет), а вернул: $DEP_OUT"
  fi
fi

# --- 6. fisheye: отброс вырожденных кадров (правка №1) ----------------------
hdr "6. Калибровка fisheye (проверка отброса ill-conditioned)"
ros2 service call /calibration/set_config camera_calibration/srv/SetCalibrationConfig \
  "{board_type: '$BOARD_TYPE', camera_model: 'fisheye', board_width: $BOARD_W, board_height: $BOARD_H, square_size: $SQUARE, marker_size: $MARKER, aruco_dict: '$ARUCO_DICT', min_frames: 5, save_captures: false}" \
  >/dev/null 2>&1
trig /calibration/start_session >/dev/null
for i in $(seq 1 8); do trig /calibration/capture >/dev/null; sleep 0.4; done
FISH_OUT="$(trig_msg /calibration/calibrate)"
if echo "$FISH_OUT" | grep -q "success="; then
  ok "fisheye calibrate ответил, нода жива (не упала на CALIB_CHECK_COND)"
  MSG="$(echo "$FISH_OUT" | grep -o "message='[^']*'")"
  log "  fisheye: $MSG"
  if echo "$MSG" | grep -qi "ill-conditioned\|too few usable\|Calibrated (fisheye)"; then
    ok "результат fisheye из ожидаемого набора (отброс кадров / мало годных / успех)"
  else
    log "  (примечание) неожиданный текст сообщения fisheye — проверь вручную"
  fi
else
  bad "fisheye calibrate не ответил корректно"
fi

# --- 7. новые поля состояния -------------------------------------------------
hdr "7. Поля состояния (фича авто-деплоя)"
STATE_DUMP="$(timeout 6 ros2 topic echo /camera_calibration/state std_msgs/msg/String --once --field data 2>/dev/null)"
for k in last_deployed_yaml auto_deploy deploy_config_dir sensor_name; do
  if echo "$STATE_DUMP" | grep -q "\"$k\""; then
    ok "состояние содержит поле $k"
  else
    bad "в /camera_calibration/state нет поля $k"
  fi
done

# --- 8. нода всё ещё жива ----------------------------------------------------
hdr "8. Живучесть ноды"
if kill -0 "$NODE_PID" 2>/dev/null && ros2 node list 2>/dev/null | grep -q camera_calibration; then
  ok "нода camera_calibration жива после всех сценариев"
else
  bad "нода упала/исчезла — см. $LOG"
fi

# --- итог --------------------------------------------------------------------
hdr "ИТОГ"
echo "PASS=$PASS  FAIL=$FAIL"
echo "Лог ноды: $LOG"
echo "Деплой-каталог: $DEPLOY_DIR"
echo "Каталог калибровок: $OUTPUT_DIR"
[ "$FAIL" -eq 0 ] && echo "РЕЗУЛЬТАТ: ОК — можно пробовать полноценную калибровку с движением доски." \
                  || echo "РЕЗУЛЬТАТ: есть проваленные проверки, разбери [FAIL] выше."
exit "$FAIL"
