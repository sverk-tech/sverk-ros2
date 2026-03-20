## Сборка и установка локальных версий контейнеров

В этом разделе описаны два способа получения Docker-образов для Raspberry Pi (архитектура `arm64`):  
- **полная сборка** образа с компиляцией всех пакетов;  
- **загрузка готовых образов** с Google Disk (быстрый старт).

---

### 1. Сборка образа вручную

Этот метод создаёт готовый образ, который после загрузки на Raspberry Pi сразу содержит все собранные пакеты. Подходит для установки на несколько дронов без повторной компиляции.

**Предварительные требования**  
- Docker Desktop установлен и запущен.  
- Достаточно ресурсов: рекомендуется не менее 8 ГБ RAM и 50 ГБ свободного места на диске.  
- Все команды выполняются из **корня репозитория** `~/sverk-ros2`.

#### 1.1 Подготовка Dockerfile

Перед сборкой отредактируйте файл `scripts/Dockerfile` – закомментируйте строки, отвечающие за сборку рабочего пространства. Это необходимо, чтобы сборка происходила на этапе создания образа, а не при каждом запуске контейнера.

```dockerfile
# USER sverk
# RUN bash -c "source /opt/ros/humble/setup.bash && colcon build"
# USER root
```

#### 1.2 Проверка архитектуры и поддержки сборки  
Убедитесь, что Docker может собирать образы для `linux/arm64`.

```bash
docker info --format '{{.Architecture}} / {{.OSType}}'
docker buildx ls
```

В выводе `docker buildx ls` должна присутствовать платформа `linux/arm64`.

#### 1.2 Включение эмуляции ARM64 (binfmt)  
Если вы ещё не настраивали эмуляцию, выполните однократно:

```bash
docker run --privileged --rm tonistiigi/binfmt --install arm64
```

Проверьте, что эмуляция работает:

```bash
docker run --rm --platform linux/arm64 alpine uname -m
# Ожидается вывод: aarch64
```

#### 1.3. Создание buildx‑билдера для ARM64  
Если билдер с именем `sverk-arm64` ещё не создан, выполните:

```bash
docker buildx create --name sverk-arm64 --use
docker buildx inspect --bootstrap
```

#### 1.4 Сборка образа

Выполните сборку образа для архитектуры `linux/arm64`. Можно использовать созданный билдер или стандартный (default). Если вылезла ошибка перезапустите сборку.

```bash
# Через созданный билдер
docker buildx build --no-cache --builder sverk-arm64 --platform linux/arm64 -t sverk/ros2:latest --progress=plain -f scripts/Dockerfile --load .

# Или через билдер по умолчанию
docker buildx build --platform linux/arm64 --builder default -t sverk/ros2:latest -f scripts/Dockerfile --load .
```

#### 1.5 Сохранение образа в архив

После успешной сборки упакуйте образ в сжатый tar-архив для последующего переноса на Raspberry Pi.

```bash
docker save sverk/ros2:latest | gzip > sverk_ros2.tar.gz
```
**5. Проверка успешной сборки**

```bash
docker image inspect sverk/ros2:latest --format '{{.Architecture}}'
# Ожидается вывод: arm64 или aarch64
```
---

### 2 Скачивание готовых образов с Google Disk

Если вы не хотите собирать образ самостоятельно, можно скачать уже готовые образы с Google Disk. Для этого потребуется установить утилиту `gdown`.

#### 2.1 Установка gdown

Создайте виртуальное окружение Python и установите `gdown`:

```bash
python3 -m venv myenv
source myenv/bin/activate
pip install gdown
```

#### 2.2 Скачивание образов

Выполните команды для загрузки актуальных версий образов (идентификаторы файлов указаны в комментариях):

```bash
gdown 1oWuu9UIKgHNgm5VeSwjphahMcLeB5UKy   # v1.6 ros2 image
gdown 1nSK1h7qzCXVCGm3AOou3-q-FCqpmhVC1   # v1.7 ros2 image
gdown 11D8Y0LNlA7qJ5i0Xqsyp8CyQrFu76BO7   # frontend image
gdown 10ZTR5yLqHFSMDb-5swCqniIg7Xbyb-pL   # dotmd image
gdown 1MVS8e4YGN0U209VPqC-rJln-vU4dgaQb   # flight_review image
```

```bash
deactivate
```

#### 2.3 Загрузка образов в Docker

Загрузите скачанные образы в локальное хранилище Docker:

```bash
gunzip -c sverk_* | docker load
```

Эта команда распакует все файлы, начинающиеся с `sverk_`, и загрузит их в Docker.

---

### 3 Первый запуск контейнера и сборка рабочего пространства

После того как образ загружен (или собран), запустите контейнер, как описано в разделе **3. Подготовка Raspberry Pi** (команда `docker compose up --no-start --build`). При первом запуске внутри контейнера необходимо дополнительно собрать рабочее пространство ROS 2, так как на этапе создания образа этот шаг был пропущен.

Подключитесь к контейнеру и выполните сборку:

```bash
# Войдите в контейнер
docker exec -it sverk_ros2 bash

# Перейдите в рабочее пространство и выполните сборку
cd ~/sverk_ws
colcon build

# Активируйте окружение (необязательно, если используется start.sh)
source install/setup.bash
```

Сборка `colcon` выполняется только один раз, после первого запуска контейнера. При последующих запусках она не требуется.

---

### 4 Проверка

Убедитесь, что все образы успешно загружены и контейнер работает корректно, выполнив на Raspberry Pi:

```bash
bash ~/sverk-ros2/scripts/check.sh
```