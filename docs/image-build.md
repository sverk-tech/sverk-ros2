# Sverk ROS2 — Автоматическая сборка образов (multi-platform, multi-OS) (УСТАРЕЛО)

> ⚠️ **Устарело.** Документ описывает старый GitLab CI/CD-пайплайн
> (`.gitlab-ci.yml`, `scripts/image/`) — оба удалены. Сборка теперь только
> через веб-сборщик (Web_image_bulder) или локально `image_builder/test_local.sh`,
> движок описан в [`image_builder/README.md`](../image_builder/README.md).
> Документ оставлен для истории решений (почему устроено именно так), не как
> текущая инструкция запуска.

Здесь два уровня: **простое объяснение для запуска** (ниже) и **подробный
справочник для разработчика** (со «[Содержания](#содержание)»).

---

## Простыми словами: что это и что на выходе

CI/CD сам собирает готовые образы для SD-карты / eMMC. Всего может получиться
до **6 образов** = **3 платы × 2 варианта**.

**3 платы — каждая на своей операционной системе:**

| Плата | Чип | Операционная система |
|---|---|---|
| `rpi` — Raspberry Pi | BCM2712 | Raspberry Pi OS |
| `radxa` — Radxa CM5 | RK3588S | Radxa OS |
| `orange` — Orange Pi 5 Pro | RK3588S | Orange Pi OS |

**2 варианта каждого образа:**

| Вариант | Что внутри | Первый запуск | Размер скачивания |
|---|---|---|---|
| **full** | Все Docker-образы уже встроены | Без интернета, быстро | Больше |
| **minimal** | Docker-образов нет, скачиваются при первом включении | Нужен доступ к корпоративному реестру (Docker Hub Enterprise) | Меньше |

> Размер: `minimal` меньше, потому что в нём нет ~10 ГБ встроенных образов
> (пустое место в `.img.gz` сжимается почти в ноль). Сам раздел на карте всё
> равно расширяется — образам нужно место после скачивания.

---

## Что нужно заполнить перед запуском (ОБЯЗАТЕЛЬНО)

Заполняется **один раз** в GitLab: **Settings → CI/CD → Variables → Add variable**.
Для секретов ставьте галочки **Masked** (и **Protected**, если релизы идут только
с защищённых веток/тегов).

### 1. Ссылки на ОС для Radxa и Orange Pi

Без них **radxa и orange просто не будут собираться** (Raspberry Pi эти ссылки
уже имеет по умолчанию). Падения не будет — платы тихо пропускаются.

| Переменная | Что вписать | Пример |
|---|---|---|
| `RADXA_BASE_URL` | Прямая ссылка на образ Radxa OS **или** Google Drive | `https://dl.radxa.com/.../radxa-cm5_debian.img.xz` |
| `ORANGE_BASE_URL` | Прямая ссылка на образ Orange Pi OS **или** Google Drive | `gdrive:1AbCdEf...` |

Поддерживаются форматы: `http(s)://...img.xz` / `.gz` / `.7z` / `.zip` / `.img`,
а также Google Drive в виде `gdrive:<ID файла>` (ID берётся из ссылки вида
`https://drive.google.com/file/d/<ID>/view`). Скачивание и распаковку делает
[`fetch_base.sh`](#scriptsimagefetch_basesh).

При желании можно поменять и имена распакованных файлов: `RADXA_BASE_FILE`,
`ORANGE_BASE_FILE`, `RPI_BASE_URL`, `RPI_BASE_FILE` (необязательно).

### 2. Доступ к корпоративному реестру (для варианта `minimal`)

Без них **вариант `minimal` не создаётся** (создаётся только `full`). Это
нормально, если вам нужны только офлайн-образы.

| Переменная | Что вписать | По умолчанию |
|---|---|---|
| `COMPANY_REGISTRY` | Адрес реестра + namespace | встроенный GitLab-реестр `${CI_REGISTRY}/${CI_PROJECT_PATH}` |
| `COMPANY_REGISTRY_USER` | Логин (лучше read-only deploy-token) | логин CI `${CI_REGISTRY_USER}` |
| `COMPANY_REGISTRY_PASSWORD` | Пароль/токен (**Masked!**) | токен CI `${CI_REGISTRY_PASSWORD}` |

> Если используете встроенный GitLab Container Registry — обычно ничего вписывать
> не нужно, эти переменные подставляются автоматически. Свой внешний реестр
> (например `registry.sverk.io/sverk`) — задайте все три.
>
> ⚠️ Логин/пароль попадают в `minimal`-образ как обычный текст (firstboot
> логинится в реестр). Используйте **отдельный read-only deploy-token**, не
> личный пароль.

### 3. (необязательно) Прочее

| Переменная | Зачем | По умолчанию |
|---|---|---|
| `WIFI_SSID` / `WIFI_PASSWORD` / `WIFI_COUNTRY` | Авто-подключение к Wi-Fi | пусто / `GB` |
| `CAMERA_SENSOR` | `ov5647` или `imx219` (только RPi) | `ov5647` |
| `RPI_PASSWORD` | Пароль пользователя на устройстве | `raspberry` |
| `SKIP_REGISTRY_PUSH` | `1` = не пушить в реестр (тогда `minimal` тоже выключен) | `0` |
| `SKIP_PREBAKE` | `1` = быстрее сборка, но медленнее первый boot | `0` |

---

## Что и когда собирается (сценарии)

Пайплайн **не падает**, если что-то не заполнено — он просто **не собирает** то,
для чего нет данных.

| Что заполнено | Что соберётся |
|---|---|
| Ничего лишнего (только дефолты) | `rpi-full` + (если есть GitLab-реестр) `rpi-minimal`. Radxa/Orange — **пропущены**. |
| `RADXA_BASE_URL` | Добавятся `radxa-full` (+ `radxa-minimal`, если есть реестр) |
| `ORANGE_BASE_URL` | Добавятся `orange-full` (+ `orange-minimal`, если есть реестр) |
| Все 3 ссылки + рабочий реестр | Все **6 образов** |
| `SKIP_REGISTRY_PUSH=1` или нет кред реестра | Только `*-full`. Все `*-minimal` — **пропущены** |

Правила пропуска (в [`.gitlab-ci.yml`](#gitlab-ciyml), секции `rules:`):

- `radxa`/`orange` пропускаются, если соответствующий `*_BASE_URL` пуст —
  на стадиях `prepare`, `docker-build` и `img-build`.
- `*-minimal` пропускается, если `SKIP_REGISTRY_PUSH=1`, либо не заданы
  `COMPANY_REGISTRY_USER` / `COMPANY_REGISTRY_PASSWORD`, либо нет адреса реестра.
- `release:create` помечает зависимости как `optional` — релиз соберётся из тех
  образов, что реально получились (например 1, 2, 4 или 6 штук).

---

## Как запустить релиз

1. Заполнить переменные (раздел выше).
2. Поставить тег вида `vX.Y` и запушить:
   ```bash
   git tag v1.0 && git push origin v1.0
   ```
3. Пайплайн соберёт образы, зальёт `.img.gz` в Package Registry и создаст
   **GitLab Release** со ссылками на все получившиеся образы.

### Альфа / тестовые сборки

Чтобы собрать **тестовую (альфа) версию**, поставьте тег с суффиксом после
`vX.Y` — например `v1.8a`, `v1.8-alpha`, `v1.8-rc1`, `v1.8-test`:

```bash
git tag v1.8a && git push origin v1.8a
```

Пайплайн отработает **точно так же** (соберёт образы, зальёт в Package Registry),
но Release будет **помечен как «⚠️ ALPHA / тест»** в названии и описании — чтобы
не путать с продакшеном. Docker-теги внутри образа тоже получат версию из тега
(`...:v1.8a-rpi` и т.д.), так что альфа не перетирает стабильные образы в реестре.

Как это определяется (в [`release:create`](#gitlab-ciyml)):

| Тег | Тип релиза |
|---|---|
| `v1.8`, `v1.8.2` (чистый `vX.Y` / `vX.Y.Z`) | стабильный |
| `v1.8a`, `v1.8-alpha`, `v1.8-rc1`, `v1.8-test`, … | альфа / тест |

> Тег должен начинаться с `vX.Y` (например `v1.8…`). Имена вроде `alpha-1.7`
> или `test` пайплайн не подхватит.

Локально (без GitLab) — см. [`test_local.sh`](#scriptsimagetest_localsh).

---

## Подробности для разработчика

Дальше — **технический справочник**: каждый скрипт, его входы/выходы, точки
расширения и диагностика. `01–04` сделаны board-aware (разметка определяется
динамически через [`lib/board.sh`](#scriptsimagelibboardsh), специфика RPi
применяется только при `BOARD_FAMILY=rpi`).

## Содержание

- [Простыми словами: что это и что на выходе](#простыми-словами-что-это-и-что-на-выходе)
- [Что нужно заполнить перед запуском (ОБЯЗАТЕЛЬНО)](#что-нужно-заполнить-перед-запуском-обязательно)
- [Что и когда собирается (сценарии)](#что-и-когда-собирается-сценарии)
- [Как запустить релиз](#как-запустить-релиз)
- [Архитектура](#архитектура)
- [Поток данных](#поток-данных)
- [Точки входа](#точки-входа)
- [Канонический процесс buildx + QEMU](#канонический-процесс-buildx--qemu)
- [Переменные окружения](#переменные-окружения)
- [Файлы по порядку выполнения](#файлы-по-порядку-выполнения)
  - [`.gitlab-ci.yml`](#gitlab-ciyml)
  - [`scripts/image/test_local.sh`](#scriptsimagetest_localsh)
  - [`scripts/image/build_image.sh`](#scriptsimagebuild_imagesh)
  - [`scripts/image/01_resize.sh`](#scriptsimage01_resizesh)
  - [`scripts/image/02_mount.sh`](#scriptsimage02_mountsh)
  - [`scripts/image/03_install.sh`](#scriptsimage03_installsh)
  - [`scripts/image/04_configure.sh`](#scriptsimage04_configuresh)
  - [`scripts/image/05a_embed.sh`](#scriptsimage05a_embedsh)
  - [`scripts/image/05_repo.sh`](#scriptsimage05_reposh)
  - [`scripts/image/06_cleanup.sh`](#scriptsimage06_cleanupsh)
  - [`scripts/image/07_shrink.sh`](#scriptsimage07_shrinksh)
  - [`scripts/image/prebake.sh`](#scriptsimageprebakesh)
- [systemd-юниты](#systemd-юниты)
- [Сценарии модификации](#сценарии-модификации)
- [Локальный режим без push](#локальный-режим-без-push)
- [Диагностика](#диагностика)

---

## Архитектура

```
┌─────────────────────────────────────────────────────────────────────┐
│  GitLab CI Pipeline (1 push на тег vX.Y)                            │
│                                                                      │
│  prepare ──► docker-build (matrix×3) ──► img-build (matrix×6)       │
│                    │                            │                    │
│                    ▼                            ▼                    │
│       Docker .tar в .cache/docker-tars/  *.img.gz + urls/*.txt      │
│       + push в COMPANY_REGISTRY          + curl upload в Package Reg │
│                                                  │                   │
│                                                  ▼                   │
│                                          release:create              │
│                                          (6 asset links в Release)   │
└─────────────────────────────────────────────────────────────────────┘
```

### Что делает каждая стадия

| Stage | Что |
|---|---|
| **prepare** | Parallel matrix по 3 платформам. Скачивает и распаковывает базовую ОС **этой платформы** через [`fetch_base.sh`](#scriptsimagefetch_basesh) (xz/gz/7z/zip/gdrive), кэширует в `.cache/base/${PLATFORM}/`. |
| **docker-build** | Parallel matrix по 3 платформам. Собирает `Dockerfile.base` + `Dockerfile.${PLATFORM}` через buildx+QEMU, тегирует/пушит в `COMPANY_REGISTRY` (registry-теги с `${DOCKER_TAG_VERSION}`), сохраняет `.tar` для embed. |
| **img-build** | Parallel matrix 3×2 = 6 джобов. Каждый берёт `.tar` от своей платформы (для full) и собирает `.img`. На тегах сразу же заливает `.img.gz` в Package Registry. |
| **deploy** (release:create) | Собирает все `urls/*.txt` и создаёт GitLab Release с 6 asset-link'ами. |

### На устройстве

```
Boot 1 (sverk-firstboot.service):
  full (pre-baked) → nothing — образы уже в /var/lib/docker (~30 sec)
  full (legacy)    → docker load -i /opt/sverk-images/*.tar (~5-10 min)
  minimal          → docker login ${COMPANY_REGISTRY} + docker compose pull
  → systemctl enable+start sverk-ros2-docker.service
  → systemctl disable sverk-firstboot.service

Boot 2+ (sverk-ros2-docker.service):
  docker compose -f docker-compose.yml \
                 -f scripts/docker/docker-compose.${PLATFORM}.yml \
                 -f scripts/docker/docker-compose.web.yml \
                 up -d
```

### Pre-baked Docker storage (ускорение первого boot)

Стандартный подход (.tar embed + `docker load` на Pi) занимает ~5-10 минут при первой загрузке из-за распаковки слоёв на медленной SD-карте.

**Pre-baked** обходит это: на build-хосте мы запускаем временный `dockerd` с кастомным `--data-root`, делаем `docker load` в него, и упаковываем получившуюся директорию (`/var/lib/docker/`). Эта структура потом распаковывается **прямо в `/var/lib/docker/`** целевого `.img`.

```
┌─ build host ────────────────────────────┐    ┌─ Pi (firstboot) ─────────┐
│ docker buildx → platform.tar            │    │                          │
│              ↓                          │    │ docker daemon стартует   │
│ dockerd --data-root=/tmp/bake           │    │ → сразу видит образы     │
│   docker load -i platform.tar           │ ─► │ → НИКАКОГО docker load   │
│              ↓                          │    │ → ~30 сек до compose up  │
│ tar /tmp/bake/ → prebaked.tar           │    │                          │
└─────────────────────────────────────────┘    └──────────────────────────┘
```

**Что нужно знать:**
- Pre-bake требует Docker daemon на build-хосте (CI dind / Docker Desktop на Mac).
- Используется `overlay2` storage driver — тот же, что и у Pi по дефолту.
- xattrs (`trusted.overlay.*`) сохраняются через `tar --xattrs --xattrs-include='*'`. Без этого whiteout dirs ломаются.
- Отключается флагом `SKIP_PREBAKE=1` (CI) или `PREBAKE=0` (test_local.sh) — для быстрой итерации в разработке.
- Если pre-bake пропущен, фолбэк — старый embed `.tar` + `docker load` на Pi.

## Поток данных

| Файл | Откуда берётся | Куда попадает |
|---|---|---|
| `.cache/base/${PLATFORM}/*.img` | вендорский источник (`*_BASE_URL`) через `fetch_base.sh` | вход для `build_image.sh` |
| `.cache/docker-tars/${PLATFORM}/platform.tar` | `docker save` после buildx-сборки | embed в `/opt/sverk-images/` (для VARIANT=full) |
| Repo (`/repo`) | bind-mount build-хоста / GitLab checkout | embed в `/home/${RPI_USER}/sverk-ros2/` |
| `.build_state` | пишется `02_mount.sh`, читают остальные | состояние loop-устройств и mount-точек |
| `sverk-ros2-${PLATFORM}-${VARIANT}-${SHA}.img.gz` | вывод `build_image.sh` | GitLab artifact + Package Registry + Release |

## Точки входа

| Где | Файл | Запуск |
|---|---|---|
| Локально | `scripts/image/test_local.sh` | `bash scripts/image/test_local.sh [SENSOR] [PLATFORM] [VARIANT]` |
| GitLab CI | `.gitlab-ci.yml` (job `build:docker` → `build:img`) | автоматически на push/tag |

---

## Канонический процесс buildx + QEMU

Источник: [`docs/instal_local_docker.md`](instal_local_docker.md). Используется и в `test_local.sh`, и в стадии `docker-build` CI.

```bash
# 1. Регистрация QEMU для aarch64 (один раз на хост; в свежем dind — каждый раз)
docker run --privileged --rm tonistiigi/binfmt --install arm64

# Sanity:
docker run --rm --platform linux/arm64 alpine uname -m   # → aarch64

# 2. Создать buildx-билдер
docker buildx create --name sverk-arm64 --use
docker buildx inspect --bootstrap

# 3. Собрать с явным platform и --load (иначе образ останется в buildx-кэше,
#    docker save его не найдёт)
docker buildx build \
    --builder sverk-arm64 \
    --platform linux/arm64 \
    -f scripts/docker/Dockerfile.base \
    -t sverk/ros2-base:latest \
    --load \
    .

# 4. Верификация архитектуры
docker image inspect sverk/ros2-base:latest --format '{{.Architecture}}'
# → arm64

# 5. Сохранение в .tar (НЕ .tar.gz — для embed нужен чистый .tar)
docker save sverk/ros2-base:latest > .cache/docker-tars/base.tar
```

**Важно:**
- `--load` обязателен. Без него `docker save` не увидит образ.
- Embed только `.tar`, **не `.tar.gz`**. Gzip-стрим внутри сжатого `.img` повреждается при общем gzip всего образа и его распаковке на Pi (`unpigz: invalid deflate data`).

---

## Переменные окружения

### Обязательные

| Переменная | Описание |
|---|---|
| `IMAGE_NAME` | Имя рабочего `.img` файла |

### Платформа и вариант

| Переменная | Значения | Дефолт | Описание |
|---|---|---|---|
| `PLATFORM` | `rpi`, `radxa`, `orange` | `rpi` | Какой `Dockerfile.${PLATFORM}` собирать и какой `sverk-ros2-docker-${PLATFORM}.service` копировать |
| `VARIANT` | `full`, `minimal` | `full` | full = embed `.tar` в `/opt/sverk-images/`. minimal = ничего не embed, firstboot pull-ит из хаба |

`BOARD_FAMILY` (`rpi` / `rk3588`) и `RPI_USER` (`pi` / `radxa` / `orangepi`)
выставляются автоматически из `PLATFORM` в [`lib/board.sh`](#scriptsimagelibboardsh).
`RPI_USER` можно переопределить через CI/CD Variables.

### Базовая ОС платы

| Переменная | Дефолт | Описание |
|---|---|---|
| `RPI_BASE_URL` / `RPI_BASE_FILE` | Raspberry Pi OS lite arm64 | Источник и имя распакованного `.img` для rpi |
| `RADXA_BASE_URL` / `RADXA_BASE_FILE` | (пусто) / `radxa-cm5-debian.img` | Radxa OS — **задать URL в CI/CD Variables** |
| `ORANGE_BASE_URL` / `ORANGE_BASE_FILE` | (пусто) / `orangepi-5pro.img` | Orange Pi OS — **задать URL** (часто `gdrive:<FILE_ID>`) |

Формат `*_BASE_URL`: `http(s)://...img.{xz,gz,7z,zip,img}` или `gdrive:<FILE_ID>`.
Скачивание/распаковку делает [`fetch_base.sh`](#scriptsimagefetch_basesh).

### Теги образов на устройстве (.env)

| Переменная | Описание |
|---|---|
| `SVERK_OS_IMAGE` | Тег OS-образа; пишется в `${REPO_DIR}/.env` в `05_repo.sh` |
| `SVERK_DOTMDVIZ_IMAGE` / `SVERK_FRONTEND_IMAGE` / `SVERK_FLIGHTREVIEW_IMAGE` | Теги web-сервисов в `.env` |

Эти теги передаёт вызывающая сторона (CI / `test_local.sh`) и они **совпадают**
с тем, что push/save в `docker-build`. compose резолвит их через `${SVERK_*_IMAGE}`:
для **full** находит вшитые образы локально (без pull), для **minimal** делает
`docker compose pull` из `COMPANY_REGISTRY`.

### Локальный Docker-хаб компании

| Переменная | Дефолт | Описание |
|---|---|---|
| `COMPANY_REGISTRY` | `${CI_REGISTRY}/${CI_PROJECT_PATH}` | URL хаба (например `registry.sverk.io/sverk`) |
| `COMPANY_REGISTRY_USER` | `${CI_REGISTRY_USER}` | Логин для push/pull |
| `COMPANY_REGISTRY_PASSWORD` | `${CI_REGISTRY_PASSWORD}` | Пароль/токен (Protected, Masked в CI/CD Variables) |
| `SKIP_REGISTRY_PUSH` | `0` | `1` = только локальная сборка и embed, без push в хаб. Также отключает `VARIANT=minimal` (без хаба бессмыслен) |
| `SKIP_PREBAKE` | `0` | `1` = пропустить pre-bake шаг. Сборка docker-build быстрее, но первый boot на Pi займёт 5-10 мин вместо 30 сек. |
| `PREBAKE` (test_local.sh) | `1` | `0` = аналог `SKIP_PREBAKE=1` для локальных тестов |
| `DOCKER_TAG_VERSION` | `${CI_COMMIT_TAG:-${CI_COMMIT_SHORT_SHA}}` | Тег для собираемых Docker-образов |

### Остальные

| Переменная | Дефолт | Описание |
|---|---|---|
| `EXTRA_MB` | `14336` | Сколько MiB добавить к `.img`. Должно покрывать `.tar` (~10 GB) + repo + apt-кэш |
| `CAMERA_SENSOR` | `ov5647` | `ov5647` или `imx219` |
| `RPI_USER` | `pi` | Имя пользователя на Pi |
| `RPI_PASSWORD` | `raspberry` | Пароль |
| `WIFI_SSID/PASSWORD/COUNTRY` | (пусто/GB) | NetworkManager-профиль |
| `SHRINK_TARGET_MB` | `0` | `0` = skip shrink |
| `COMPRESS_IMG` | `0` | `1` = gzip итогового `.img` |
| `REPO_EMBED_SRC` | `/repo` | Откуда копировать репо |
| `DOCKER_IMAGE_CACHE` | (пусто) | Директория с `.tar` для embed |

---

## Файлы по порядку выполнения

### `.gitlab-ci.yml`

Точка входа CI. 4 стадии: `prepare` → `docker-build` → `img-build` → `deploy`.

**Что менять для типичных задач:**

| Что | Где |
|---|---|
| Базовая ОС платы | `RPI_BASE_URL` / `RADXA_BASE_URL` / `ORANGE_BASE_URL` (+ `*_BASE_FILE`) в `variables:` или CI/CD > Variables |
| Размер .img | `EXTRA_MB` |
| Реестр компании | `COMPANY_REGISTRY` + `COMPANY_REGISTRY_USER/PASSWORD` в CI/CD UI |
| Отключить push в хаб | `SKIP_REGISTRY_PUSH: "1"` |
| Добавить платформу | См. [Сценарии модификации](#добавить-новую-платформу) |
| Триггер релиза | `rules:` в `release:create` (regex по `CI_COMMIT_TAG`) |

**Правила пропуска (skip) — где смотреть:**

- `radxa`/`orange` без `*_BASE_URL` → пропуск на `prepare`, `build:docker`,
  `build:img` (одинаковое условие `$PLATFORM == ... && $*_BASE_URL == ""`).
- `*-minimal` без реестра → пропуск в `rules:` у `build:img`
  (`SKIP_REGISTRY_PUSH=1`, пустые `COMPANY_REGISTRY_USER/PASSWORD`, или
  `COMPANY_REGISTRY` без хоста).
- `build:docker`/`build:img` вычисляют `IMAGE_NS`: если реестра нет —
  локальный namespace `sverk` (full всё равно соберётся, push пропустится).
- `release:create` → `needs: optional: true` (релиз из того, что собралось).

**Требования к runner:**
- `privileged = true` (для loop, kpartx, binfmt_misc, dind)
- ~30 GB на промежуточные файлы

**Решение проблемы 413/499 при upload .img.gz:** Заливаем напрямую в Package Registry внутри `build:img` (см. инлайн-блок `if echo ${CI_COMMIT_TAG} ... curl --upload-file`). В artifact остаётся только маленький `urls/*.txt`.

---

### `scripts/image/test_local.sh`

Локальное воспроизведение CI через `docker run --privileged`.

**Запуск:**
```bash
# Дефолт (ov5647 / rpi / full)
bash scripts/image/test_local.sh

# С платформой и вариантом
bash scripts/image/test_local.sh ov5647 rpi full
bash scripts/image/test_local.sh ov5647 radxa full
bash scripts/image/test_local.sh imx219 orange full

# Minimal вариант (нужен COMPANY_REGISTRY на Pi для pull)
COMPANY_REGISTRY=registry.sverk.io bash scripts/image/test_local.sh ov5647 rpi minimal
```

**Что делает:**

1. Качает базовую ОС платформы в `.cache/base/${PLATFORM}/` (через `fetch_base.sh`).
2. Настраивает QEMU+buildx (idempotent).
3. Собирает `Dockerfile.base` + `Dockerfile.${PLATFORM}` через `docker buildx`. Кэширует уже собранные через `docker image inspect`.
4. Сохраняет `.tar` в `.cache/docker-tars/${PLATFORM}/platform.tar`.
5. Запускает `docker run debian:bookworm-slim ... build_image.sh` с правильными env.

**Точки модификации:**

| Что | Где |
|---|---|
| Базовая ОС платы | `RPI_BASE_URL` / `RADXA_BASE_URL` / `ORANGE_BASE_URL` (env) |
| Включить gzip | `COMPRESS_IMG=1 bash scripts/image/test_local.sh ...` |
| WiFi | Аргументы `$4` (SSID) и `$5` (PASSWORD) или env `WIFI_*` |

---

### `scripts/image/build_image.sh`

Оркестратор. Экспортирует env, ставит EXIT trap на cleanup, последовательно запускает 01→05_repo.

**Что добавлено для multi-platform:**
- Валидация `PLATFORM` и `VARIANT` (выходит с ошибкой при недопустимых значениях).
- Проверка что `VARIANT=minimal` идёт вместе с `COMPANY_REGISTRY`.
- Финальный rename: `sverk-ros2-${PLATFORM}-${VARIANT}-${BASE}.img(.gz)` — для матрицы CI.

**Точки модификации:**

| Что | Как |
|---|---|
| Добавить новый этап | `run_stage "08 — Foo" 08_foo.sh` в нужное место |
| Поменять порядок | Переставить вызовы `run_stage` |
| Отключить zeroing | Убрать блок `dd if=/dev/zero` (потеря ~20-30% компрессии) |

---

### `scripts/image/lib/board.sh`

Абстракция семейства платы (sourced-хелпер). `board_init <platform>` выставляет
`BOARD_FAMILY` (`rpi`/`rk3588`), `DEFAULT_USER`, `BOOT_CONFIG_KIND`, `BOOT_MOUNT_REL`.
`detect_partitions <loop_base>` определяет `ROOT_PART` (наибольший ext), `BOOT_PART`
(vfat — есть только у RPi) и `ROOT_PART_NUM`. Подключается из `build_image.sh` и `01–04`.

---

### `scripts/image/fetch_base.sh`

Скачивает и распаковывает базовую ОС платформы в кэш. Источники: `http(s)://...img.{xz,gz,7z,zip,img}`
или `gdrive:<FILE_ID>` (gdown). Идемпотентно (cache hit по `${DEST_DIR}/${BASE_FILE}`).
Используется стадией `prepare` и `test_local.sh`.

---

### `scripts/image/01_resize.sh`

Расширяет `.img` и **последний** раздел (rootfs). `truncate +EXTRA_MB` → (для GPT)
`sgdisk -e` переносит резервный GPT-заголовок в конец → `parted resizepart <last> 100%`
→ `kpartx + e2fsck + resize2fs`. Номер раздела определяется через `partx` (а не хардкод `2`),
тип таблицы — через `blkid PTTYPE`. Работает и для MBR (RPi), и для GPT (RK3588).

**Pitfall:** `parted` обязательно ДО `resize2fs`. В контейнере — `kpartx`, не `losetup --partscan`.
Загрузчик RK3588 (idbloader/u-boot) лежит в сырых секторах ДО p1 — мы его не трогаем.

---

### `scripts/image/02_mount.sh`

`board_init` + `losetup` → `kpartx` → `detect_partitions` → `mount` rootfs (+ boot **только**
для RPi: vfat `/boot/firmware`) → `chroot` setup (qemu-static, bind /proc /sys /dev, resolv.conf,
policy-rc.d). Пишет `MOUNT_ROOT`/`MOUNT_BOOT` (для RK3588 `MOUNT_BOOT` пуст) в `BUILD_STATE`.

**Pitfall:** `resolv.conf` должен быть файлом, не симлинком. `policy-rc.d` блокирует автозапуск сервисов при `apt install` внутри chroot.

---

### `scripts/image/03_install.sh`

`apt install` внутри chroot (ветвится по `BOARD_FAMILY`): `gpiod`, `i2c-tools`, `network-manager`,
Docker CE, `gdown`. `rpicam-apps` — только для `rpi`. URI Docker-репо берётся из `/etc/os-release`
(`debian`/`ubuntu`). Если Docker уже есть в вендорском образе — установка пропускается.

**Pitfall:** Docker daemon не запускается в chroot. `systemctl enable docker` создаёт только симлинки.

---

### `scripts/image/04_configure.sh`

**Board-aware.** Копирует `scripts/sverk-ros2-docker-${PLATFORM}.service` и патчит на лету:
- `WorkingDirectory` и `HOME=/home/{pi|radxa|orangepi}` → фактический `${RPI_USER}` (= `DEFAULT_USER` платы).
- `TimeoutStartSec=300` → `infinity`.

Специфика RPi (`config.txt`, headless `/boot/firmware/ssh`, `dphys-swapfile`) применяется
**только при `BOARD_FAMILY=rpi`**. Для `rk3588`: SSH через `systemctl enable ssh`, swap — zram
вендорской ОС, камера-оверлей — через extlinux/`*Env.txt` (TODO, отдельный подшаг).

**Секции файла:**

1. config.txt — awk-патч, добавляет camera overlay + spi1+uart+i2c. Удаляет `start_x=1`, `gpu_mem`, `temp_soft_limit` (ломают CM5).
2. `/etc/modules`: i2c-dev, spi-dev.
3. SSH port 22→2222, `PasswordAuthentication yes`, headless `/boot/firmware/ssh`.
4. User: `useradd -m -u 1000 ${RPI_USER}` + chpasswd + sudo NOPASSWD.
5. Groups: sudo adm docker dialout tty i2c video gpio spi.
6. **systemd-юниты:** per-platform docker service + firstboot service.
7. WiFi: `${SSID}.nmconnection` если SSID задан.
8. NM-conf: `managed=true`, `dhcp=internal`.
9. `/etc/wpa_supplicant/wpa_supplicant.conf` country=$(WIFI_COUNTRY).
10. Swap 2 GB (`CONF_SWAPSIZE=2048` в `/etc/dphys-swapfile`).

**Pitfall:** `start_x=1` и `gpu_mem` **ломают** boot на CM5.

---

### `scripts/image/05a_embed.sh`

**Три пути embed-инга:**

1. **`VARIANT=full` + `prebaked.tar` есть (приоритет, быстрый boot):**
   Распаковывает `${DOCKER_IMAGE_CACHE}/prebaked.tar` прямо в `${MOUNT_ROOT}/var/lib/docker/` с сохранением xattrs (`--xattrs --xattrs-include='*'`). Создаёт маркер `/opt/sverk-images/.prebaked`. На Pi `firstboot` ничего не делает — образы уже видны Docker'у.

2. **`VARIANT=full` + только `*.tar` (legacy, медленный boot):**
   Копирует `*.tar` из `${DOCKER_IMAGE_CACHE}` в `/opt/sverk-images/`. `tar -tf` integrity check + `cp` + `sync` + проверка размера. На Pi `firstboot` делает `docker load -i` (5-10 минут).

3. **`VARIANT=minimal`:** создаёт маркер `/opt/sverk-images/.minimal` и выходит. firstboot тянет из реестра.

`sync` обязателен перед общим zeroing в `build_image.sh` — иначе ext4-журнал может попортить эмбед.

**Pitfalls:**
- Embed только чистый `.tar`, не `.tar.gz` (gzip-стрим внутри сжатого `.img` повреждается на Pi).
- При распаковке `prebaked.tar` обязательны `--xattrs` и `--numeric-owner` — без них overlay2 layers сломаются.

---

### `scripts/image/05_repo.sh`

Генерирует `/usr/local/bin/sverk-firstboot.sh` через heredoc + копирует `.env-example` → `.env`.

**Multi-variant firstboot:**

- **full** — для каждого `.tar` в `/opt/sverk-images/`: `tar -tf` → `docker load -i` → rm. Если ничего нет — деградирует до `minimal`.
- **minimal** — ждёт сеть → `docker login ${COMPANY_REGISTRY}` → `docker compose -f ... pull`.

Оба варианта в конце: `systemctl enable+start sverk-ros2-docker.service` → `systemctl disable sverk-firstboot.service`.

`COMPOSE_FILES` строится в Bash-хосте и подставляется в heredoc: `-f docker-compose.yml -f scripts/docker/docker-compose.${PLATFORM}.yml -f scripts/docker/docker-compose.web.yml`.

**Pitfall:** Heredoc `<< FIRSTBOOT` (без кавычек) делает substitution. Внутренние `\$` экранируются, чтобы переменные резолвились на Pi.

---

### `scripts/image/06_cleanup.sh`

Идемпотентно отмонтирует всё и освобождает loop. Без `set -e` (каждый шаг независимо). Вызывается из EXIT trap, в конце `build_image.sh`, и в CI `after_script`.

**Точки модификации:** при добавлении нового bind-mount в `02_mount.sh` — добавить соответствующий umount.

---

### `scripts/image/07_shrink.sh`

Опциональный shrink: `resize2fs -M` → `parted resizepart` → `truncate`. Запускается только при `SHRINK_TARGET_MB != 0`.

---

### `scripts/image/prebake.sh`

Helper для создания **pre-baked Docker storage**. Не вызывается из `build_image.sh` — это **отдельный этап**, выполняемый ПЕРЕД image-сборкой (в `docker-build` стадии CI или в `test_local.sh`).

**Что делает:**
1. Запускает временный `dockerd --data-root=$TMP --storage-driver=overlay2 --host=unix://...`.
2. Ждёт socket (до 60 сек).
3. `docker load -i INPUT.tar`.
4. Гасит dockerd (SIGTERM, ждёт до 15 сек, потом SIGKILL).
5. `sync` — flush ext4-журнал.
6. `tar --xattrs --xattrs-include='*' --numeric-owner -cf OUTPUT.tar -C $TMP .`

**Аргументы:**
```bash
prebake.sh INPUT.tar OUTPUT.tar
# INPUT.tar  — результат `docker save sverk/ros2:latest`
# OUTPUT.tar — упакованный /var/lib/docker для embed в .img
```

**Требования:**
- Запуск под `root` (нужен `dockerd`).
- Доступ к ядру с overlay-fs + cgroups (privileged-контейнер или нативный Linux).
- На macOS — только через `docker run --privileged docker:24-dind` (см. как используется в `test_local.sh`).

**Pitfalls:**
- xattrs (`trusted.overlay.*`) — без `--xattrs-include='*'` whiteout dirs ломаются на Pi.
- UID 0 (root) хардкодится через `--numeric-owner` — иначе на сборочном хосте с другим `/etc/passwd` могут поехать права.
- НЕ запускать на хосте где уже работает основной dockerd — конфликт сокетов (мы используем `--host=unix:///tmp/...`).

## systemd-юниты

| Файл | Где | Что делает |
|---|---|---|
| `scripts/image/systemd/sverk-firstboot.service` | На образе из image-pipeline | Запускает `/usr/local/bin/sverk-firstboot.sh` (генерируется в 05_repo.sh) |
| `scripts/sverk-ros2-docker-{rpi,radxa,orange}.service` | На образе после патчинга | `docker compose -f ... up -d` с правильными override-файлами |

`04_configure.sh` копирует `scripts/sverk-ros2-docker-${PLATFORM}.service` → `/etc/systemd/system/sverk-ros2-docker.service`, заменяя hardcoded путь `/home/{pi|radxa|orangepi}/sverk-ros2` на `/home/${RPI_USER}/sverk-ros2` и timeout 300 → infinity.

---

## Сценарии модификации

### Поменять Docker-образ (одну из платформ)

1. Отредактировать `scripts/docker/Dockerfile.${PLATFORM}` или `scripts/docker/Dockerfile.base`.
2. Локально: `bash scripts/image/test_local.sh ov5647 ${PLATFORM} full` — buildx пересоберёт и кэширует.
3. В CI пересборка автоматическая (тег коммита меняется → cache miss).

### Добавить новую платформу

Допустим, `rdk`:

1. Создать `scripts/docker/Dockerfile.rdk` и `scripts/docker/docker-compose.rdk.yml`.
2. Создать `scripts/sverk-ros2-docker-rdk.service`.
3. В `.gitlab-ci.yml`: добавить `rdk` в matrix обоих джобов:
   ```yaml
   build:docker:
     parallel:
       matrix:
         - PLATFORM: [rpi, radxa, orange, rdk]   # <-- сюда
   build:img:
     parallel:
       matrix:
         - PLATFORM: [rpi, radxa, orange, rdk]
           VARIANT:  [full, minimal]
   ```
4. В `scripts/image/build_image.sh` и `04_configure.sh` добавить `rdk` в `case PLATFORM`.
5. В `test_local.sh` добавить `rdk` в валидацию аргументов.

### Поменять реестр

В GitLab CI/CD UI → Variables:
- `COMPANY_REGISTRY` = `your-registry.example.com/your-namespace`
- `COMPANY_REGISTRY_USER`, `COMPANY_REGISTRY_PASSWORD` (Protected, Masked)

Локально — через env: `COMPANY_REGISTRY=... bash scripts/image/test_local.sh ...`.

### Добавить новый пакет на Pi

В `scripts/image/03_install.sh` — дописать в нужный `apt-get install` блок.

### Изменить триггер релиза

В `.gitlab-ci.yml` regex `'^v\d+\.\d+'` в `rules:` у `release:create`. Он не
заякорен в конце, поэтому **уже ловит и альфа-теги** (`v1.8a`, `v1.8-alpha`,
`v1.8-rc1`). Стабильный vs альфа различается внутри `release:create` отдельной
проверкой `^v[0-9]+\.[0-9]+(\.[0-9]+)?$` (см. [Альфа / тестовые сборки](#альфа--тестовые-сборки)).
Тот же regex стоит в блоке upload в `build:img` — чтобы менять триггер, правьте
оба места.

---

## Локальный режим без push

Если локального хаба ещё нет или нужно быстро проверить сборку:

```yaml
# В .gitlab-ci.yml CI/CD > Variables
SKIP_REGISTRY_PUSH: "1"
```

Что произойдёт:
- `build:docker` всё ещё собирает `.tar` и кладёт в artifact (теги в локальном
  namespace `sverk`, т.к. `IMAGE_NS` фолбэчит при отсутствии реестра).
- Шаг `docker push` пропускается.
- `build:img` matrix-комбинации `*minimal` автоматически скипаются (без хаба
  бессмысленны) — остаются только `*full` job'ы.
- `release:create` создаёт Release только из собранных `*full` (и только тех
  плат, у которых задан `*_BASE_URL`).

> То же самое произойдёт без явного `SKIP_REGISTRY_PUSH=1`, если просто не заданы
> `COMPANY_REGISTRY_USER` / `COMPANY_REGISTRY_PASSWORD` (или нет адреса реестра):
> `*-minimal` пропустятся, `*-full` соберутся.

Тот же режим локально:
```bash
SKIP_REGISTRY_PUSH=1 bash scripts/image/test_local.sh ov5647 rpi full
```

---

## Диагностика

| Симптом | Где смотреть |
|---|---|
| Pi загорается красным после boot | Мало места в `EXTRA_MB` или `docker load` упал. `/var/log/sverk-firstboot.log` через UART/SD-ридер |
| `unpigz: invalid deflate data` в firstboot | Embed-нут `.tar.gz` вместо `.tar`. Проверь [05a_embed.sh](#scriptsimage05a_embedsh) (`for f in ...*.tar`) |
| `cr: command not found` в 04_configure.sh | Скрипт запущен напрямую без `source BUILD_STATE`. Запускать только через `build_image.sh` |
| `losetup: failed to set up loop device` | Не privileged-runner |
| `binfmt_misc: no qemu-aarch64` | binfmt не зарегистрирован. См. `before_script` в build:docker |
| GitLab artifact слишком большой (413) | Уже исправлено: `.img.gz` заливается напрямую в Package Registry, в artifact только `urls/*.txt` |
| WiFi не подключается | `/etc/NetworkManager/system-connections/${SSID}.nmconnection` + `nmcli con show` |
| `docker compose up` ошибка `image not found` | На устройстве: `docker images` — проверить что нужный тег есть. Compose ждёт `${COMPANY_REGISTRY}/sverk-ros2:${TAG}-${PLATFORM}` |
| `docker buildx build` падает на ARM64 этапе | binfmt не зарегистрирован в dind. Перезапусти `tonistiigi/binfmt --install arm64` |
| `[firstboot] ERROR: pre-baked marker есть, но docker images пустой` | xattrs не сохранились при распаковке prebaked.tar. Проверь что в `05a_embed.sh` `tar -xf` идёт с `--xattrs --xattrs-include='*'`. Если CI-runner на ФС без xattr-support — можно `SKIP_PREBAKE=1`. |
| `prebake.sh` зависает на «waiting for dockerd» | Контейнер не privileged или ядро без overlay-fs. Запускать ТОЛЬКО внутри `docker run --privileged docker:24-dind`. |
| Pre-bake сильно увеличил размер артефакта | Норма: `prebaked.tar` (~10G) больше `platform.tar` (~10G) только на размер метаданных Docker (~50 MB). Если разница в разы — что-то пошло не так. |

### Полезные команды на Pi (SSH порт 2222)

```bash
# Логи firstboot
sudo cat /var/log/sverk-firstboot.log

# Статус сервисов
systemctl status sverk-firstboot.service
systemctl status sverk-ros2-docker.service
journalctl -u sverk-ros2-docker.service -f

# Docker
docker images
docker ps -a
docker compose -f ~/sverk-ros2/docker-compose.yml \
               -f ~/sverk-ros2/scripts/docker/docker-compose.rpi.yml \
               -f ~/sverk-ros2/scripts/docker/docker-compose.web.yml logs

# WiFi
nmcli radio wifi
nmcli con show
```

### Полезные команды на build-хосте

```bash
# Очистить весь кэш (RPi OS + docker .tar)
rm -rf .cache/

# Только docker-кэш (например, после смены Dockerfile)
rm -rf .cache/docker-tars/

# Пересобрать конкретную платформу
docker image rm sverk/ros2:local-test-rpi sverk/ros2-base:local-test 2>/dev/null
bash scripts/image/test_local.sh ov5647 rpi full

# Подключиться к работающему build-контейнеру
docker exec -it sverk-image-builder bash

# Принудительный cleanup после падения
sudo bash scripts/image/06_cleanup.sh
```

### Связанные документы

- [`docs/instal_local_docker.md`](instal_local_docker.md) — каноническая процедура buildx+QEMU для ручной сборки Docker-образов.
- [`docs/doc_docker.md`](doc_docker.md) — структура мульти-платформенных Docker-образов и compose-overrides.
