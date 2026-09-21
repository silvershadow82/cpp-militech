# follow_app — Білд і симуляція

Візуальний трекінг цілі через MAVLink: камера (справжня або симульована) відслідковує ціль, а дрон
наводиться на неї і тримає дистанцію. Див. [PROJECT.md](PROJECT.md) з описом задачі та
[COMPONENTS.md](COMPONENTS.md) зі схемою підключень на реальному дроні.

Система може працювати в двох режимах:

- `--sim` — симульована камера з **ArduPilot SITL**. Без фактичної камери, без OpenCV, без
  Raspberry Pi. Це режим для перевірки і той, що описаний нижче.
- `--hw` — камера Raspberry Pi і справжній трекер. Потребує збірку з `FOLLOW_WITH_OPENCV=ON`
  і реальні компоненти; див. [docs/VISION.md](docs/VISION.md).

Усі команди нижче запускаються з каталогу **`coursework/`**, де лежить `Makefile`.

## Швидкий старт

```bash
make            # список цілей
make check      # усе, що має бути зеленим перед комітом
```

`make check` — це `build` + `test` + `build-vision` + `test-vision` + `format-check` + `purity`.

| Ціль | Що робить |
| --- | --- |
| `make build` | збірка за замовчуванням, `FOLLOW_WITH_OPENCV=OFF` |
| `make build-vision` | збірка з OpenCV, `FOLLOW_WITH_OPENCV=ON` |
| `make test` / `make test-vision` | юніт- та інтеграційні тести відповідної збірки |
| `make sitl SCENARIO=circle` | один сценарій проти ArduCopter SITL |
| `make sitl-all` | усі сценарії з підсумком |
| `make qgc SCENARIO=circle` | запуск, який видно в QGroundControl |
| `make venv` | створює venv з `pymavlink` для симулятора пілота |
| `make format` / `make format-check` | clang-format (окрім `control/Core.h`) |
| `make purity` | падає, якщо OpenCV потрапив у `follow_core` |
| `make clean` | прибрати каталоги збірки |

Змінні: `SCENARIO`, `ARDUPILOT_DIR`, `PYTHON`, `JOBS`, `BUILD_DIR`, `VISION_DIR`, `TIMEOUT_S`.

## Що покриває симуляція

`--sim` запускає увесь цикл польоту з ArduCopter: MAVLink по UDP, пілот, що армує та переключає
режим польоту на GUIDED, апроксиматор дистанції, стейт-машина супервайзера, контролер і
встановлення маркерів швидкості. Рух цілі замість камери описується одним зі сценаріїв у файлах
конфігурації.

Симуляція **не** використовує реальну камеру — трекер OpenCV, пайплайн GStreamer і накладання на
фреймбуфер працюють тільки в режимі `--hw`. Зелена симуляція нічого про них не говорить.

## Структура проекту

Цей модуль створений на базі `homework_11`: каталоги за роллю, імена файлів у PascalCase збігаються
з класом, інтерфейси з префіксом `I`, підключення без префікса проекту (`#include "control/Core.h"`).

```text
include/
  Types.h  ControlLoop.h  MissionProcessor.h  HwMissionProcessor.h  StatCollector.h
  interfaces/ абстрактні інтерфейси: IFrameSource, ITracker, IByteLink, ICameraModel,
              IConfigLoader
  models/     дані і розрахунки: Angles, Frames, AttitudeHistory, Config, Intrinsics,
              PinholeModel, FisheyeKbModel
  control/    модуль керування і прийняття рішень: Core, FollowController, TargetEstimator,
              Supervisor
  comms/      протоколи комунікацій: LinkSpec, SocketLink, SerialLink, MavLink, MavlinkIo
  providers/  провайдери: SyntheticFrameSource, VideoFileSource, PiCameraSource,
              CameraTrackerSource, SimVision
  vision/     OpenCV: TrackerFactory, Overlay, Calibration
  config/     конфігурації і лоадери: FileConfigLoader, ScenarioLoader, ComponentFactory
  util/  sim/
```

Простори імен повторюють каталоги під коренем `follow::` (`follow::models`, `follow::control`,
`follow::comms`, ...). Корінь `follow::` прибирати не можна: кореневий `CMakeLists.txt` збирає
`homework_11` і `coursework` в одному проєкті, а `homework_11` уже має глобальний `namespace comms`
зі своїми `MavLink`/`SerialLink`/`SocketLink` — без `follow::` вони б конфліктували.

### follow_core відокремлена бібліотека

**`models/` і `control/` — а тому і `follow_core` — працюють без потоків, I/O, без JSON, без
MAVLink, без OpenCV і таймерів.** Усе в бібліотеці працює як чисті функції, які легко тестувати без
дрона.

Усе, чому потрібні потоки, сокети, файли або таймери, знаходиться поза ними: `ControlLoop.h` і
`MissionProcessor` тримають потоки, `comms/` — сокети, `config/` — парсинг файлів, `providers/`
працюють з камерою.

Два практичні наслідки:

- `FOLLOW_WITH_OPENCV=OFF` — режим за замовчуванням і має завжди білдитись. Якщо підключення OpenCV
  дістанеться до `models/` або `control/`, одночасно зламаються збірка за замовчуванням, збірка в
  devcontainer (gcc-13) і крос-збірка під aarch64. Перевіряється через `make purity`.
- `include/control/Core.h` відформатований вручну — це єдиний файл, який ніколи не передається в
  clang-format. Усе решта має бути чистим за `--style=file:.devcontainer/.clang-format`; `make format`
  виключає його автоматично.

## Попередні вимоги

**C++20 toolchain і CMake ≥ 3.20.** Збірка за замовчуванням не потребує нічого іншого — без OpenCV.

**ArduPilot SITL** (потрібен лише для цілей `sitl`, `sitl-all` і `qgc`):

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./waf configure --board sitl
./waf copter
```

Створює `build/sitl/bin/arducopter`. Треба вказати `ARDUPILOT_DIR`.

**Python з `pymavlink`**, для симуляції пілота:

```bash
make venv
```

Створює `../build/sitl-venv` і ставить туди `pymavlink`, не чіпаючи системний Python.

**QGroundControl** — для візуального спостереження, необов'язково, але приємно.

## Збірка і тестування

```bash
make build   && make test          # FOLLOW_WITH_OPENCV=OFF
make build-vision && make test-vision   # FOLLOW_WITH_OPENCV=ON, потрібен OpenCV
```

`make build` створює `follow_app` і `follow_check_run`; `make test` запускає юніт- та інтеграційні
тести. Збірка з OpenCV необов'язкова для симуляції.

## Запуск одного сценарію

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
make sitl SCENARIO=stationary
```

Скрипт за цією ціллю запускає ArduCopter SITL, `follow_app --sim` і `sitl_operator.py` в якості
пілота: він армує, злітає в режимі LOITER, зависає, потім переводить у GUIDED, що власне і запускає
стеження. Коли сценарій закінчується, запускається `follow_check_run`, який перевіряє лог відносно
очікувань сценарію.

На виході лише один рядок:

```text
PASS stationary (1361 steps, tolerance x1.5)
```

Код виходу 0 — ТЕСТ ПРОЙДЕНО, 1 — НЕ ПРОЙДЕНО, 2 — ПОМИЛКА ІНІЦІАЛІЗАЦІЇ. Логи потрапляють у
`../build/sitl-runs/<scenario>-<timestamp>/`: `run.csv`, `app.log`, `sitl.log`, `operator.log` і
`check.txt`.

Допуски розширені в 1.5 раза для SITL, бо відповідь справжнього автопілота — не ідеальна модель, під
яку писалися очікування сценарію.

## Запуск усіх сценаріїв

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
make sitl-all
```

Кожен сценарій запускає власний інстанс SITL і відтворюється 1–2 хв, тож усі разом — приблизно
п'ятнадцять хвилин. У кінці друкується підсумок `== N passed, M failed`.

| Сценарій | Що перевіряє |
| --- | --- |
| `stationary` | Ціль стоїть за 3 м. |
| `stationary_known_size` | Відома висота 1.7 м дає метричну дистанцію, дрон підходить на `d_set`. |
| `walk_line` | Ціль відходить зі швидкістю 1.0 м/с; відставання має стабілізуватись, а не рости. |
| `stop_and_go` | Ціль йде, зупиняється, знову йде, зупиняється. |
| `circle` | Ціль йде по колу радіусом 6 м зі швидкістю 1 м/с навколо точки за 9 м. |
| `yaw_only` | Етап 2 пусконаладки: `enable_vx = false`, керується тільки курс. |
| `fast_dash` | Ціль швидко рухається вбік зі швидкістю 8 м/с та залишається в полі зору 128°. |
| `pass_by` | Ціль ведеться, потім проходить повз дрон на 5 м/с і виходить із поля зору. |
| `occlusion_short` | Ціль зникає на 1 с і знову з'являється. |
| `occlusion_long` | Ціль зникає на 5 с, довше, ніж `lost_timeout`. |
| `lock_miss` | Ціль поза зоною захоплення (квадрат у центрі екрану): захоплення не відбувається. |

Сценарії `occlusion_*` і `lock_miss` задіюють гілку Lost/Reacquire, тож вони найчутливіші до змін у
апроксиматорі або в адаптері трекера.

## Відстеження через QGroundControl

```bash
export ARDUPILOT_DIR=/path/to/ardupilot
make qgc SCENARIO=circle
```

Звичайний сценарій не дає звʼязку для наземної станції: `follow_app` використовує SERIAL4, а
симулятор пілота — TCP-порт SERIAL0. Ціль `qgc` запускає SITL з додатковим MAVLink-портом,
націленим на UDP 14550, який QGroundControl слухає за замовчуванням. Дрон зʼявиться сам — видно,
як він армується, набирає висоту, переходить у GUIDED, а далі доводиться курсом і зміщується за
ціллю.

Дві речі, які скрипт робить за вас:

- виставляє `SERIAL5_PROTOCOL 2` — без цього SITL відкриє порт, але не говоритиме по ньому MAVLink,
  і QGroundControl нічого не побачить;
- перевіряє, чи не зайнятий `tcp:5760` попереднім запуском, і зупиняється з поясненням замість
  `bind failed on port 5760`.

> **SITL друкує `Waiting for connection ....` і виглядає так, ніби завис.** Це нормально: він не
> починає працювати, доки хтось не підключиться до TCP 5760 — це робить симулятор пілота наступним
> кроком. Не запускайте SITL вдруге.

## Перевірка лога вручну

`run.csv` можна перевірити повторно, без нового польоту:

```bash
../build/coursework/follow_check_run \
  --log ../build/sitl-runs/<run>/run.csv \
  --scenario config/scenarios/stationary.json \
  --config config/follow.json \
  --tolerance-scale 1.5
```

## Запуск follow_app напряму

Цілі `make` — це зручність. Сам застосунок:

```text
follow_app --sim --scenario FILE [--config FILE] [--link SPEC] [--log FILE]
  --scenario  JSON сценарію (config/scenarios/*.json)
  --config    follow.json (за замовчуванням config/follow.json)
  --link      udp:PORT, udp:PORT:HOST:PORT або uart:DEVICE:BAUD (за замовчуванням udp:14560 для --sim)
  --log       CSV лог запуску (за замовчуванням follow_run.csv)
```

Якщо на лінку немає автопілота, застосунок пише `FC: mavlink link wait failed` і залишається в
стані `NoFc`, не виставляючи жодного setpoint — колонки `vx` і `yaw_rate` у лозі лишаються
порожніми. Це безпечна поведінка за замовчуванням, а не помилка: він чекає на польотний контролер,
замість керувати дроном, якого не бачить.

## Усунення несправностей

**`bind failed on port 5760 - Address already in use`** — живий SITL з попереднього запуску. SITL
друкує `Waiting for connection ....` і чекає, доки підключиться симулятор пілота, тож справний
інстанс виглядає завислим і його легко запустити вдруге. Знайти і зупинити старий:

```bash
lsof -nP -iTCP:5760 -sTCP:LISTEN     # показує PID, що тримає порт
pkill -f 'build/sitl/bin/arducopter' # або kill <PID> для конкретного
```

Врахуйте: другий інстанс перезаписує `sitl.log` першого перед тим, як впасти, тож у лозі буде
помилка порту, хоча робочим лишається саме перший.

**`<python> cannot import pymavlink`** — `PYTHON` не вказує на virtualenv. Запустіть `make venv`.

**`no SITL binary at ...`** — не задано `ARDUPILOT_DIR` або SITL не зібрано. Див. «Попередні вимоги».

**`sitl_operator.py exited early`** — дивіться `operator.log` у каталозі запуску. Звичайна причина —
армування відхилено, бо SITL ще не має GPS-фіксу; оператор чекає до 120 с (`--arm-timeout`).

**Дрон армується і зависає, але не летить за ціллю** — стеження вмикається лише в GUIDED. Перевірте
`operator.log` на перемикання режиму і колонку `state` у `run.csv`: `NoFc` означає, що MAVLink від
автопілота взагалі не приходить, `Idle` — що звʼязок є, але GUIDED не вмикався.

**Залишається в `Idle`, хоча дрон уже в GUIDED** — стеження запускає *перехід* у GUIDED, а не сам
факт перебування в ньому (`Supervisor.cpp`: `becameGuided`). Якщо перемкнути в GUIDED у
QGroundControl до запуску `follow_app`, застосунок не побачить фронту. Перемкніть у LOITER і назад у
GUIDED уже із запущеним застосунком. Це навмисно: перезапуск застосунку поруч із заармованим дроном
не може змусити його самовільно полетіти за ціллю.
