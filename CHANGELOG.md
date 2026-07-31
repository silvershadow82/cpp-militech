# Changelog

Усі помітні зміни в цьому репо фіксуються тут.
Формат - [Keep a Changelog](https://keepachangelog.com/uk/1.1.0/), дати в ISO 8601.

## 2026-07-03

### Added

- Block 5 / Lesson 5.4: додано ROS 2 demo `demos/lesson_5_4` як набір
  окремих малих workspace-прикладів: власний `.msg` і типізований топік,
  власний `.srv` і service client/server, service server з типізованим
  топіком статусу, YAML-параметри, launch-файл і чиста C++ логіка без
  `rclcpp`.
- Block 5 / Lesson 5.4 / Homework 13: додано стартовий package
  `antidrone_turret` з `Target.msg`, `ActuatorStatus.msg`,
  `TriggerActuator.srv`, `target_track_publisher_node`, CSV-треками FPV,
  `actuator_node`, YAML/launch і постановкою ДЗ. Повідомлення для команд
  гімбала, серво та статусу турелі студенти додають самостійно. Готові ноди
  показують розділення ROS 2 обгортки і звичайної C++ логіки через
  `TargetTrackSimulation` та `ActuatorModel`, а стартові тести перевіряють
  читання треку, послідовність цілей та стан актуатора. README описує
  сценарії треків: дальній проліт, низька оцінка надійності розпізнавання,
  постріл при `READY`, блокування повторної команди пострілу під час
  `RELOADING` і наступний епізод. README пояснює, що `confidence` означає
  оцінку надійності розпізнавання цілі, а гімбал, yaw-серво і актуатор
  відповідають за різні частини наведення та команду пострілу. Актуатор
  описано як корисне навантаження,
  закріплене на лінії наведення турелі.
  Контракт команд наведення описує центр кадру, `direction`, `target_x`,
  `target_y`, `error_x`, `error_y` і правила перетворення координат цілі на
  команди гімбала та yaw-серво.
  Діаграма явно показує, що `/turret/status` публікує
  `turret_controller_node`.
  README явно вказує, що `turret_controller_node`, `gimbal_driver_node` і
  `yaw_servo_driver_node` реалізуються студентами, а не надаються у
  стартовому коді.
  README уточнює склад чистої C++ логіки контролера і мінімальний обсяг
  тестів: хоча б один тест на кожну частину рішення.
  README посилається на `demos/lesson_5_4/06_logic_split` як конкретний
  приклад розділення чистої C++ логіки, ROS 2 wrapper-ноди і тестів.
  Sequence diagram у постановці ДЗ має короткі псевдо-назви інстансів для
  нод.
  `system.launch.py` має аргумент `log_level` для ввімкнення debug-логів під
  час діагностики.

### Changed

- Block 5 / Lesson 5.4 / Homework 13: `system.launch.py` тепер за
  замовчуванням запускає `track:=all`, тобто всі готові CSV-треки послідовно.
  Окремий трек можна обрати через `track:=...`. README уточнює, що
  `TARGET_LOW_CONFIDENCE` означає `confidence < 0.80`, а `reload_pressure.csv`
  містить сценарій, де другий FPV може дійти до захищеної зони, поки актуатор
  ще у `RELOADING`.
- Block 5 / Lesson 5.4: `.gitignore` ігнорує Python bytecode
  `__pycache__/` та `*.py[cod]`, які можуть з'являтися після локальної
  перевірки launch-файлів.
- Block 5 / Lesson 5.4 / Homework 13: component diagram у README тепер має
  короткі псевдо-назви інстансів нод (`tgt`, `ctrl`, `gmb`, `yaw`, `act`) поруч
  із реальними назвами ROS 2 нод.

## 2026-06-30

### Added

- Block 5 / Lesson 5.3: додано ROS 2 pub/sub demo `demos/lesson_5_3`
  з пакетом `robot_telemetry`, запуском через `colcon`, CLI-перевірками graph
  і сценарієм навмисно зламаного topic name.
- Block 5 / Lesson 5.3: devcontainer переведено на `osrf/ros:jazzy-desktop`
  і доповнено ROS 2 інструментами для `colcon`, `ament_auto` та `rqt_graph`.

### Changed

- Block 5 / Lesson 5.3: `status_monitor` друкує стартовий log, а README
  показує перевірку активного ROS 2 middleware.
- Block 5 / Lesson 5.3: devcontainer використовує CycloneDDS
  (`rmw_cyclonedds_cpp`) як default ROS 2 middleware.
- Block 5 / Lesson 5.3: додано `CYCLONEDDS_URI` config для host-network
  devcontainer: host interface autodetect, multicast loopback і local peer
  `127.0.0.1`.
- Block 5 / Lesson 5.3: devcontainer синхронізує ROS 2 CLI daemon у
  `postStartCommand`, щоб `ros2 node list` і `ros2 topic list` працювали без
  додаткових прапорів.

## 2026-06-29

### Added

- Block 5 / Lesson 5.1: додано Docker runtime demos для multi-stage build,
  `docker build`, `docker run`, volumes і healthcheck.

## 2026-06-26

### Added

- Block 5 / Lesson 5.2 / Homework 12: додано стартовий C2-сервiс для НРК,
  ArduRover SITL compose stack, QGC-маршрут бiля Лимана та `edge/docker-compose.yml`
  з готовим `auto_stub` для Guided-сценарiю.

## 2026-05-17

### Added

- Block 2 / Lesson 2.6: додано `docs/code-quality.md` з базовими командами
  для запуску `clang-format`, `cmake-format`, `clang-tidy`, налаштуванням
  VS Code auto-format, optional VS Code task для `clang-tidy` і правилами
  точкового використання `NOLINT` без wrapper scripts.

## 2026-05-08

### Changed

- Block 2 / Lesson 2.6: послаблено `.clang-tidy` для ДЗ 6. Зауваження static
  analyzer лишаються помилками, а style/modernize/core-guidelines сигнали
  лишаються warning-ами для ручного розбору.

## 2026-04-30

### Added

- Block 2 / Lesson 2.4: `debug` CMake preset, VS Code launch config для
  `homework_04` і build task, який використовує Debug-збірку через preset.
- Block 2 / Lessons 2.4-2.5: стартовий код `homework_05` для діагностики
  телеметрії з C++ бібліотекою, виконуваним файлом, проблемними вхідними
  файлами і навмисними помилками під час виконання.
  `CMakeLists.txt` для `homework_05` не додано навмисно, це частина ДЗ 5.
- Block 2 / Lesson 2.4: demo `demos/lesson_2_4/debug_probe` для локального
  консольного GDB, core dump, Valgrind і віддаленого GDB/gdbserver на
  Raspberry Pi / ARM64-пристрої.
- Block 2 / Lesson 2.4: ARM64 cross-compilation preset `aarch64-debug` і
  CMake toolchain `cmake/toolchains/aarch64-linux-gnu.cmake`.
- Block 2 / Lessons 2.3-2.4: VS Code tasks для CMake configure/build і
  видалення локальної `build/` директорії. `CMake: configure and build`
  зроблено default build task для `Ctrl+Shift+B`.
  ([#10](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/10))
- Block 2 / Lessons 2.3-2.6: clang-tidy конфіг, debug/diagnostics tools
  (`gdb`, `gdbserver`, `gdb-multiarch`, `valgrind`) і налаштування clangd через
  `build/compile_commands.json`.
  ([#9](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/9))

### Changed

- Block 2 / Lessons 2.1-2.6: оновлено setup docs під GitHub Template flow,
  додано пояснювальні коментарі до student-facing tooling/config файлів і
  зменшено `.gitignore` до мінімального C++/CMake набору.
  ([#9](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/9))
- Block 2 / Lesson 2.4: clangd тепер читає compile database з
  `build/debug/compile_commands.json`, щоб відповідати `debug` preset.
- Block 2 / Lesson 2.4: README/preps і CI build flow переведено на
  `cmake --preset debug` і `cmake --build --preset debug`.
- Block 2 / Lesson 2.4: CI build flow додатково перевіряє
  `aarch64-debug` cross-build.
- Block 2 / Lesson 2.4: remote GDB інструкції уточнено: `satelite` лишається
  SSH alias, а `target remote` використовує IP пристрою.
- Block 2 / Lesson 2.4: core dump інструкції доповнено fallback-сценарієм для
  WSL/Docker, де core-файл може перехоплюватись системним handler-ом.
- Block 2 / Lesson 2.4: devcontainer runtime args доповнено `SYS_PTRACE`,
  `seccomp=unconfined` і `core=-1` для GDB/core dump сценаріїв.
- Block 2 / Lesson 2.4: core dump README доповнено WSL host setup через
  `kernel.core_pattern`, перевіркою `core.%e.%p` після rebuild/reopen
  devcontainer і запуском GDB по фактичному `core*` файлу.
- Block 2 / Lesson 2.4: core dump README уточнює, що `ulimit -c unlimited`
  потрібно виконати у поточній shell-сесії, якщо перевірка показує `0`.
- Block 2 / Lesson 2.4: devcontainer отримав `libclang-rt-18-dev` і
  `llvm-18`, щоб ASan/UBSan збірка через Clang лінкувалась і показувала
  читабельний stack trace з file:line.
- Block 2 / Lesson 2.4: `.gitignore` ігнорує локальні `core` / `core.*`
  файли, які з'являються під час core dump demo.
- Block 2 / Lesson 2.4: clang-tidy diagnostics у clangd вимкнено до заняття
  2.6 через devcontainer settings і `.clangd`; конфіг і пакет clang-tidy
  лишаються в repo/container для майбутнього ввімкнення.
- Block 2 / Lesson 2.4: `initializeCommand` запускає
  `.devcontainer/scripts/initialize` через `bash`, щоб копіювання файлів без
  executable bit не ламало старт devcontainer.
- Block 2 / Lesson 2.4: README/preps доповнено snapshot sync flow через
  `git clone` + `tar`, `git status` і commit, щоб оновлення курс-репо не
  залежали від ручного списку файлів.

## 2026-04-25

### Changed

- Devcontainer user setup спростили. Прибрали хардкод `useradd -u 1000`;
  покладаємось на `updateRemoteUserUID` від VS Code і `devcontainers/ci`, який
  синхронізує UID контейнерного користувача з host-ом на старті. Dockerfile
  тепер лише видаляє дефолтного `ubuntu` умовно через `if id ubuntu`, щоб
  звільнити слот UID 1000.
  ([#5](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/5))

### Added

- GitHub Actions workflow `build.yml` через `devcontainers/ci@v0.3`. CI будує
  проєкт у тому самому devcontainer-і, що й локальний VS Code, запускається на
  push у `main` і на кожен PR.
  ([#4](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/4))

## 2026-04-23

### Added

- Пакети `ssh` і `git` у devcontainer з коробки. Базовий git-workflow і
  SSH-операції тепер працюють без донастройки.
  ([#3](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/3))
- Стартові матеріали блоку 2: корневий `CMakeLists.txt`, `homework_04/`
  (starter для wheel odometry), `.devcontainer/` з Dockerfile і
  initialize-скриптом, `preps/` з інструкціями для Linux, macOS, Windows 11
  + WSL2 і Dev Containers CLI.
