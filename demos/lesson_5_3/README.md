# Demo для заняття 5.3

ROS 2 pub/sub demo для першого проходу по `rclcpp`, `colcon` і `ros2` CLI.

Команди нижче запускати всередині git-репозиторію. Перехід у demo зроблено через
`git rev-parse`, тому він працює і з кореня репозиторію, і з вкладеної папки.

```bash
cd "$(git rev-parse --show-toplevel)/demos/lesson_5_3"
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select robot_telemetry
source install/setup.bash
```

У devcontainer CycloneDDS увімкнено за замовчуванням:

```bash
echo "${RMW_IMPLEMENTATION:-unset}"
echo "${CYCLONEDDS_URI:-unset}"
```

Очікувані значення:

```text
rmw_cyclonedds_cpp
file:///etc/cyclonedds.xml
```

CycloneDDS config використовує host network interface autodetect, multicast
loopback і локальний peer `127.0.0.1`. Якщо значення `unset` або інші,
перебудувати devcontainer і відкрити новий термінал.

Devcontainer також синхронізує ROS 2 CLI daemon під час `postStartCommand`.
Тому `ros2 node list` і `ros2 topic list` нижче виконуються у звичайному
daemon-mode без додаткових прапорів.

## Запуск

Термінал 1:

```bash
cd "$(git rev-parse --show-toplevel)/demos/lesson_5_3"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_telemetry telemetry_publisher
```

Термінал 2:

```bash
cd "$(git rev-parse --show-toplevel)/demos/lesson_5_3"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_telemetry status_monitor
```

Термінал 3:

```bash
cd "$(git rev-parse --show-toplevel)/demos/lesson_5_3"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 node list
ros2 topic list
ros2 topic info /robot/telemetry --verbose
ros2 topic echo /robot/telemetry --once
ros2 topic hz /robot/telemetry
```

`telemetry_publisher`, `status_monitor` і `ros2 topic hz` працюють до `Ctrl+C`.
`status_monitor` одразу друкує `listening on /robot/telemetry`. Перші
`telemetry: ...` мають з'явитися після запуску `telemetry_publisher`.

Якщо `ros2 topic echo` або `ros2 topic hz` нічого не показують, перевірити, що
publisher запущений і topic називається саме `/robot/telemetry`.

Якщо `ros2 node list` або `ros2 topic list` зависають чи повертають порожній
результат, перевірити, що термінал відкрито після rebuild devcontainer і
активний саме локальний CycloneDDS config:

```bash
echo "${RMW_IMPLEMENTATION:-unset}"
echo "${CYCLONEDDS_URI:-unset}"
```

Якщо після цього graph все ще порожній, перевірити однаковий `ROS_DOMAIN_ID` у
всіх терміналах:

```bash
echo "${ROS_DOMAIN_ID:-unset}"
```

Необов'язкова візуалізація graph:

```bash
rqt_graph
```

Якщо GUI не відкривається, достатньо CLI-команд вище.

## Навмисна поломка

1. Зупинити `telemetry_publisher`.
2. У `robot_telemetry/src/telemetry_publisher.cpp` змінити topic:

```cpp
constexpr auto kTelemetryTopic = "/robot/telem";
```

3. Перезібрати package:

```bash
colcon build --symlink-install --packages-select robot_telemetry
source install/setup.bash
```

4. Знову запустити `telemetry_publisher`.
5. Перевірити graph:

```bash
ros2 topic list
ros2 topic info /robot/telemetry
ros2 topic info /robot/telem
```

Очікуваний висновок: publisher і subscriber працюють, але не зустрілися на
одному topic.
