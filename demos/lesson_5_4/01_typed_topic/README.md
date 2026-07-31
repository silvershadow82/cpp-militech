# 01_typed_topic

Приклад показує власний `.msg`, типізований publisher і типізований
subscriber.

Корисно для ДЗ 13, коли потрібно додати `GimbalCommand.msg`,
`ServoCommand.msg`, `TurretStatus.msg` і працювати з ними без парсингу рядків.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_4/01_typed_topic/robot_ws
colcon build --symlink-install --packages-select typed_topic_demo
source install/setup.bash
```

## Run

Термінал 1:

```bash
ros2 run typed_topic_demo target_publisher
```

Термінал 2:

```bash
ros2 run typed_topic_demo target_subscriber
```

Перевірка:

```bash
ros2 topic info /demo/target
ros2 topic echo /demo/target --once
```

Очікуваний тип:

```text
typed_topic_demo/msg/Target
```

## Файли

- `msg/Target.msg` - власний тип повідомлення;
- `src/target_publisher.cpp` - типізований publisher;
- `src/target_subscriber.cpp` - типізований subscriber;
- `include/typed_topic_demo/target_formatter.hpp` - приклад include
  generated type у `.hpp`;
- `CMakeLists.txt` - генерація `.msg` і підключення typesupport до targets.
