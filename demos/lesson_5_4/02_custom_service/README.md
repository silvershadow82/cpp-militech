# 02_custom_service

Приклад показує власний `.srv`, service server і service client.

Корисно для ДЗ 13, коли потрібно викликати `/actuator/trigger` з C++ ноди.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_4/02_custom_service/robot_ws
colcon build --symlink-install --packages-select custom_service_demo
source install/setup.bash
```

## Run

Термінал 1:

```bash
ros2 run custom_service_demo trigger_server
```

Термінал 2:

```bash
ros2 run custom_service_demo trigger_client 0.9 25.0
```

CLI-перевірка:

```bash
ros2 service list
ros2 service type /demo/trigger
ros2 service call /demo/trigger custom_service_demo/srv/TriggerActuator "{confidence: 0.9, distance_m: 25.0}"
```

Очікуваний тип сервісу:

```text
custom_service_demo/srv/TriggerActuator
```

## Файли

- `srv/TriggerActuator.srv` - власний тип сервісу;
- `src/trigger_server.cpp` - service server;
- `src/trigger_client.cpp` - service client з callback;
- `CMakeLists.txt` - генерація `.srv` і підключення typesupport до targets.
