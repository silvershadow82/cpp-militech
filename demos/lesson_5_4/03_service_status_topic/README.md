# 03_service_status_topic

Приклад показує service server, який після прийнятої команди публікує
типізований топік статусу.

Корисно для ДЗ 13, де `actuator_node` має сервіс `/actuator/trigger` і топік
`/actuator/status`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_4/03_service_status_topic/robot_ws
colcon build --symlink-install --packages-select service_status_demo
source install/setup.bash
```

## Run

Термінал 1:

```bash
ros2 launch service_status_demo actuator.launch.py
```

Термінал 2:

```bash
ros2 service call /demo/trigger service_status_demo/srv/TriggerActuator "{confidence: 0.9, distance_m: 25.0}"
ros2 topic echo /demo/actuator_status --once
```

Очікувані типи:

```text
/demo/trigger           service_status_demo/srv/TriggerActuator
/demo/actuator_status   service_status_demo/msg/ActuatorStatus
```

## Файли

- `srv/TriggerActuator.srv` - команда пострілу як сервіс;
- `msg/ActuatorStatus.msg` - типізований топік статусу;
- `src/actuator_server.cpp` - service server + status publisher;
- `config/actuator.yaml` - параметри `reload_ms` і `status_publish_hz`;
- `launch/actuator.launch.py` - запуск ноди з YAML.
