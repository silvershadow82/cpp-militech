# 05_launch

Приклад показує launch-файл, запуск кількох нод і перемикання YAML-профілю
через launch argument.

Корисно для ДЗ 13, де потрібно розширити `system.launch.py` новими нодами і
передати параметри контролера.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_4/05_launch/robot_ws
colcon build --symlink-install --packages-select launch_demo
source install/setup.bash
```

## Run

```bash
ros2 launch launch_demo system.launch.py
```

Профіль можна змінити без перезбірки:

```bash
ros2 launch launch_demo system.launch.py profile:=field.yaml
```

Перевірка:

```bash
ros2 node list
ros2 param get /sensor_process publish_hz
ros2 param get /controller_process mode
```

## Файли

- `src/sensor_process.cpp` - перша нода;
- `src/controller_process.cpp` - друга нода;
- `config/sim.yaml` і `config/field.yaml` - різні профілі;
- `launch/system.launch.py` - launch argument `profile` і запуск двох нод.
