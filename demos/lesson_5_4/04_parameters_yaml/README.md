# 04_parameters_yaml

Приклад показує параметри ROS 2 ноди і YAML-файл.

Корисно для ДЗ 13, де `turret_controller_node` має читати
`confidence_threshold` і `max_distance_m`.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_4/04_parameters_yaml/robot_ws
colcon build --symlink-install --packages-select parameters_demo
source install/setup.bash
```

## Run без YAML

```bash
ros2 run parameters_demo parameter_reader
```

## Run з YAML

```bash
ros2 run parameters_demo parameter_reader --ros-args --params-file src/parameters_demo/config/demo.yaml
```

Перевірка:

```bash
ros2 param list
ros2 param get /parameter_reader confidence_threshold
ros2 param get /parameter_reader max_distance_m
ros2 param get /parameter_reader publish_hz
```

## Файли

- `src/parameter_reader.cpp` - `declare_parameter(...)` і читання параметрів;
- `config/demo.yaml` - YAML для запуску;
- `CMakeLists.txt` - встановлення `config/` у `share` пакета.
