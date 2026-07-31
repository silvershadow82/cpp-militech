# 06_logic_split

Приклад показує розділення чистої C++ логіки і ROS 2 wrapper-ноди.

Корисно для ДЗ 13, де логіка контролера турелі не має бути пов'язана з
`rclcpp`, publisher, subscriber або service client.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_4/06_logic_split/robot_ws
colcon build --symlink-install --packages-select logic_split_demo
source install/setup.bash
```

## Tests

```bash
colcon test --packages-select logic_split_demo
colcon test-result --verbose
```

## Run

```bash
ros2 run logic_split_demo decision_node
```

## Файли

- `robot_ws/src/logic_split_demo/include/logic_split_demo/target_decision.hpp`
  - чиста C++ логіка без `rclcpp`;
- `robot_ws/src/logic_split_demo/src/decision_node.cpp` - ROS 2 wrapper-нода;
- `robot_ws/src/logic_split_demo/test/target_decision_test.cpp` - unit tests
  для чистої логіки.
