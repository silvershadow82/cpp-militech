# Demo для заняття 5.4

Маленькі незалежні ROS 2 приклади для ДЗ 13. Кожна папка - окремий малий
workspace з одним пакетом, README і мінімальним кодом під одну тему.

Це навчальний код, а не готове рішення ДЗ. Приклади показують форму роботи з
ROS 2, але не містять повний `antidrone_turret` pipeline.

## Приклади

```text
demos/lesson_5_4/
  01_typed_topic/
    robot_ws/src/typed_topic_demo/
  02_custom_service/
    robot_ws/src/custom_service_demo/
  03_service_status_topic/
    robot_ws/src/service_status_demo/
  04_parameters_yaml/
    robot_ws/src/parameters_demo/
  05_launch/
    robot_ws/src/launch_demo/
  06_logic_split/
    robot_ws/src/logic_split_demo/
```

## Що де шукати

- `01_typed_topic` - власний `.msg`, типізований publisher, типізований
  subscriber, generated type у `.cpp` і `.hpp`;
- `02_custom_service` - власний `.srv`, service server, service client з
  callback після `async_send_request(...)`;
- `03_service_status_topic` - service server, який після команди публікує
  типізований топік статусу;
- `04_parameters_yaml` - параметри ноди і YAML-файл;
- `05_launch` - launch-файл, launch argument і запуск кількох нод з YAML;
- `06_logic_split` - чиста C++ логіка без `rclcpp`, ROS 2 wrapper-нода і
  unit tests.

## Загальний build pattern

Для кожного прикладу заходити у його `robot_ws` і збирати тільки відповідний
пакет:

```bash
source /opt/ros/jazzy/setup.bash
cd repository/demos/lesson_5_4/01_typed_topic/robot_ws
colcon build --symlink-install --packages-select typed_topic_demo
source install/setup.bash
```

Назву папки і пакета змінювати відповідно до прикладу.
