# ros2_parte2
Entrega código para la parte práctica de parcial corte uno. Juliana Chaparro Á y Nicolás Guzmán.
## Cómo compilar
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

## Cómo ejecutar (3 terminales)
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
ros2 run turtle_modes turtle_mode_controller

## Modos (servicio)
ros2 service call /turtle_modes/set_mode turtle_modes_interfaces/srv/SetMode "{mode: 0, circle_dir: 0}"
ros2 service call /turtle_modes/set_mode turtle_modes_interfaces/srv/SetMode "{mode: 1, circle_dir: 1}"
ros2 service call /turtle_modes/set_mode turtle_modes_interfaces/srv/SetMode "{mode: 2, circle_dir: 0}"

## Acción trayectoria
ros2 action send_goal /turtle_modes/follow_trajectory turtle_modes_interfaces/action/FollowTrajectory "{start: true}"
