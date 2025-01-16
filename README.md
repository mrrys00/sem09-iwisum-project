# sem09-iwisum-project

## How to run?

```
make prepare_pc
```

then in separate terminals:

start gazebo simulation node:

```
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

start navigation node:

```
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

start SLAM toolbox node:

```
source install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node
```

start rviz simulation:

```
source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```

start map to json node:

```
source install/setup.bash
ros2 run map_json_node map_json_node
```
