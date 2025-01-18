# sem09-iwisum-project

## How to run?

```bash
make prepare_pc
```

then in separate terminals in prepared `ros2_workspace`:

start gazebo simulation node:

```bash
source install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

start navigation node:

```bash
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false
```

start SLAM toolbox node:

```bash
source install/setup.bash
ros2 run slam_toolbox async_slam_toolbox_node
```

start rviz simulation:

```bash
source install/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```

start map predictor node:

```bash
source install/setup.bash
ros2 run map_predictor map_predictor --ros-args --params-file src/configs/map_predictor.yaml
```

map_predictor.yaml description:

```yaml
map_processor_node:
  ros__parameters:
    output_directory: "messages"  # output directory for jsons
    save_json: true               # save metrics and grids in jsons
    fixed_fig_size: -1            # setup greatest figure size; if -1 -> it will be calculated automatically while running
```
