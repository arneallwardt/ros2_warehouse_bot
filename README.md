# Autonomous warehouse robot with ROS2

## Set Up

### General 

- [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (only for Ubuntu 22.04.)
- Add Packages for TurtleBot3: `sudo apt install ros-humbe-turtlebot3*`
- Install Gazebo: `sudo apt install ros-humble-gazebo-ros-pkgs`
- Install nav2: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- Install colcon `sudo apt install python3-colcon-common-extensions`
- Install twist_mux: `sudo apt install ros-humble-twist-mux`
- Install slam_toolbox: `sudo apt install ros-humble-slam-toolbox`
- Add xacro support: `sudo apt install ros-humble-xacro`
- Install GUI for joint-state-publisher: `sudo apt install ros-humble-joint-state-publisher-gui`

### OpenManipulator
[Quick start guide](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/)

### Package specific

- Check for updates in the package: `rosdep install -i --from-path src --rosdistro humble -y`
- Build package (execute from inside the workspace): `colcon build —symlink-install`
- Source the package: `source install/setup.bash`

### Finding errors
- run `ros2 doctor` to check for errors. If you get `All <n> checks passed` as a result, you're good to go.
- run `ros2 doctor --report` to receive a full report

## ROS2 packages

- Nodes are definded in `src/<package_name>/<package_name>`

### package.xml

This file contains all the necessary information about a package. Including all the dependencies needed to build / test / run the code. Types of dependencies are the following:
- **test_depend**: used in testing the code
- ** build_depend**: used in building the code
- **build_export_depend**: needed by headers in the code exports
- **exec_depend**: only used when running the code
- **depend**: for mixed purposes; covers build, export and execution time dependencies

Every library which is imported inside a .py file, must be specified as a dependency in the `package.xml`!

You can check if all dependencies are available and install them if required by running `rosdep install -i --from-path src --rosdistro <your_ros_distro> -y`

### setup.py

This file contains information about entry points and launch files. If you want to execute nodes / services with `ros2 run`, they need to be specified here either as `entry_points` (single nodes) or `data_files` (launch files).

### Running a package
- navigate to workspace directory
- build packages: `colcon build`
- source underlying: `source install/setup.bash`
- run
  - single node: `ros2 run <package_name> <node_name>` (NOTE: the `node_name` is specified in `setup.py` `entry_points`)
  - launch file: `ros2 run <package_name> <launch_file>`

## Using slam_toolbox and nav2
- *sim_time:=true ONLY FOR GAZEBO*
- you might want to build the `warehouse_bot` package first by executing `colcon build` in the packages root directory 
- remember to execute `source /home/kilab/ros2_warehouse_bot/install/setup.bash` in every shell you open
- set `map_file_name` in [mapper_params_localization.yaml](./src/warehouse_bot/config/mapper_params_localization.yaml) and to the path where your map files are. You can find them in `/maps/new` but you have to provide the rest of the absolute path starting with `/home`...

### Mapping
- start mapping (including twist_mux, scan_filter, slam_toolbox, rviz2): `ros2 launch warehouse_bot warehouse_bot_mapping_launch.py`

- **Saving Map**
    - add slam_toolbox panel to rviz
    - save → old format to use with external libs like nav2
    - serialize → new format to use with slam_toolbox

### Navigation (slam_toolbox and nav2)
- start the bot itself (in bots shell; includes turtlebot3_bringup, scan_filter, image_provider): `ros2 launch warehouse_bot warehouse_bot_tb_launch.py`
- start localization (includes twist_mux, slam_toolbox, rviz2): `ros2 launch warehouse_bot warehouse_bot_localization_launch.py`
- start navigation: `ros2 launch warehouse_bot warehouse_bot_navigation_launch.py`

## Misc

- use this to reset turtlebot movement: `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'`