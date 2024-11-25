# Autonomous warehouse robot with ROS2

## Set Up

### General 

- [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (only for Ubuntu 22.04.)
- Add Packages for TurtleBot3: `sudo apt install ros-humbe-turtlebot3*`
- Install Gazebo: `sudo apt install ros-humble-gazebo-ros-pkgs`
- Install nav2: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- Install colcon `sudo apt install python3-colcon-common-extensions`
- Install twist_mux: `sudo apt install ros-humble-twist-mux`
- Add xacro support: `sudo apt install ros-humble-xacro`
- Install GUI for joint-state-publisher: `sudo apt install ros-humble-joint-state-publisher-gui`

### Package specific

- Check for updates in the package: `rosdep install -i --from-path src --rosdistro humble -y`
- Build package (execute from inside the workspace): `colcon build —symlink-install`
- Source the package: `source install/setup.bash`

## Using slam_toolbox / amcl with nav2

### Mapping
- *sim_time:=true ONLY FOR GAZEBO*
- `mode: mapping` and `scan_topic: /scan/filtered` in mapper_params_online_async.yaml
- launch gazebo: `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py`
    - if it is not working try `source /usr/share/gazebo/setup.sh`
- start teleoperating node: `ros2 run turtlebot3_teleop teleop_keyboard`
- launch scan filtering node (only when arm is mounted on robot): `python3 ros2_warehouse_bot/misc/scan_filter.py`
- start mapping with slam toolbox: `ros2 launch slam_toolbox online_async_launch.py slam_params_file:=<path_to_ws>/src/test_package/config/mapper_params_online_async.yaml use_sim_time:=true`
- run rviz: `rviz2`
    - set *Fixed Frame = map* in rviz

- **Saving Map**
    - add slam_toolbox panel to rviz
    - save → old format to use with external libs like nav2
    - serialize → new format to use with slam_toolbox

### Localization
- *sim_time:=true ONLY FOR GAZEBO*
- **mapper_params_localization.yaml** (if slam_toolbox is used)
    - `mode: localization`
    - `map_file_name: <path_to_ws>/maps/new/turtlebot3_house_serialize`
    - `map_start_at_dock: true` (= start, where bot starded while mapping)
- run map server to publish map on topic /map: `ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<path_to_ws>/maps/old/turtlebot3_house_save.yaml -p use_sim_time:=true`
    - start map_server: `ros2 run nav2_util lifecycle_bringup map_server`
- start gazebo: `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py`
- start rviz: `rviz2`
    - if map is not loaded: Map -> Topic -> Durability Profile -> Transient Local
- run amcl: `ros2 run nav2_amcl amcl --ros-args -p use_sim_time:=true`
    - start amcl: `ros2 run nav2_util lifecycle_bringup amcl`
    - set initial pose in rviz

### Navigation 
- *sim_time:=true ONLY FOR GAZEBO*
- start twist_mux (because teleop and nav2 publish on different topics): `ros2 run twist_mux twist_mux --ros-args --params-file <path_to_ws>/src/test_package/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped`
- start gazebo: `ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py`
- start rviz: `rviz2`
- **start localization**
    - slam_toolbox: `ros2 launch slam_toolbox localization_launch.py slam_params_file:=<path_to_ws>/src/test_package/config/mapper_params_localization.yaml use_sim_time:=true`
    - **OR** nav2: `ros2 launch nav2_bringup localization_launch.py map:=/home/arne/ros_workspaces/ros2_warehouse_bot/maps/old/turtlebot3_house_save.yaml use_sim_time:=true`
- start nav2: `ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true`
    - default parameters for nav2 are set up for *TurtleBot3*, no need to change anything
- optional: add *costmap* to rviz: Map -> *Topic = /global_costmap/costmap* / *Color Scheme = costmap*
- 