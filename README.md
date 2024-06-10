# ROS2 Notes

## Rqt Tree
ROS2 does not have the equivalent of rosrun rqt_tf_tree rqt_tf_tree or rqt_tree. In order to see the tf tree:
 - Install rqt_tf_tree package from rqt_tf_Tree repo (in ROS Index):
```
sudo apt install ros-humble-rqt-tf-tree
```
Then, use directly rqt, under the Plugins Tab > Visualization Tab > TF Tree. Alternatively, you can run: `ros2 run rqt_tf_tree rqt_tf_tree --force-discover`.

Otherwise, you have to run: `ros2 run tf2_tools view_frames`, which scans for 5 s the TFs and saves a pdf file. Note: Keep attention that it saves a file where this command is executed.

## Additional features for colcon:
In order to have similar behavior of roscd, and autocompletion in colcon usage add the following in .bashrc:
```
# Colcon Setup - Start
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/humble/
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# Colcon setup - End
```

## Nav2 BT Nodes:
All bt navigation nodes can be seen here (https://github.com/ros-planning/navigation2/blob/main/nav2_behavior_tree/nav2_tree_nodes.xml), if you installed Nav2 you can find it here:
```
/opt/ros/humble/share/nav2_behavior_tree
```
You can add them in Groot Editor.

## Multiple ROS2 Version
If you have multiple ROS2 installed version you can customize your .bashrc in such a way to source by default "humble" version and source the version that you would like throw a bash function. Add the following in .bashrc:
```
#Function for source all the necessary things - Start
function set_ros_env() {
    if [ $# -eq 0 ]; then
        echo "Errore: Specificare una desired_version"
        return 1
    fi
    local desired_version=$1


    # Source ROS version
    source "/opt/ros/$desired_version/setup.bash"
    echo "ROS 2 version: $desired_version is sourced"

    # Source turtlebot examples / models
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$desired_version/share/turtlebot3_gazebo/models

    # Colcon Setup - Start
    source /usr/share/colcon_cd/function/colcon_cd.sh
    export _colcon_cd_root=/opt/ros/$desired_version/
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    # Colcon setup - End
}

#Function for source all the necessary things - Stop

# Gazebo
source /usr/share/gazebo/setup.bash

#Plansys
#source ~/plansys2_ws/install/setup.bash

# ROS settings
export ROS_DEFAULT_VERSION="humble"
export ROS_CONFIG_FILE="$HOME/.ros_config"      # It is cleaned  in ~/reset_ros_config.sh at startup in crontab -e

if [ -f "$ROS_CONFIG_FILE" ] && [ -s "$ROS_CONFIG_FILE" ]; then #Exist and not empty
    source "$ROS_CONFIG_FILE"
    export desired_version="$ACTUAL_ROS_VERSION"
else
    export desired_version="$ROS_DEFAULT_VERSION"
fi

set_ros_env $desired_version

# Function for changing ros version - Start
function set_ros_version() {
    if [ $# -eq 0 ]; then
        # No version specified, use the default version
        export desired_version="$ROS_DEFAULT_VERSION"
    else
        export desired_version=$1

        # Check if the desired version is installed in /opt/ros
        if [ -d "/opt/ros/$desired_version" ]; then
            echo "ROS2 version $desired_version is installed in /opt/ros"
            set_ros_env $desired_version
        else
            echo "Error: ROS2 version $desired_version is not installed in /opt/ros"
            return 1
        fi

        # Save the desired version permanently
        echo "export ACTUAL_ROS_VERSION=\"$desired_version\"" > "$ROS_CONFIG_FILE"
    fi
}
# Function for changing ros version - Stop

```

Create another file "reset_ros_config.sh":
```
#!/bin/bash
ROS_CONFIG_FILE="$HOME/.ros_config"

if [ -f "$ROS_CONFIG_FILE" ]; then
    # Svuota il contenuto del file
    echo -n > "$ROS_CONFIG_FILE"
    echo "Contenuto di $ROS_CONFIG_FILE è stato rimosso."
else
    echo "$ROS_CONFIG_FILE non esiste o è vuoto."
fi
```

In such a way to call this reset_ros_config.sh every time you reboot you notebook:
```
crontab -e
```
copy and paste this:


```
@reboot ~/reset_ros_config.sh
```

## Kill ROS Deamons
```
ps aux | grep ros2 	# To check ros2-deamons 
pkill -9 -f ros2 	# To kill them
```


## Robots Communications
ROS2 works on UDP and DDS, Fast DDS or Cyclone dds. Tiago use Cyclon DDS. 
Usefull commands are:
```
sudo apt install ros-rolling-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Docker TIPS
To run Docker with display abilities (This for Intel): 
```
docker run -it \
    --volume=/tmp/.X11-unix:/tmp/.X11-unix \
    --device=/dev/dri:/dev/dri \
    --env="DISPLAY=$DISPLAY" \
    osrf/ros:noetic-desktop-full (image name)
```
If you want to enable others Graphics Accelerator as Nvidia follow this [link](http://wiki.ros.org/action/login/docker/Tutorials/Hardware%20Acceleration#nvidia-docker2).

## rqt_image_view
```
sudo apt install ros-humble-rqt-image-view
```

## Debug
Or: 
```
build/plansys2_executor/test/unit/executor_test
```

Or run with gdb:
```
gdb build/plansys2_executor/test/unit/executor_test
```
then "run".

## Fix uncrustify issue:
ament_uncrustify path --reformat

## Flake8 & PyLint
Flake8 is a command-line utility for enforcing style consistency across Python projects.
Pylint is a static code analysis tool for the Python programming language.


## Tips for compilation command (vscode autoc.):
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
alias mcc=$(find . -iname "compile_commands.json" | xargs jq -s 'map(.[])' > compile_commands.json)

## Debug tips:
[Backward ROS](https://github.com/pal-robotics/backward_ros):
Add backward_ros to your package.xml:
```
<depend>backward_ros</depend>
```

Add backward_ros to your CMakeLists.txt:
```
find_package(backward_ros REQUIRED)
```

Basic debug: 
```
set(CMAKE_BUILD_TYPE Debug)
```
## Turtlebot4

Commands to put into .bashrc to run at startup lidar and tf static broadcaster `base_link` -> `lidar`:

```
# This portion is for launch lidar & static publisher just as sourced this bashrc the first time.
# Lockfile is created in such a way to check if it is already created in that case that launches are not called.
LOCKFILE="/tmp/.ros2_launch_lock"
if [ ! -f "$LOCKFILE" ]; then
    echo "Calling startup launches for lidar and tf static base_link -> laser"
    touch "$LOCKFILE"
    ros2 launch rplidar_ros rplidar_s2_launch.py ns:=$NAMESPACE
    ros2 run tf2_ros static_transform_publisher 0 0 0.1 3.14 0 0 base_link laser --ros-args -r /tf_static:=/$NAMESPACE/tf_static
fi
```
