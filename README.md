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
@reboot ~/reset_ros_config.sh
```