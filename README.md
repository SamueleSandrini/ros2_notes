# ros2_notes

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