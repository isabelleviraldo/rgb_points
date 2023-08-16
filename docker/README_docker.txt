
***in each window, do $ source /opt/ros2_ws/install/setup.bash***
MUST BE RUNNING LIDAR AND ZED CAMERA
MUST BE RUNNING AHC AHC_node: do $ cat AHC_info.txt for details

- to run the node, do once: $ ros2 run rgb_points <do $ cat MENU.txt to see options>

Opening more windows in the same docker image:
1. on local machine, find container NAMES with $ docker ps
2. on local machine, $ docker exec -it <container_name> bash
-------------------------------------------------------------
