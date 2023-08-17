# rgb_points

Author: [@isabelleviraldo](https://github.com/isabelleviraldo)

Release Date: 08/18/2023


## Table of Contents

- [Intro](#intro)
- [Running rgb_points](#running-rgb_points)
- [Nodes In Depth](#nodes-in-depth)
	- [initial_attempt](#initial_attempt)
	- [fixed_math](#fixed_math)
  - [color_objects_using_center](#color_objects_using_center)
  - [few_colors](#few_colors)
- [Inputs and Outputs](#inputs-and-outputs)

## Intro


## Running rgb_points

To run rgb_points, you can build [this Dockerfile](https://github.com/isabelleviraldo/rgb_points/blob/main/docker/rgb_points_dockerfile/Dockerfile) on a device with an NVIDIA card

To open a new window on any Docker container do:

```sh
$ docker exec -it <container_name> bash
```

Once built, each window must do:

```sh
$ source /opt/ros2_ws/install/setup.bash
```

To enter the [internal README](https://github.com/isabelleviraldo/rgb_points/blob/main/docker/README_docker.txt) do:

```sh
$ cat README.txt
```

The following requirements must be met before any node within rgb_points can be run:

1. Running Lidar and Zed2i Camera
2. Running [@hazelrah2](https://github.com/hazelrah2)'s AHC file ([Dockerfile](https://github.com/isabelleviraldo/rgb_points/blob/main/docker/AHC_dockerfile/Dockerfile))

If those requirements are met, you can find which version is right for you through the [internal MENU](https://github.com/isabelleviraldo/rgb_points/blob/main/docker/MENU_docker.txt) by doing:

```sh
$ cat MENU.txt
```

When viewing rgb_points' output in rviz2, use [this configuration](https://github.com/isabelleviraldo/rgb_points/blob/main/rviz2/rgb_points_config.rviz)


## Nodes In Depth

[@hazelrah2](https://github.com/hazelrah2)'s AHC file MUST be running for these to work.

Inputs: 
- /lidar_0/AHC/clusters (Point Cloud)
- /zed2i/zed_node/left/image_rect_color (Image)

Output: 
- /rgb_points/testing (Point Cloud)


### initial_attempt


### fixed_math


### color_objects_using_center


### few_colors


## Inputs and Outputs


 
