# rgb_points

Author: [@isabelleviraldo](https://github.com/isabelleviraldo)

Contact: isabelleviraldo@gmail.com

Date Last Updated: 08/18/2023

## Table of Contents

- [Intro](#intro)
- [Running rgb_points](#running-rgb_points)
  - [Requirements](#requirements)
  - [Colcon Build](#colcon-build)
  - [Dockerfile Build](#dockerfile-build) 
- [Nodes In Depth](#nodes-in-depth)
  - [initial_attempt](#initial_attempt)
  - [fixed_math](#fixed_math)
  - [color_objects_using_center](#color_objects_using_center)
  - [few_colors](#few_colors)
- [TODO](#todo)

## Intro

## Running rgb_points

### Requirements

The following requirements must be met before any node within rgb_points can be run:

1. Running Lidar and Zed2i Camera or equivilant (rosbag file here) within ros2
2. Running [@hazelrah2](https://github.com/hazelrah2)'s AHC file ([Dockerfile](https://github.com/isabelleviraldo/rgb_points/blob/main/docker/AHC_dockerfile/Dockerfile))

### Colcon Build

If all dependancies are installed (found here), then you can clone this github repository as rgb_points, and store it in your ros2_ws/src folder, then do the following:

Go to where that clone is stored and source install/setup.bash

```sh
$ cd ~/ros2_ws/src
$ source install/setup.bash
```

Colcon build rgb_points

```sh
$ colcon build --packages-select rgb_points
```

Then you can run any node by doing the following for the one you want:

```sh
$ ros2 run rgb_points cp_v1
```
```sh
$ ros2 run rgb_points cp_new_math
```
```sh
$ ros2 run rgb_points cp_center
```
```sh
$ ros2 run rgb_points cp_few_colors
```

When viewing rgb_points' output in rviz2, use [this configuration](https://github.com/isabelleviraldo/rgb_points/blob/main/rviz2/rgb_points_config.rviz)

### Dockerfile Build

To run rgb_points, you can also build 
[this Dockerfile](https://github.com/isabelleviraldo/rgb_points/blob/main/docker/rgb_points_dockerfile/Dockerfile)
 on a device with an NVIDIA card

This Dockerfile will setup an environment for rgb_points to run in by building off of a ros humble base in ubuntu 22.04, and install all of the dependancies nessesary

As always, to build a Dockerfile you want to put it in it's own folder, and cd to that folder, after that, just do:

```sh
$ docker build -t <container_name> .
```

Once Dockerfile is built, each new terminal must do:

```sh
$ source /opt/ros2_ws/install/setup.bash
```

To open a new terminal on any Docker container do:

```sh
$ docker exec -it <container_name> bash
```

To enter the [internal README](https://github.com/isabelleviraldo/rgb_points/blob/main/docker/README_docker.txt) do:

```sh
$ cat README.txt
```

Then you can run any node by doing the following for the one you want:

```sh
$ ros2 run rgb_points cp_v1
```
```sh
$ ros2 run rgb_points cp_new_math
```
```sh
$ ros2 run rgb_points cp_center
```
```sh
$ ros2 run rgb_points cp_few_colors
```

## Nodes In Depth

[@hazelrah2](https://github.com/hazelrah2)'s AHC file MUST be running for these to work.

For all of the following, the inputs and outputs are as follows:

Inputs: 
- /lidar_0/AHC/clusters (Point Cloud)
- /zed2i/zed_node/left/image_rect_color (Image)

Output: 
- /rgb_points/testing (Point Cloud)

### initial_attempt

Once your environment is set up, to run this node do:

```sh
$ ros2 run rgb_points cp_v1
```

This node will take the points from the point cloud published from the lidar, and uses each point as a reference to find the color associated with it from the zed camera using the point's location and comparing its location in space to its location relative to the camera.

This was the initial attempt to color the points, the math written is *very* inaccurate for a few reasons:

1. The arctan function outputs radians, and I didn't realise that, thinking it output degrees
2. I instead put in my degrees as decimal points (125 as 1.25)
3. While debugging I divided something by 2, which really shouldnt be halved, but left it
4. There are 2 lines without any parenthesis when multiplying and dividing

However, dispite these problems fundamentally wrong with the math, I have it as a part of the final upload because it is somehow *more* accurate than the later iterations. I do not understand how, but this is why I decided that it is important enough to keep here.

Details about what specifically goes on within the code can be found within the comments on [the file](https://github.com/isabelleviraldo/rgb_points/blob/main/rgb_points/initial_attempt.py)

### fixed_math

Once your environment is set up, to run this node do:

```sh
$ ros2 run rgb_points cp_new_math
```



Details about what specifically goes on within the code can be found within the comments on [the file](https://github.com/isabelleviraldo/rgb_points/blob/main/rgb_points/fixed_math.py)

### color_objects_using_center

Once your environment is set up, to run this node do:

```sh
$ ros2 run rgb_points cp_center
```

Details about what specifically goes on within the code can be found within the comments on [the file](https://github.com/isabelleviraldo/rgb_points/blob/main/rgb_points/color_objects_using_center.py)

### few_colors

Once your environment is set up, to run this node do:

```sh
$ ros2 run rgb_points cp_few_colors
```

Details about what specifically goes on within the code can be found within the comments on [the file](https://github.com/isabelleviraldo/rgb_points/blob/main/rgb_points/few_colors.py)

## Todo
