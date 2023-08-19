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
  - [few_colors](#few_colors)
  - [color_objects_using_center](#color_objects_using_center)
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
$ ros2 run rgb_points cp_few_colors
```
```sh
$ ros2 run rgb_points cp_center
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
$ ros2 run rgb_points cp_few_colors
```
```sh
$ ros2 run rgb_points cp_center
```

When viewing rgb_points' output in rviz2, use [this configuration](https://github.com/isabelleviraldo/rgb_points/blob/main/rviz2/rgb_points_config.rviz)

## Nodes In Depth

[@hazelrah2](https://github.com/hazelrah2)'s AHC file MUST be running for these to work.

For all of the following, the inputs and outputs are as follows:

Inputs: 
- /lidar_0/AHC/clusters (Point Cloud) (x, y, z, label)
- /zed2i/zed_node/left/image_rect_color (Image) (b, g, r, a)

Output: 
- /rgb_points/testing (Point Cloud) (x, y, z, rgb, label)

### initial_attempt

Once your environment is set up, to run this node do:

```sh
$ ros2 run rgb_points cp_v1
```

When viewing rgb_points' output in rviz2, use [this configuration](https://github.com/isabelleviraldo/rgb_points/blob/main/rviz2/rgb_points_config.rviz)

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

When viewing rgb_points' output in rviz2, use [this configuration](https://github.com/isabelleviraldo/rgb_points/blob/main/rviz2/rgb_points_config.rviz)

This was made to make the intended math actually run correctly as intended. All later nodes use this version of the math.

Conseptually how the math works is by taking advantage of the camera's known fov, the lidar's known fov, the returned image's width and height, and if we know where a point is in space, then by taking its angle relative to the lens vertically and horizontally, we can find the pixel that is located at that angle horizontally and vertically. 

The math as I have it coded within the function pixelpick() is as follows:

1. Find angle between object and lens horizontally and vertically
      - (use x, y, z of point and take arctan of width and height compared to distance)
2. Find how far left or right/up or down the object is
      - (taking a ratio of left/right angle by zed camera width fov, same with up/down angle and height fov)
3. Find how many pixels from the center of the image that is
      - (multiplying that ratio by half width or height of the image)
4. Convert those numbers to be in reference of the top left of the image
      - (subtract the distance from center from the center point)

Now that it is the actual location, getting the rgb at that pixel is really simple. 

To note, you want to frequently check that you never accedentally call for a pixel that isn't there, so I have a lot of lines where it will return a bright color if it is out of bounds.

Additionally, always make sure that it is cast as an int before attempting to find the pixel in the array.

Like said before in initial_attempt, this math produces a less accurate projection, it could be due to some coding oversight, or something inherently wrong with the consept. Some investivation is needed to determine why, because I could not figure it out myself, but it might remain a mystery.

All later nodes use this math, because it actually completes the code as intended, even with the higher quality output by the initial attempt.

Details about what specifically goes on within the code can be found within the comments on [the file](https://github.com/isabelleviraldo/rgb_points/blob/main/rgb_points/fixed_math.py)

### few_colors

Once your environment is set up, to run this node do:

```sh
$ ros2 run rgb_points cp_few_colors
```

When viewing rgb_points' output in rviz2, use [this configuration](https://github.com/isabelleviraldo/rgb_points/blob/main/rviz2/rgb_points_config.rviz)

This node uses the math as explained in [fixed_math](#fixed_math)

This was made to explore the idea of reducing the number of times the code would run through the math while still ensuring it collected a useful amount of colors. 

What was decided here was to instead of coloring each pixel individually, by taking labels published by the AHC node made by [@hazelrah2](https://github.com/hazelrah2) and only pick a new color every 30 points within an object. While also reducing the number of times running through the math, the output would include the object information.

This has been shown to greatly increase the framerate of the program's output, as well as still color the object relevant colors. 

If a more accurate way of picking the colors can be found, I would recommend using this strategy, because it has many ways it could be expanded upon, adjusting how many points are colored at once based on the size of the object, if a color is "unnatural", if it is approaching an area of intrest, etc.

All of this however is done to try and compensate for the slow nature of the math done within my code, so if a fast, less computationally intensive 

Details about what specifically goes on within the code can be found within the comments on [the file](https://github.com/isabelleviraldo/rgb_points/blob/main/rgb_points/few_colors.py)

### color_objects_using_center

Once your environment is set up, to run this node do:

```sh
$ ros2 run rgb_points cp_center
```

When viewing rgb_points' output in rviz2, use [this configuration](https://github.com/isabelleviraldo/rgb_points/blob/main/rviz2/rgb_points_config.rviz)

This node uses the math as explained in [fixed_math](#fixed_math)

This was made to explore the idea of reducing the number of times the code would run through the math even further by only taking the centerpoint of the object's color.

This requires calculating the centerpoint of each object, which can be done by manually calculating the centerpoint of each object, as is done here. Alternatively, it could be done by subscribing to the topic being published by the AHC node made by [@hazelrah2](https://github.com/hazelrah2), 'lidar_0/AHC/centroids'











Details about what specifically goes on within the code can be found within the comments on [the file](https://github.com/isabelleviraldo/rgb_points/blob/main/rgb_points/color_objects_using_center.py)

## Todo
