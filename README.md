# crop_height
CropHeight

(This is for catkin, if you are using legacy rosmake, visit https://github.com/owenqyzhang/CropHeight )

@brief Calculating distance to the ground/crop with RPLidar

Installation Guide

1. cd YOUR/catkin/WORKSPACE/src

2. clone this package with 'git clone'

3. cd ..

4. catkin_make

To run this package

1. start roscore

2. make sure mavros and RPLidar node are running

3. run 'rosrun crop_height crop_height'

The height to the ground is published in the rostopic crop_dist