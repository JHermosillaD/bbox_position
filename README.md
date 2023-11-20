# Pedestrain position estimator
![Static Badge](https://img.shields.io/badge/ros%20-%20noetic%20-blue) ![GitHub repo size](https://img.shields.io/github/repo-size/jhermosillad/bbox_position)
 ![GitHub top language](https://img.shields.io/github/languages/top/jhermosillad/bbox_position) 

ROS package to estimate pedestrian position from bounding box coordinates. The package subscribes to the _(i,j)_ pixel coordinates of a bounding box and its respective point cloud and generates an _(x,y,z)_ position from the base_link reference frame. The coordinates are published in real time through different topics.
## Set-up
### Requirements
- [x] ROS melodic or higher
- [x] OpenCV
- [x] Published bounding box coordinates [[1]](https://github.com/JHermosillaD/hog/tree/main),[[2]](https://github.com/JHermosillaD/haar_cascade)
### Installation
Clone the repository to the workspace source path.
```
user@hostname:~/workspace/src$ git clone https://github.com/JHermosillaD/bbox_position.git
```
Compile the package.
```
user@hostname:~/workspace$ catkin_make
```
## Usage
Edit file `launch/position_estimation.launch` by the appropriate value of parameter `/bbox_topic` according to the name of the used bounding box topic :bangbang:

Run the launcher.
```
user@hostname:~/workspace$ roslaunch bbox_position position_estimation.launch
```
## Visualization
The reference frame can be displayed in Rviz, the coordinates through topic `/pedestrian/position"`.

<img width="605" height="360" src="/pos-min.png"> 
