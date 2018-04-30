# Camera calibration and visual odometry via optical flow tracking
Course work for CIVILEN 6451

Prerequisites:
---
1. Matlab (successfully run on OpenCV2.4.13)
2. Only the main code of vo is published

Usage:
---
1. download
2. cmake and run
3. see the main.cpp for interface examples

Performance:
---
1. camera calibration
(1) input chess borad image
![image](https://github.com/xiaohulugo/images/blob/master/calib2.png)
(2) detected corners
![image](https://github.com/xiaohulugo/images/blob/master/calib1.png)
(3) draw the 3d coordinate onto the image
![image](https://github.com/xiaohulugo/images/blob/master/calib3.png)

2. visual odometry
(1) dataset: TUM Visin Group, fr3/long_office_household dataset in the following link
https://vision.in.tum.de/data/datasets/rgbd-dataset/download
(2) results: 700 frames in 10 minutes
![image](https://github.com/xiaohulugo/images/blob/master/vo1.png)

Feel free to correct my code, if you spotted the mistakes. You are also welcomed to Email me: fangzelu@gmail.com
