# heightcameraestimation
Estimate the heigh of the depth camera: Kinect Asus..etc


Quick Start:

To use this package clone the repository to your disc and build it:

cd catkin_ws/src 

git clone https://github.com/JaouadROS/heightcameraestimation

cd ..

catkin_make

source devel/setup.bash

Open a new shell:
Start the Openni driver or freenect driver:

roslaunch freenect_launch freenect.launch

Open a new shell and run subKinect node:
rosrun heightcameraestimation subKinect 
