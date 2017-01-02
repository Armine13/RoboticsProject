# RoboticsProject

Our project is in two parts:
* isr_activity_recognition - human activity recognition
* rgbdslam_kinetic - wireless real-time rgbdslam based on Kinect

While the two packages do not overlap the aim in the future would be to merge them to be able to control map building with human gestures.
The two packages are described below.

# isr_activity_recognition - human activity recognition

The package was developed to track and recognize human activities real-time. It uses an RGB-D sensor to track the human skeleton and extract features. The activities are classified with Dynamic Bayesian Mixture Model (DBMM), which is a combination of several classifiers to improve accuracy.

The idea behind the original package is to enable the robot to monitor the human activities and react accordingly. e.g. if the detected activity is falling, it will ask if help is needed or follow when the human says "follow me". In our project, however, we had some troubles with voice recognition, and instead the input is given through joystick buttons. 

**ACTIVITIES:** 

1. Walking
2. Standing still
3. Working on computer
4. Talking on the phone
5. Running
6. Jumping
7. Falling
8. Sitting down

The isr_activity_recognition was implemented and tested using ROS Hydro running on Ubuntu 12.04.


The instructions below will help you to 
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

The NITE library must be manually installed for openni_tracker to function.

The scikit-learn must be installed for classifica_w.py to work. 

The sound_play node must be installed for text-to-speech. 

The pocketsphinx package must be installed for speech recognition. 

### About the package

openni_launch: This package contains launch files for using OpenNI-compliant devices such as the Microsoft Kinect in ROS.

openni_tracker: The OpenNI tracker broadcasts the OpenNI skeleton frames using tf.

pi_speech_tutorial: This package contains lauch files for speech recognition.

learning_image_geometry: This package projects the tf frames of the skeleton acquired by the openni_tracker onto an image.

learninf_tf: This package uses the tf_listener node to get the coordinates of the skeleton being tracked relative to the torso and camera, and the classifica_w.py node to recognize the activity being performed.

random_navigation_goals: This package is responsible for robot navigation. The simple_navigation_goals node makes the robot randomly navigate the environment. The follower_speed node makes the robot follow a person, using velocity commands. The follower node makes the robot follow a person if it hear "follow me", avoiding collision with the human. Finally, the follower_speed node does the same as the follower node, sending velocity commands instead.

### Installing

## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```
                

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.








# rgbdslam_kinetic - wireless real-time rgbdslam based on Kinect

### Introduction** 

The package was developed for wireless real-time rgbdslam based on Kinect. It uses an xbox 360 kinect camera to get the image sequence with rgb information and depth information. With the help of these image information and tf information the algorithm extracts features and compare the features between different frames, and then reconstructs the 3D environmemt with these image frames. The 3D reconstruction method does not need other sensor, such as laser sensor, which makes the system much more convenient and cheaper.  

The idea is inspired from such papers:

1)Real-time 3D visual SLAM with a hand-held RGB-D camera (Engelhard et al.)

2)Using the Kinect as a Navigation Sensor for Mobile Robotics (Oliver et al.)

3)An Evaluation of the RGB-D SLAM System (Endres et al.)


However, many 3-D reconstruction methods based on Kinect cannot work very well in real time processing with wireless wifi. So in this project we propose some methods to deal with real-time wireless processing issue. With these methods, we wrote some luanch files, which are also uploaded in the package.  

### Prerequisites

The conditions of running this package:

1) Work station with Ubuntu system with ROS indigo

2) A turtle robot with xbox 360 kinect camera and a laptop on the robot

3) The wireless wifi connection between the robot and work station 

4) Make sure that you have openni_launch, freenect_launch, or other proper packages to control the kinect and get image data.


### About the package

openni_launch: This package contains launch files for using OpenNI-compliant devices such as the Microsoft Kinect in ROS.

freenect_launch: Similar with openni_launch.

rgbdslam: The package to get image information and other information, process such information and reconstruct the 3D environment.

rtabmap_ros: Similar with rgbdslam package.

### Installing

This section introduces how to install the packages.

1) Download the package and extract it into src folder of catkin_ws folder.

2) Go to catkin_ws directory and input command "catkin_make".

3) Input command line "rospack profile"

The rtabmap_ros package is also recommended:

sudo apt-get install ros-indigo-rtabmap-ros


## About the launch file for kinect

In the launch folder, there are lots of launch files.
You could run openni_launch in the laptop, move the robot and launch the rgbdslam.launch in the work station.
But you may not run it properly, because these launch files are not designed for wireless real-time processing. 


The wireless internet speed is very slow, so we cannot send too much information from laptop on the robot to workstation through wireless internet connection. Of course, with such low data transfer speed, we cannot make it run in real-time. There are some ideas to make it work in real-time and wireless process: reduce the frame rate, reduce the image resolution and compress the information.

So here it is recommended to use this launch file written by us:

```

<!-- This file demonstrates the use of SIFT features for online SLAM with a Kinect. 
     The openni driver is started from this file -->
<launch>
   <param name="/camera/driver/data_skip" value="10" /><!--reduce the kinect frame rate-->

  <include file="$(find openni_launch)/launch/openni.launch"/>
  <node name="$(anon dynparam)" pkg="dynamic_reconfigure" type="dynparam" args="set_from_parameters /camera/driver" clear_params="true">
     
    <param name="image_mode" type="int" value="5" />
    <param name="depth_mode" type="int" value="5" />
  </node>
</launch>

```








## Running the tests

Explain how to run the automated tests for this system

### Break down into end to end tests

Explain what these tests test and why

```
Give an example
```

### And coding style tests

Explain what these tests test and why

```
Give an example
```
                

## Authors

* **Billie Thompson** - *Initial work* - [PurpleBooth](https://github.com/PurpleBooth)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.





## References
1. Package isr_activity_recognition by Mario Vieira: https://github.com/mario-vieira/isr_activity_recognition.git
  1a. Mario Vieira, Diego R. Faria, Urbano Nunes, Real-time Aplication for Monitoring Human Daily Activities and Risk Situations in Robot-assisted Living, 2015
  1b. Diego R. Faria, Mário Vieira, Cristiano Premebida, Urbano Nunes, Probabilistic Human Daily Activity Recognition towards Robot-assisted Living, 2015
