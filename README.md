# RoboticsProject: Human Activity Recognition and Wireless Real-time RGBD-SLAM

Our project is in two parts:
* isr_activity_recognition - human activity recognition
* rgbdslam_kinetic - wireless real-time rgbdslam based on Kinect

While the two packages do not overlap the aim in the future would be to merge them to be able to control map building with human gestures.
The two packages are described below. 

Section 1 introduces human activity recognition package.  Section 2 introduces wireless real time rgbdslam. Section 3 is the Conclusion and Future Work. Section 4 is the Reference.


## 1. isr_activity_recognition - human activity recognition

Package isr_activity_recognition by Mario Vieira et al.: https://github.com/mario-vieira/isr_activity_recognition.git

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

The isr_activity_recognition was originally implemented and tested using ROS Hydro running on Ubuntu 12.04. We used it successfully in ROS Indigo.

The instructions below will help you get the package up and running on your machine.

### Prerequisites

* The NITE library must be manually installed for `openni_tracker` to function. 

* The scikit-learn must be installed for `classifica_w.py` to work. 

* The `sound_play` node must be installed for text-to-speech. 

* The `pocketsphinx` package must be installed for speech recognition. 

### About the package

* **openni_launch:** This package contains launch files for using OpenNI-compliant devices such as the Microsoft Kinect in ROS.

* **openni_tracker:** The OpenNI tracker broadcasts the OpenNI skeleton frames using `tf`.

* **learning_image_geometry:** This package projects the `tf` frames of the skeleton acquired by the `openni_tracker` onto an image.

* **learninf_tf:** This package uses the `tf_listener` node to get the coordinates of the skeleton being tracked relative to the torso and camera, and the `classifica_w.py` node to recognize the activity being performed.

* **pi_speech_tutorial:** This package contains launch files for speech recognition.

* **random_navigation_goals:** This package is responsible for robot navigation. The `simple_navigation_goals` node makes the robot randomly navigate the environment. The `follower_speed` node makes the robot follow a person, using velocity commands. The `follower` node makes the robot follow a person if it hear "follow me", avoiding collision with the human. Finally, the `follower_speed` node does the same as the `follower` node, sending velocity commands instead. 

* **isr_activity_recognition:** This is the main package. It launches all necessary files for activity recognition. 

### Installation

* Download and make `openni_launch` and `openni_tracker` packages on the system of the robot, as these need direct access to the Kinect.

* Make the `isr_activity_recognition` on the workstation.

* Once all the packages have been downloaded and installed, place the file `isr_activity_recognition/isr_activity_rec.launch` on the TurtleBot system(e.g. in `/home`). This is necessary to separate the launch of `openni_launch` from `openni_tracker` as combining them results in errors.

### Running the package

To start the program run the following on the TurtleBot:

`roslaunch isr_activity_rec.launch`
This will launch `openni` and sound packages.

Launching openni_tracker node:
`rosrun openni_tracker openni_tracker`

Launch the workstation:

`roslaunch isr_activity_recognition activity_recognition.launch`

### Challenges Faced
During the implementation of this part of the project we faced the following challenges:

* Unreliable voice recognition. The voice recognition package running on the TurtleBot does not capture speech well, probably due to distance from the human. The goal of the package is to enable interactive communication between the human and the robot. Without reliable voice recognition this aim is undermined. We were able to create a temporary workaround, by using joystick for input.

* Human skeleton detection failure (`openni_tracker`). `openni_tracker` fails to calibrate a human skeleton when the Kinect is far from the subject. 

* Problem with navigation module. The navigation module of the package is designed to enable the robot to follow the human. This module does not perform as expected. This means, unfortunately, that the navigation task cannot be accomplished combined with the rest of the program. 


### Demo example

In the demo video, we see the human(Peixi) in one window and the classified activities in a command window.

https://www.youtube.com/watch?v=AFvDDGZ19P4


## 2. rgbdslam_kinetic - wireless real-time rgbdslam based on Kinect

### Introduction

The work was developed for wireless real-time rgbdslam based on Kinect. It uses an xbox 360 kinect camera to get the image sequence with rgb information and depth information. With the help of such image information and tf information the algorithm extracts features, compare the features between different frames, find the correspondence and transformations, and reconstructs the 3D environment with the information. The 3D reconstruction method does not need other sensors, such as the laser sensor, which makes the system much more convenient and cheaper.  

The idea is inspired from such papers:

1)Real-time 3D visual SLAM with a hand-held RGB-D camera (Engelhard et al.)

2)Using the Kinect as a Navigation Sensor for Mobile Robotics (Oliver et al.)

3)An Evaluation of the RGB-D SLAM System (Endres et al.)

4)Appearance-Based Loop Closure Detection for Online Large-Scale and Long-Term Operation (Labbe et al.)



However, many 3-D reconstruction methods based on Kinect cannot work very well in real time processing with wireless wifi. So in this project we propose some methods to deal with real-time wireless processing issues. With these methods, we wrote some launch files, which are introduced in the subsection "About the launch file for kinect".  

### Prerequisites

The conditions of running this package:

1) Work station with Ubuntu system with ROS indigo

2) A turtle robot with xbox 360 kinect camera and a laptop on the robot

3) The wireless wifi connection between the robot and the work station 

4) Make sure that you have openni_launch, freenect_launch, or other proper packages to control the kinect and get the image data.


### About the package

'openni_launch': This package contains launch files for using OpenNI-compliant devices such as the Microsoft Kinect in ROS.

freenect_launch: Similar with openni_launch.

rgbdslam: The package to get image information and other information, process such information and reconstruct the 3D environment.

rtabmap_ros: Similar with rgbdslam package.

### Installing

This section introduces how to install the packages.

1) Download the package and extract it into src folder of catkin_ws folder.

2) Go to catkin_ws directory and input command "catkin_make".

3) Input command "rospack profile"

The rtabmap_ros package is also recommended to install:

sudo apt-get install ros-indigo-rtabmap-ros


### About the launch file for kinect

In the launch folder, there are lots of launch files.
You could run openni_launch in the laptop, move the robot and launch the rgbdslam.launch in the work station.
But you may not run it properly, because these launch files are not designed for wireless real-time processing. 


The wireless internet speed is very slow, so we cannot transfer too much information from laptop on the robot to workstation through wireless internet connection. Of course, with such low data transfer speed, we cannot make it work in real-time. There are some ideas to make it work in real-time and wireless process: 1)reduce the frame rate  2)reduce the image resolution  3)compress the information.

So it is recommended to use this launch file written by us, to make sure it works in real-time with wireless wifi:

```

<!-- This file demonstrates the use of SIFT features for online SLAM with a Kinect. 
     The openni driver is started from this file -->
<launch>
   <param name="/camera/driver/data_skip" value="10" /><!--reduce the kinect frame rate-->

  <include file="$(find openni_launch)/launch/openni.launch"/>
  <node name="$(anon dynparam)" pkg="dynamic_reconfigure" type="dynparam" args="set_from_parameters /camera/driver" clear_params="true">
     
    <param name="image_mode" type="int" value="5" /><!--reduce the resolution-->
    <param name="depth_mode" type="int" value="5" />
  </node>
</launch>

```

This launch file is launched in the laptop on the robot. While the "rgbdslam.launch" is launched in work station.  



### Running the demo of rgbdslam package

This section briefly introduces how to run the demo of rgbdslam package:

1) In the work station: "gedit ~/.bashrc" to uncomment 

2) ssh to the turtle robot to make it be ready to move: 

Move it with "turtlebot_bringup minimal.launch" and "turtlebot_teleop keyboard_teleop.launch"

or move it with joystick


3) From workstation, ssh to the turtle robot again and launch the kinect with the launch file written by us, which is displayed in previous section ("About the launch file for kinect"):

just copy that launch file to the laptop on the robot and run it with ssh from work station:

roslaunch qvga-kinect.launch


You could check if kinect works well by opening another terminal in work station and input such command line:

rosrun image_view image_view image:=/camera/rgb/image_color


4) then input this command in another terminal of work station:

roslaunch rgbdslam rgbdslam.launch

Then you can observe the result.



### Record the rosbag

If the user wants to record the rosbag, such topics should be recorded:

/tf

/camera/rgb/image_mono OR /camera/rgb/image_color 

/camera/rgb/camera_info

/camera/depth/image

/camera/depth/camera_info


Do not record more topics. The rosbag would be very big.



### Running the demo of rtabmap_ros package

The rtabmap_ros is similar with rgbdslam package. You could test it with such steps:


1) In the work station : "gedit ~/.bashrc" to uncomment the last three lines

2) ssh to the turtle robot to make it be ready to move: 

Move it with "turtlebot_bringup minimal.launch" and "turtlebot_teleop keyboard_teleop.launch"

or move it with joystick


3) From workstation, ssh to the turtle robot again and launch the kinect with freenect_launch with compression information:


roslaunch freenect_launch freenect.launch depth_registration:=true compressed:=true

In the work station, open another terminal:

check if the kinect work:

rosrun image_view image_view image:=/camera/rgb/image_color


4) then run rtabmap_ros package in the work station:

roslaunch rtabmap_ros rgbd_mapping.launch rtabmap_args:="--delete_db_on_start" compressed:=true

Then the demo is displayed


### Demo example

An example of the demo (Youtube video).


https://www.youtube.com/watch?v=7kmf-Sb3PuM&t=2s


From the video you can see that there is an interface. The bottom left corner of the interface shows the Kinect camera frames and the 
feature detection. The right part of the interface shows the reconstruction of the 3D environment.

It can be seen that with the robot moving slowly, the 3d environment is slowly built.

In 00:56 it can be seen that the robot turns right and goes back, then aisle and short walls on the right side are reconstructed within the 3D environment. 


           


## 3. Conclusion and Future Work

In this report, we proposed two works, human activity recognition and wireless real-time rgbdslam. The prerequisites, installation, implementation and other technical details of the packages are explained in the report. The demos are shown with Youtube videos. Our idea was to merge our two works so that we could implement controlling map building with human gestures (hand gesture, for instance). That could be a potential continuation of this work.




## 4. References
1. Package isr_activity_recognition by Mario Vieira: https://github.com/mario-vieira/isr_activity_recognition.git

  1a. Mario Vieira, Diego R. Faria, Urbano Nunes, Real-time Aplication for Monitoring Human Daily Activities and Risk Situations in Robot-assisted Living, 2015
  
  1b. Diego R. Faria, Mário Vieira, Cristiano Premebida, Urbano Nunes, Probabilistic Human Daily Activity Recognition towards Robot-assisted Living, 2015
  
2. Package rgbdslam by Felix Endresa: https://github.com/felixendres/rgbdslam_v2

   Package rtabmap_ros by Mathieu Labbe and Franc¸ois Michaud: https://github.com/introlab/rtabmap_ros


  2a. Engelhard, N., Endres, F., Hess, J., Sturm, J., & Burgard, W. (2011, April). Real-time 3D visual SLAM with a hand-held RGB-D camera. In Proc. of the RGB-D Workshop on 3D Perception in Robotics at the European Robotics Forum, Vasteras, Sweden (Vol. 180).
  
  2b. Endres, F., Hess, J., Engelhard, N., Sturm, J., Cremers, D., & Burgard, W. (2012, May). An evaluation of the RGB-D SLAM system. In Robotics and Automation (ICRA), 2012 IEEE International Conference on (pp. 1691-1696). IEEE.
  
  2c. Labbe, M., & Michaud, F. (2013). Appearance-based loop closure detection for online large-scale and long-term operation. IEEE Transactions on Robotics, 29(3), 734-745.
  
  2d. Oliver, A., Kang, S., Wünsche, B. C., & MacDonald, B. (2012, November). Using the Kinect as a navigation sensor for mobile robotics. In Proceedings of the 27th Conference on Image and Vision Computing New Zealand (pp. 509-514). ACM.
  
  
