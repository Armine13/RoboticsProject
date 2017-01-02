# RoboticsProject

Our project is in two parts:
* isr_activity_recognition - human activity recognition
* ...
While the two packages do not overlap the aim in the future would be to merge them to be able to control map building with human gestures.
The two packages are described below.

## isr_activity_recognition - human activity recognition

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

A step by step series of examples that tell you have to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

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



## isr_activity_recognition - human activity recognition

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

A step by step series of examples that tell you have to get a development env running

Say what the step will be

```
Give the example
```

And repeat

```
until finished
```

End with an example of getting some data out of the system or using it for a little demo

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
