Skip to content
This repository
Search
Pull requests
Issues
Gist
 @Armine13
 Unwatch 2
  Star 0
  Fork 0 Armine13/RoboticsProject
 Code  Issues 0  Pull requests 0  Projects 0  Wiki  Pulse  Graphs  Settings
RoboticsProject/ 
README.md
   or cancel
    
 Edit file    Preview changes
1
# RoboticsProject
2
​
3
Our project is in two parts:
4
* isr_activity_recognition - human activity recognition
5
* ...
6
While the two packages do not overlap the aim in the future would be to merge them to be able to control map building with human gestures.
7
The two packages are described below.
8
​
9
## isr_activity_recognition - human activity recognition
10
​
11
The package was developed to track and recognize human activities real-time. It uses an RGB-D sensor to track the human skeleton and extract features. The activities are classified with Dynamic Bayesian Mixture Model (DBMM), which is a combination of several classifiers to improve accuracy.
12
​
13
The idea behind the original package is to enable the robot to monitor the human activities and react accordingly. e.g. if the detected activity is falling, it will ask if help is needed or follow when the human says "follow me". In our project, however, we had some troubles with voice recognition, and instead the input is given through joystick buttons. 
14
​
15
**ACTIVITIES:** 
16
​
17
1. Walking
18
2. Standing still
19
3. Working on computer
20
4. Talking on the phone
21
5. Running
22
6. Jumping
23
7. Falling
24
8. Sitting down
25
​
26
The isr_activity_recognition was implemented and tested using ROS Hydro running on Ubuntu 12.04.
27
​
28
​
29
The instructions below will help you to 
30
These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.
31
​
32
### Prerequisites
33
​
34
The NITE library must be manually installed for openni_tracker to function.
35
​
36
The scikit-learn must be installed for classifica_w.py to work. 
37
​
38
The sound_play node must be installed for text-to-speech. 
39
​
40
The pocketsphinx package must be installed for speech recognition. 
41
​
42
### About the package
43
​
@Armine13
Commit changes

Update 

Add an optional extended description…
  Commit directly to the master branch.
  Create a new branch for this commit and start a pull request. Learn more about pull requests.
Commit changes  Cancel
Contact GitHub API Training Shop Blog About
© 2017 GitHub, Inc. Terms Privacy Security Status Help
