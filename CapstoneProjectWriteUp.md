# CarND-Capstone System Integration Project
### Abdelkoddous Khamsi

[image1]:./writeup_resources/ArchitectureDiagram.png "Project architecture diagram" 
[image2]:./writeup_resources/green_detection.JPG "green TL detection" 
[image3]:./writeup_resources/red_detection.JPG "red TL detection" 
[gif1]:./writeup_resources/capstone_demo.gif "red TL detection" 

The goal of this final project is to implement various core functionalities in the three main layers of a self driving car: Preception, planning and Control.

## 1.Architecture Diagram:



This project uses the Robot Operating System ([ROS](https://www.ros.org/about-ros/)) framework under Python. 
Each layer has a couple of nodes that implements a specific functionnality as shown in the Architecture diagram below:

<img src="./writeup_resources/ArchitectureDiagram.png" alt="Architecture Diagram"
	title="Architecture Diagram" width="800"  />

Nodes exchange data using a publish/subscribe policy: A node that is subscribed to a given topic is notified whenever some other node has published data to this topic.

## 2.Walkthrough sessions
These videos sessions were of precious help to get me going with the source code of the project:
* **Waypoint Updater 1st part**: Used the input from /base_waypoints (total waypoints of the track) and /current_pose topics to generate /final_waypoints (a fixed number of waypoints that are ahead of the current position of the car)
* **DBW Node**: Generated steering, brake and throttle commands for the car using a PID controller and the /twist_cmd input.
* **Detection**: Processing traffic lights with regards to the current position of the car. Initially no classification model was used to predict the state of the Traffic lights, but used the state provided by the simulator.
* **Waypoint Updater 2nd part:** Handle input from detection to readjust velocities of waypoints accordingly. For example: if a RED traffic light was detected, then the velocity of the waypoint at this TL stop line should be zero, Therefore the car will plan its route to satisfying this condition by decreasing gradually its speed.


## 3.Tensorflow Object detection API

* At this point Traffic Light Detection node was not relying on a classification model but only the actual internal state of the TL that provided by the simulator. Therefore I had to train classification model for this task. 
For this I went trough the follwing steps:
    * Captured and saved training, validation and test images from the simulator.
    * Used LabelImg software to label these images.
    * Used TF Object detection API to train my model. You can find the jupyter notebook where I trained my model using transfer learning as I started from the ssd_mobilenet_v2 as a starting point model.
    * Converted my model checkpoints to a Saved_model under .pb format for easier deployement

Here are sample results I got ater I tested my model on new images from the simulator:


green TL detection     |  red TL detection
:-------------------------:|:-------------------------:
![alt text][image2]       |  ![alt text][image3]



Notebook for my model training and graph exporting is available [here](TL_Classification_TF_ObjectDetectionAPI.html).


**Important note**:
The model I trained was developed for TF2, but as Udacity workspace has only TF=1.3 you'd need to run the followoing commands to upgrade to TF2:

`>> python -m pip install --upgrade pip`

`>> pip install --upgrade tensorflow==2`


## 4.System integration in action

![alt text][gif1]  

The full video can be found [here](./writeup_resources/final_run_accelerated.mp4).