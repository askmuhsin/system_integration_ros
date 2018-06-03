This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car.      

![Carla](https://github.com/askmuhsin/system_integration_ros/blob/master/imgs/carla.jpg)

---
## Udacity self-driving Car Capstone Project : System Integration
This project is done as team effort and marks the end of a nine month intensive effort on getting a **real self-driving car** to navigate autonomously, utilising several onboard sensors (cameras, lidars, radars), sensor fusion and path planning algorithms, machine learning modules, and a lot of hard work. Special thanks to my team members!     

## Project Team Members of "No-LEFT-TURN"
|Name              |Udacity Account Email Address|
|------------------|-----------------------------|
|Sahil Bahl*       |bahlsahil28@gmail.com        |
|Akiyuki Ishikawa  |aki.y.ishikawa@gmail.com     |
|Muhsin Mohammed   |askmuhsin@gmail.com          |
|Wei-Fen Lin       |weifen@mijotech.net          |       

`*Team lead`

---
### Recommended Setup and Usage
* Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

* Install Docker
[Install Docker](https://docs.docker.com/engine/installation/)

* Build the docker container
```bash
docker build . -t capstone
```

* Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

* Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
OR
./run.sh
```
* Run the simulator     
Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

**For instructions on native installation click [here](https://github.com/askmuhsin/system_integration_ros/blob/master/imgs/additional_info.md).**

---
### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

---
### Discussion
Core autonomous vehicle features of the project is written as ROS nodes which is included but not limited to: traffic light detection, control, and waypoint following from stored map. The ROS nodes and topics for this project are as follows:   
![ros_graph](https://github.com/askmuhsin/system_integration_ros/blob/master/imgs/final-project-ros-graph-v2.png)   

#### Implementation HighLight
##### Waypoint Updater
  The walkthrough section gives a detailed implementation on the first part of waypoint updater. The eventual purpose of this node is to publish a fixed number of waypoints ahead of the vehicle with the correct target velocities, depending on traffic lights and obstacles. The goal for the first version of the node should be simply to subscribe to the topics.
-   `/base_waypoints`
-   `/current_pose`

and publish a list of waypoints to

-   `/final_waypoints`

Once the implementation is done, we can run the simulation and see a line of green dots shown on the simulator, which depicts the generated waypoints.ß
I changed LOOKAHEAD_WPS from to 50 in the final submission to reduce the computation overhead. During debugging,

##### Twist Controller
  Within the twist controller package, two modules are required to be implmented, dbw_node.py and twist_controller.py. In dbw_node.py, we need to handle ROS subscribers for  the /current_velocity, /twist_cmd, and /vehicle/dbw_enabled topics and publish thottle, steering and brake signals. During debugging, it might help to reduce the publish rate and see the change in the simulation. However, the DBW system on Carla expects messages at 50Hz, and will disengage (reverting control back to the driver) if control messages are published at less than 10hz. This is a safety feature on the car intended to return control to the driver if the software system crashes.  Therefore, we need to make sure the simulation still works at 50Hz in the final implementation.

##### Traffic Light Detection
  The tasks for this package were broken into two parts. In the first part, we need to implement the tl_detector.py module. The walkthrough section gives enough details to implement this module. What is not mentioned in the walkthrough code is the second part, to build a traffic light classifier. Most people used the tensorflow object dection API for this project. There is a very good reference from Alex Lechner at https://github.com/alex-lechner/Traffic-Light-Classification. It gives a detailed tutorial on how to build a traffic light classifier in this project. I followed the same methodoligy to test a couple of pre-trained models in the tensowflow library.
  I end up using the SSD Inception V2 model for this project. Two seperate models are trained for simulator and real-world testing. Both models were trained for 20,000 steps.

---
### Results   
Video link of full lap ride in highway -->  [link_highway](https://youtu.be/L35UgI55J_k)       
Video link of full lap ride in udacity lot -->  [link_lot](https://youtu.be/GNqRoENU62I)       
