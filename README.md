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
Please use **one** of the two installation options, either native **or** docker installation.

### Recommended Usage
Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
OR
./run.sh
```
4. Run the simulator    

**For instructions on native installation click [here]().


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
