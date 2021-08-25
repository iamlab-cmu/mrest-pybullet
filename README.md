# Ballbot PyBullet Sim

This package uses PyBullte to simulate the CMU ballbot. This package is currently experimental and is not guarentee to work. The simulation provides a python only interface and a ROS interface (recommended). 

## Getting Started

### Requirements
 - Ubuntu 16.04/18.04
 - PyBullet 
 - ROS Kinetic/Melodic (if using ROS interface)
 
### Install PyBullet 
This package has been tested to work with Ubuntu 16.04.

1. Upgrade python version from Py3.5 to 3.8. You can follow [this instructions](https://medium.com/analytics-vidhya/installing-python-3-8-3-66701d3db134)
2. Install PIP manually using 
    ```
    sudo apt install python3.8-distutils

    wget https://bootstrap.pypa.io/get-pip.py
    sudo python3 get-pip.py
    ```
3. Pybullet can be installed using
    ```
    pip install pybullet
    ```
4. Install ros in virtualenv
    ```
    pip install rosinstall
    ```

## Installing Package
The package has been setup to be a ROS package. Thus, DO NOT MANUALLY INVOKE setup.py, USE CATKIN INSTEAD as follows

```
$ catkin build ballbot_pybullet_sim
```
This will install the python modules in `/src': 
  - controllers 

Then you can source your catkin workspace `devel` folder as usual.

## Setting up ROS interface
This requires a working ROS Kinetic/Melodic workspace which contains the ballbot packages.
ROS workspace must contain the following packages:
- [ballbot_arm_description](http://clarinet.msl.ri.cmu.edu:9999/rshum/ballbot_arm_description)
- [ballbot_arm_ros/ballbot_arm_msgs](http://clarinet.msl.ri.cmu.edu:9999/rshum/ballbot_arm_ros/tree/master/ballbot_arm_msgs)
- [rt_msgs](http://clarinet.msl.ri.cmu.edu:9999/Ballbot/rt_msgs)

If other packages in ballbot_arm_ros do not compile, you can flag them as ignore using these commands 
```
touch src/ballbot_arm_ros/ballbot_arm_controllers/CATKIN_IGNORE
touch src/ballbot_arm_ros/ballbot_arm_visualization/CATKIN_IGNORE
catkin build
```
1. (Optional but highly recommended) Create python3 virtual environment (e.g. using [venv](https://docs.python.org/3/library/venv.html))  
    ```python3 -m venv /path/to/new/virtual/environment``` 
2. (Optional) activate venv  
    ```source /path/to/venv/bin/activate```
3. (With activated venv), Install required python modules  
    ```pip install pybullet
    pip install numpy pyyaml rospkg```
4. Source ROS ballbot workspace  
    ```source /path/to/ballbot_ws/devel/setup.bash```
5. Run ballbot simulation  
    In a seperate terminal, launch ROS master:  
    `roscore`  
    Launch simulation:  
    `python /ballbot_pybullet_sim/scripts/ballbot_sim.py`
