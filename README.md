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

4. To record a video of the simulation you will need to install ffmpeg:
    ```
    sudo apt-get update
    sudo apt-get install ffmpeg
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

####


# How to install the pybullet simulation environment
The main issue with this package is that pybullet runs in Python3 by ROS only supports Python2. Thus, some special installation steps are required. 


NOTE: This installation instruction have been tested in Ubuntu 16.04 with ROS Kinetic. I assume they will be very similar for Ubuntu 18.04 and ROS Melodic. However, for Ubuntu 20.04 and ROS Noetic they may be different because it already has support for Python3.


1) Install your favorite version of Python3. These instructions where tested with Python3.7

2) Create a Python3 catkin workspace 
```
mkdir -p ~/Workspaces/ballbot_sim_py3_ws/src
cd ~/Workspaces/ballbot_sim_py3_ws
```

3) Make a Python3 virtual environment inside the workspace you just created
```
python3 -m venv ballbot_sim_venv
```

4) Activate the virtual environment
```
source ballbot_sim_venv/bin/activate
```

5) Install Python package dependencies
```
python -m pip install -U rosinstall catkin_tools numpy pybullet
```

6) RECOMMENDED: If you want to use the ROS interface then you will need to install `tf2 packages` to use Python3. 

```
cd ~/Workspaces/ballbot_sim_py3_ws/src
git clone https://github.com/ros/geometry2.git
cd geometry2/
git checkout 0.6.5

```

If you get the error XXX then you will need to install 
```
python -m pip install empy
```


7) Now you can build the workspace, note that you will need to modify the Python executable to the path in your machine
```
catkin build geometry2 ballbot_pybullet_sim --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/home/ballbot/Workspaces/ballbot_sim_ws/sim_venv/bin/python3 -DPYTHON_LIBRARY=/usr/lib/python3.7/config-3.7m-x86_64-linux-gnu/libpython3.7m.so -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m
```

8) Source your workspace environment and you should now be able to run the pybullet simulation with ROS Interface
```
source devel/setup.bash
roslaunch ballbot_pybullet_sim sim_empty_world.launch
```

## References
 - https://medium.com/@inderpreetsinghchhabra/using-python3-with-ros-kinetic-2488354efece
 - https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674
 - https://answers.ros.org/question/237613/how-to-define-ros-kinetic-to-use-python3-instead-of-python27

 ## Installing Python 3.7

```
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt update
sudo apt install python3.7 python3.7-dev python3.7-venv
```

## Install pip for Python 3.7
$curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python3.7 get-pip.py

## PyBullet Keyboard Shortcuts
Some useful keybaord shorcuts to modify the visualization GUI:
- `'g'` to open/close the search, test and params tabs
- `'j'` (with wireframe rendering activated) to show links and joints frames as RGB lines
- `'k'` (with wireframe rendering activated) to show joint axes as a black line
- `'l'` (with wireframe rendering activated) seems to have the same functionality as 'k'
- `'a'` (with wireframe rendering activated) to show collision boxes