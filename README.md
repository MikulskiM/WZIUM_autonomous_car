# WZIUM_autonomous_car - ROS section
Autonomous RC car project

Clone repo and place it in 'wzium' directory. Please place this repo in user's home directory as some environmental variables depend on it later.
```
git clone https://github.com/MikulskiM/WZIUM_autonomous_car.git wzium
```

## Requirements
TODO: place link to raspberry pi image 

### ROS Melodic
Available only on Ubuntu 18.04 LTS.
Link to install [here](http://wiki.ros.org/melodic/Installation/Ubuntu).

Some setup in ~/.bashrc file:
```
# Networking - substitute <ip_address> with raspberry ip address
export ROS_MASTER_URI=http://<ip_address>:11311
export ROS_IP=<ip_address>

# Add to ros packages - can cause warning before first compile
export ROS_PACKAGE_PATH=~/wzium:${ROS_PACKAGE_PATH}
export CMAKE_PREFIX_PATH=~/wzium/devel:${CMAKE_PREFIX_PATH}

# Makes your package visible to ros at creating new console
source ~/wzium/devel/setup.bash
```

After setup you should bashrc so the changes can take place
```
source ~/.bashrc
```

To install ROS packages dependencies (run from workspace root directory):
```
rosdep install --from-paths src --ignore-src -r -y
```

## Build
Enter workspace root directory ('wzium')
```
catkin_make
source devel/setup.bash
```

Building packages can get lousy. You can add flag --pkg to catkin_make to specify packages that are to be built, e.g. ```catkin_make --pkg wzium```. This can save a lot of time.

## Usage
You can run nodes directly (you have to run ```roscore``` first in some console window), then run node e.g.:
```
# cpp nodes have to be built
rosrun wzium example_cpp_node
# python nodes have to be made executable
rosrun wzium example_python_node
```
or via launch files (no roscore needed) e.g.:
```
roslaunch wzium example_launch.launch
```
