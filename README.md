# ESL ROS Library
A ROS Library developed by the ESL for internal projects.

## Pre-requisites

### Install PX4

https://dev.px4.io/en/setup/dev_env_linux_ubuntu.html

### Install ROS and MAVROS

IMPORTANT: If you are using Ubuntu 18.04, you need to install ROS Melodic. If you are using Ubuntu 16.04, you need to install ROS Kinetic.

The below commands are for Ubuntu 18.04 and ROS Melodic. If you are using Ubuntu 16.04, replace all `melodic` commands with `kinetic`.

#### Ubuntu 18.04

- Install ROS (https://wiki.ros.org/melodic/Installation/Ubuntu)
- Install MAVROS
  - `sudo apt-get install ros-melodic-mavros ros-melodic-mavros-extras`
  - `wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh`
  - `chmod +x install_geographiclib_datasets.sh`
  - `sudo ./install_geographiclib_datasets.sh`
  - `rm -rf install_geographiclib_datasets.sh`
- Install Python libraries
  - `sudo apt-get install python-rosinstall python-wstool python-rosinstall-generator python-catkin-tools -y`
  - `sudo pip install pymavlink`
  
### Create catkin workspace

The official build system of ROS is called catkin. One needs a catkin workspace to build and run ROS packages.

- `mkdir -p ~/catkin_ws/src`
- `cd ~/catkin_ws`
- `catkin init`
- `wstool init src`

The ROS packages should be in the `src` directory. We recommend to clone the ESL ROS Library contents to another folder and symlink the needed packages in the `src` directory.

- `mkdir -p ~/esl-sun`
- `cd ~/esl-sun`
- `git clone https://github.com/esl-sun/esl_ros_lib.git`
- `cd ~/catkin_ws`
- `ln -s ~/esl-sun/esl_ros_lib/px4_nav_cmd src/` (`px4_nav_cmd` is an example of a ROS package, one can symlink all the needed ROS packages here)
- `catkin build`
- `source devel/setup.bash`

## Run

- Open a terminal, navigate to the PX4 root directory and start PX4 SITL:
  - `make px4_sitl jmavsim`
- Open another terminal and start MAVROS:
  - `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`
- Open another terminal and run one of the ROS nodes in the catkin workspace directory, for example:
  - `rosrun px4_nav_cmd mc_step_input.py -t z -v 10 -d 10` (Step to 10m)

## ESL ROS Library Packages

### PX4 Navigation Commands (px4_nav_cmd)

#### Multicopter Step Input Node (mc_step_input)

Execute step inputs to the different multicopter controllers, such as angular rate, angle, linear velocity and position. One can then analyze the step response to determine whether the respective controllers meet the specifications.

For help on how to use the ROS node, run the following:
- `rosrun px4_nav_cmd mc_step_input.py --help`

#### PX4 Waypoint Scheduler (waypoint_scheduler)

A waypoint scheduler that generate position step commands to each waypoint.

At the top of the script, there are 3 variables one can change.
- waypoints: a list of waypoints consisting of N, E, D and yaw setpoints.
- threshold: this determines how small the position error and velocity should be before sending the next waypoint
- waypoint_time: if < 0, will send next waypoint when current one is reached. if >= 0, will send next waypoint after the amount of time has passed

To run the node, execute the following:
- `rosrun px4_nav_cmd waypoint_scheduler.py`