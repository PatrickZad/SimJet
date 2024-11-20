# A Simple Demo of ROS-based Development on Hiwonder JetRover & JetHexa

This repo presents a ROS-based robot camera visualization tool and illustrates how to deploy the ROS nodes on Hiwonder JetRover & JetHexa robots, serving as an exemplar for ROS-baed development on the robots.

# Development Environment

## JetRover

* Linux: Ubuntu 20.04
* ROS: ROS Noetic
* Python2: Python 2.7
* Python3: Python 3.6
* Pytorch: Pytorch 1.10

## JetHexa

* Linux: Ubuntu 18.04
* ROS: ROS Melodic
* Python2: Python 2.7
* Python3: Python 3.6
* Pytorch: Pytorch 1.10

## PC

* Linux:
  * Depends on the required ROS version. Pre-built ROS Melodic is for Ubuntu 18.04 and pre-built ROS Noetic is for Ubuntu 20.04. Using another version or distribution of Linux may requires re-building ROS.
  * The test of this demo was conducted with Manjaro Linux 24.1.2.
* ROS:
  * Better be aligned with the robot.
  * The test of this demo was conducted with ROS Noetic.
* Python3:
  * Not specificed.

# Code Structure

This demo sets two ROS nodes on the robot and PC, respectively. The ROS node on the robot reads images from the camera and sends them to the PC. The ROS node on the PC receives images and shows them.

``` bash
.
└── src
    ├── cam_vis
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── cam_vis
    │   ├── launch
    │   │   ├── cam_robo_hexa.launch # launch file for JetHexa
    │   │   └── cam_robo_rover.launch # launch file for JetRover
    │   ├── msg
    │   │   └── Img.msg # manually defined image message
    │   ├── package.xml
    │   ├── scripts
    │   │   ├── cam_read_hexa.py # robot node on JetHexa
    │   │   ├── cam_read_rover.py # robot node on JetRover
    │   │   └── cam_show.py # PC node
    │   └── src
    └── CMakeLists.txt

```

To build the code, on the robot and the PC, run

``` bash
cd ${directory_containing_the_source_code}
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python${python3_version}m # if ROS built with Python2
catkin_make # if ROS built with Python3
```

# Deployment Workflow

This demo assumes the ROS master is launched on the robot. The robot node is supposed to be started at first.

## Prerequest

Use the WonderAi APP to connect the robot to the same LAN with the PC.

## JetRover

``` bash
sudo systemctl stop start_app_node
cd ${directory_containing_the_source_code}
source devel/setup.zsh
roslaunch cam_vis cam_robo_rover.launch
```

## JetHexa

``` bash
sudo systemctl stop jethexa_bringup.service
export ROS_HOSTNAME=${robot_ip}
export ROS_MASTER_URI=http://${robot_ip}:11311
roscore
```

Then open a new terminal and run

```bash
cd ${directory_containing_the_source_code}
source devel/setup.zsh
roslaunch cam_vis cam_robo_hexa.launch
```

## PC

``` bash
source /opt/ros/noetic/setup.${your_shell} # 
cd ${directory_containing_the_source_code}
source devel/setup.${your_shell}
rosrun cam_vis cam_show.py
```

# Other Tips

1. The ROS on the robot is built with Python2. Running the customed ROS node with Python3 and importing the pre-built packages (e.g. the cv_bridge package) may fails due to incompatibility. Thus this demo defines a new image message for transfering image data.
2. Re-building is the least recommended way to install ROS on your PC. It will make you unhappy 😞.
3. If some hard-to-fix malfunctions happend (e.g. the mobile APP remote control do not work), you may need to reinstall the OS of the robot using the image provided by the manufacture.
4. If JetHexa cannot access externel links in the LAN mode, insert

    ```bash
    nameserver 8.8.8.8
    ```

    to the begining of ```/etc/resolv.conf```.
5. Carefully check the versions of dependecies when maually install new packages. Try not to upgrade the pre-installed ones in this case.
6. Pay attention to the remaining battery power. The robot will beep loudly when battery is low.
