[TOC]
# 1. Introduction
   This repository provides an adra server to connect the real adra and controll it by sending command, and publish the realtime data. The developing and testing environment are: Ubuntu 18.04 + ROS melodic.  

# 2. Getting started with 'adra_ros'

## 2.1 Create a catkin workspace
   If you already have a workspace, skip and move on to next part.
   Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
   Please note that this readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.

## 2.2 Obtain the package
```bash
cd ~/catkin_ws/src
git clone https://github.com/UmbraTek/adra_ros.git
```
## 2.3 Install other dependent packages
```bash
rosdep update
rosdep check --from-paths . --ignore-src --rosdistro melodic
```
Please change 'melodic' to the ROS distribution you use. If there are any missing dependencies listed. Run the following command to install:  
```bash
rosdep install --from-paths . --ignore-src --rosdistro melodic -y
```
And chane 'melodic' to the ROS distribution you use.  

## 2.4 Build the code
```bash
cd ~/catkin_ws
catkin_make
```
## 2.5 Source the setup script
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Skip above operation if you already have that inside your ~/.bashrc. Then do:
```bash
source ~/.bashrc
```
## 2.6 First launch the server

### 1. Chmod the port Authority

If you use the USB-to-RS485 module, you need To set the USB port permissions. This operation is not required if EtherNetb-To-Rs485 module is used.

    sudo chmod 777 /dev/ttyUSB0  (if you usb port is ttyUSB0)

### 2.Launch the server

Youe can change the port, baud, id value,which match the adra you get. The publish value is true mean that publish the realtime data which on '/adra_publish' channel. 

> If TCP connection of USB-To-Rs485 module is used

```
roslaunch adra_ros adra_server.launch device:="1" bus:="0" port:="/dev/ttyUSB0" baud:="921600" id:="1" publish:="true"
```
> If TCP connection of EtherNetb-To-Rs485 module is used

```
roslaunch adra_ros adra_server.launch device:="3" bus:="0" port:="192.168.1.168" baud:="921600" id:="1" publish:="true"
```

> If TCP connection of EtherNetb-To-Rs485 module is used

```
roslaunch adra_ros adra_server.launch device:="4" bus:="0" port:="192.168.1.168" baud:="921600" id:="1" publish:="true"
```

## 2.7  Excute the command to communicate with server 
Server's communication interface is below:

```
string api_name
string[] args
---
string[] rets
```
The first arg is api_name: like "connect", "disconnect", "into_motion_mode_pos" ...
The second arg is api's args. Most api's first arg is ID. If you need to commnunicate with adra which ID is 1.
You should send like below:

```
rosservice call /adra_server "get_motion_mode" "['1']"
```

### 2.7.1 Positon mode example

```
rosservice call /adra_server "into_motion_mode_pos" "['1']"
rosservice call /adra_server "into_motion_enable" "['1']"
rosservice call /adra_server "set_pos_target" "['1','53.6']" //let adra position move to target 53.6 rad
```
### 2.7.2 Velicity mode example  

```
rosservice call /adra_server "into_motion_mode_vel" "['1']"
rosservice call /adra_server "into_motion_enable" "['1']"
rosservice call /adra_server "set_vel_target" "['1','15.6']" //let adra run in 15.6 rad/s velicity
```
### 2.7.3 Current mode example    

```
rosservice call /adra_server "into_motion_mode_tau" "['1']"
rosservice call /adra_server "into_motion_enable" "['1']"
rosservice call /adra_server "set_tau_target" "['1','1.0']" //let adra' run in 1.0 A current
```


There are all server call  below:

```
rosservice call /adra_server "connect" "['/dev/ttyUSB0','1','921600']" // first time you roslaunch the server,it will connect automatic, so you don't need to connec again.
rosservice call /adra_server "disconnect" "[]"  
rosservice call /adra_server "into_motion_mode_pos" "['1']"   // into position mode
rosservice call /adra_server "into_motion_mode_vel" "['1']"   // into velicity mode
rosservice call /adra_server "into_motion_mode_tau" "['1']"   // into current mode
rosservice call /adra_server "get_motion_mode" "['1']"        // get motion mode
rosservice call /adra_server "get_error_code" "['1']"         // get error code

rosservice call /adra_server "into_motion_enable" "['1']"     // set enable
rosservice call /adra_server "into_motion_disable" "['1']"    // set disable
rosservice call /adra_server "get_motion_enable" "['1']"      // get enable state

rosservice call /adra_server "into_brake_enable" "['1']"      // set brake enable
rosservice call /adra_server "into_brake_disable" "['1']"     // set brake disable
rosservice call /adra_server "get_brake_enable" "['1']"       // get brake enable state

rosservice call /adra_server "set_pos_target" "['1','53.6']"  // set target position. position_value's unit is rad
rosservice call /adra_server "get_pos_target" "['1']"         // get target position
rosservice call /adra_server "get_pos_current" "['1']"        // get current position

rosservice call /adra_server "set_vel_target" "['1','15.6']"  // set target velocity. velocity_value's unit is rad/s
rosservice call /adra_server "get_vel_target" "['1']"         // get target velocity
rosservice call /adra_server "get_vel_current" "['1']"        // get current velocity

rosservice call /adra_server "set_tau_target" "['1','1.0']"   // set target current. current_value's unit is A
rosservice call /adra_server "get_tau_target" "['1']"         // get target current
rosservice call /adra_server "get_tau_current" "['1']"        // get current current
```



## 2.8  Communicate with server through code 
If you want to communicate with server through cpp, you can refer to **adra_clent.cpp**.
```
rosrun adra_ros adra_client
```

## 2.9  Get the realtime data on 'adra_publish' topic
The topic data type is :
```
uint16    id
float32 position
float32 velocity
float32 current
```

You can run the cammand below to see the realtime data.
```
rostopic echo '/adra_publish'
```

You also can refer to **adra_listener.cpp** to learn how to listener data by cpp.
```
rosrun adra_ros adra_listener
```
