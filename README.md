# 基于Gazebo的单机全链路仿真
用于项目所需,整理以下配置环境以避免重复劳动

## Overview
**Gazebo**使用的模型配置环境:[rotors_simulator](https://github.com/ethz-asl/rotors_simulator)

**Planner**:[ego-planner-swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm),修改了traj_server以适应Gazebo的控制接口,规划时只启动了单机


## Compilation

- 如果已经装好ROS，下面的这些安装步骤有些可以不用做
- 如果装好 ROS noetic ， 需要把 melodic 都换成 noetic

**1. Gazebo环境配置**
    (Ubuntu 18.04 with ROS Melodic and Gazebo 9,其他的配置环境请参考[rotors_simulator](https://github.com/ethz-asl/rotors_simulator))
```
sudo apt update
sudo apt install ros-melodic-joy ros-melodic-octomap-ros ros-melodic-mavlink ros-melodic-control-toolbox
sudo apt install python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev # 可以不装
sudo apt install python-rosinstall python-rosinstall-generator build-essential # 可以不装
```

(**非必须步骤**)如果在进行下面所有的环境配置和编译过程之后,Gazebo无法正常打开,则运行下列命令
```
sudo apt-get remove ros-melodic-gazebo* gazebo*
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo9 gazebo9-* ros-melodic-gazebo-*
sudo apt upgrade
```

**2.编译所有代码**

> 如果没装 catkin tools 可以用 catkin_make 编译，如果出现依赖问题就多编几次。
> 一定要关闭所有其它应用程序然后用命令行编译，如果内存爆了就多编几次。多编的时候不需要全删直接接着编就好了，之前编过的会自动跳过。

在项目路径下,先对两个包进行编译
```
catkin build quadrotor_msgs mav_msg
```

```
catkin build
```

## 使用说明
```
source devel/setup.bash
chmod 777 run.sh
./run.sh
```
开始运行后等待15s左右,rviz和gazebo陆续打开,无人机会先自动飞一段轨迹完成vins初始化,完成结束后,可以在rviz里按快捷键G,然后使用鼠标在地图中选择一个终点,触发轨迹规划,无人机自动起飞.

## 接口说明
1.**Planner**
- Subscribers
    位姿信息:`/hummingbird/odometry_sensor1/odometry`
    深度图:`/hummingbird/vi_sensor/camera_depth/depth/disparity`
    终点:`/hummingbird/goal`
- Publishers
    控制:`/hummingbird/command/trajectory`


## Debug

### Ubuntu 20.04

编译时 OpenCV 报错：
```
src/rotors_simulator/rotors_gazebo_plugins/src/gazebo_odometry_plugin.cpp:94:48: error: ‘CV_LOAD_IMAGE_GRAYSCALE’ was not declared in this scope
```
把第94行`covariance_image_ = cv::imread(image_name, CV_LOAD_IMAGE_GRAYSCALE);` 后面改成 `cv::IMREAD_GRAYSCALE`


