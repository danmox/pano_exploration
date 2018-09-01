# pano_exploration

A ROS package for multi-robot exploration with the [scarab](https://github.com/KumarRobotics/scarab) platform using RGBD panoramas.

## Installation

### Submodules

pano_exploration relies on the openni2_xtion submodule. Initialize this submodule by executing the following commands in the cloned repo:

```shell
git submodule init
git submodule update
```

### Dependencies

Most dependencies can be installed using rosdep. Execute the following command in the src folder of your catkin workspace:

```shell
rosdep install --ignore-src --from-paths . --rosdistro=Kinetic
```

or alternatively install using apt using:

```shell
sudo apt-get install ros-kinetic-pcl-conversions ros-kinetic-tf2-kdl ros-kinetic-actionlib-msgs ros-kinetic-catkin ros-kinetic-pcl-ros ros-kinetic-nav-msgs ros-kinetic-roscpp ros-kinetic-actionlib libyaml-cpp-dev ros-kinetic-tf ros-kinetic-nodelet ros-kinetic-sensor-msgs ros-kinetic-message-runtime ros-kinetic-std-msgs ros-kinetic-message-generation ros-kinetic-cv-bridge ros-kinetic-tf2-geometry-msgs libopenni2-dev ros-kinetic-geometry-msgs ros-kinetic-rosbag
```

pano_exploration also relies on the [scarab](https://github.com/KumarRobotics/scarab) repository. Follow the instructions given to clone and build it.
