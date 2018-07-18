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

pano_exploration also relies on the [scarab](https://github.com/KumarRobotics/scarab) repository. Follow the instructions given to clone and build it.
