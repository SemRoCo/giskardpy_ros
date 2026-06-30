# This repo is outdated!!!
It has been moved to [https://github.com/cram2/cram_ros2_packages](https://github.com/cram2/cram_ros2_packages) and is only a thin ros wrapper for [https://github.com/cram2/cognitive_robot_abstract_machine/tree/main/giskardpy](https://github.com/cram2/cognitive_robot_abstract_machine/tree/main/giskardpy)

Old README below:

# Giskardpy_ros
A motion control framework for ROS2 implemented using [Giskardpy](https://github.com/SemRoCo/giskardpy/tree/giskard_library).

## Installation instructions for Ubuntu 24.04 + Jazzy

### ROS Workspace
First, install [Giskardpy](https://github.com/SemRoCo/giskardpy/tree/giskard_library) and switch to your virtual env.
```
workon giskardpy
```
Setup workspace
```
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/giskard_ws/src && cd ~/giskard_ws
colcon build
cd src
git clone https://github.com/SemRoCo/giskardpy_ros.git -b ros2-jazzy-main
pip install -r giskardpy_ros/requirements.txt
cd ..
vcs import src < src/giskardpy_ros/$ROS_DISTRO.repos 
rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
colcon build --merge-install
source ~/giskard_ws/install/setup.bash
```
### Tutorials
https://github.com/SemRoCo/giskardpy/wiki

### How to cite
```
@phdthesis{stelter25giskard,
	author = {Simon Stelter},
	title = {A Robot-Agnostic Kinematic Control Framework: Task Composition via Motion Statecharts and Linear Model Predictive Control},
	year = {2025},
	doi = {10.26092/elib/3743},	
}
```
