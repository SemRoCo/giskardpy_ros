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

### Tools

#### Grasp Scoring (`scripts/tools/grasp_scoring_tool.py`)
A standalone grasp quality ranking tool that scores candidate grasp poses for a given object mesh.

It uses [trimesh](https://trimesh.org/) for mesh operations and [CGAL](https://www.cgal.org/) for AABB-tree based collision detection.
Each grasp is evaluated on three criteria:
- **Collision** — penalizes grasps where the gripper intersects the object
- **Clearance** — penalizes grasps where the gripper dips below the ground plane
- **Stability** — rewards grasps with two opposing contact normals and a good contact spread

Results are written live to `grasp_ranking_live.txt` as each grasp is evaluated.

Configure `OBJECT_UUID`, `GRIPPER_NAME`, and `BASE_PATH` at the bottom of the file before running:
```bash
python scripts/tools/grasp_scoring_tool.py
```

### How to cite
```
@phdthesis{stelter25giskard,
	author = {Simon Stelter},
	title = {A Robot-Agnostic Kinematic Control Framework: Task Composition via Motion Statecharts and Linear Model Predictive Control},
	year = {2025},
	doi = {10.26092/elib/3743},	
}
```
