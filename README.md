### Simple Planner ROS Package

This ROS package provides a simple path planning functionality using a grid map.

## Installation
1. Clone this repository into your ROS workspace (your_workspace directory):
 ```bash
cd path/to/your_workspace/src
git clone https://github.com/your_username/simple_planner.git
 ```
2.Build the ROS package using catkin_make:
 ```bash
cd ..
catkin_make
```
3.Source the ROS setup file:
```bash
source devel/setup.bash
```
## Running the Simple Planner
# 1.Launching the Planner Node
1.Start roscore in a terminal:
```bash
roscore
```
2. Open a new terminal and navigate to your ROS workspace:
```bash
cd path/to/your_workspace
```
3. Build the ROS workspace:
```bash
catkin build
```
4. Source the ROS setup file:
```bash
source devel/setup.bash
```
5. Run the planner node:
```bash
rosrun simple_planner node_gridmap
```
# 2. Launching RViz for Visualization
1. Open a new terminal window.
2. Launch RViz:
```bash
rosrun rviz rviz
```
3. In RViz, add the following displays:
- Add -> By topic -> Select /map to visualize the occupancy grid map.
- Add -> By topic -> Select /path to visualize the planned path.
4. Adjust the view parameters for a better visualization:
- Set X to 830, Y to 830, and Z to 1300.

Now you should see the map and the planned path displayed in RViz.
