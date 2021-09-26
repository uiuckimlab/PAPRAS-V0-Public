# PAPRAS
Project-NR: Developing Plug-And-Play Robotic Arm System (PAPRAS)

## How to Run

We use a ROS1 build environment. This project can easily be made to interface with ROS1 for visualization or for running on a real or simulated robot.
```
git clone repo
cd PAPRAS
```

1. Build Docker environment
```
docker build -t papras .
```

2. Open a terminal inside the environment with this repository mounted
```
docker run -it -v `pwd`:/opt/catkin_ws/src/PAPRAS papras

or 

./gui-docker -it -v `pwd`:/opt/catkin_ws/src/PAPRAS papras
./gui-docker -c <container>
```

3. Go to `/opt/arpp`, the mounted directory, build the package, and configure ROS2
```
cd /opt/catkin_ws
catkin_make
source devel/setup.bash
```

4. Run the executable
```
roslaunch papras _demo_coffee.launch
```
Example output:
```
[ INFO] [1628267446.776641700, 2.266000000]: initialize move group sequence action
[ INFO] [1628267446.785980000, 2.275000000]: Reading limits from namespace /robot_description_planning
Loading 'pilz_industrial_motion_planner/MoveGroupSequenceService'...
[ INFO] [1628267446.871081900, 2.358000000]: Reading limits from namespace /robot_description_planning
[ INFO] [1628267446.961461200, 2.447000000]: 

********************************************************
* MoveGroup using: 
*     - ApplyPlanningSceneService
*     - ClearOctomapService
*     - CartesianPathService
*     - ExecuteTrajectoryAction
*     - GetPlanningSceneService
*     - KinematicsService
*     - MoveAction
*     - PickPlaceAction
*     - MotionPlanService
*     - QueryPlannersService
*     - StateValidationService
*     - SequenceAction
*     - SequenceService
********************************************************

[ INFO] [1628267446.961596600, 2.447000000]: MoveGroup context using planning plugin ompl_interface/OMPLPlanner
[ INFO] [1628267446.961666700, 2.447000000]: MoveGroup context initialization complete

You can start planning now!
```
