# Configuration Control for Physical Coupling of Heterogeneous Robot Swarms
Sha Yi, Zeynep Temel, Katia Sycara

<img src="/img/form_mesh.gif" width="60%">

In this repo, we include the hardware design files and the code for our PuzzleBot configuration control project. This is based on the [original PuzzleBot repo](https://github.com/ZoomLab-CMU/puzzlebot). For more videos, please view [this](https://www.youtube.com/watch?v=vOY5iG4dra8). If you find our work useful, please consider citing our paper:  
```
@inproceedings{yi2022configuration,
  title={Configuration Control for Physical Coupling of Heterogeneous Robot Swarms},
  author={Yi, Sha and Temel, Zeynep and Sycara, Katia},
  booktitle={2022 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2022},
  organization={IEEE}
}
```

## Preparation
### Hardware Setup
The hardware setup is the same as in the original [PuzzleBot setup](https://github.com/ZoomLab-CMU/puzzlebot). CAD files and circuit design files can be found in `files/`. The code for the circuit is `hardware_code/wifitcp_motor_only.ino`. Please modify the Wifi settings as [this](https://github.com/ZoomLab-CMU/puzzlebot).

### Software Setup
#### Simulation
Our simulation environment is based on [CoppeliaSim](https://www.coppeliarobotics.com/). The physics engine used is [Vortex Studio](https://www.cm-labs.com/vortex-studio/). Both the simulation and physics engine is free for educational use.  

Note that `Bullet` and `ODE` does not work for our concave robot body and will cause instability during contacts.  

Install the [ROS interface](https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm) for _CoppeliaSim_.  

#### Packages
Basic ROS knowledge is required to run the demo. To build the ROS package, copy the repo into your ROS catkin source directory and build the package.

## Run the Demo in Simulation
1. Start `roscore`. Make sure the package is built in your catkin source directory.
2. Launch _CoppeliaSim_. Note that this should be ran after `roscore` to enable the ROS interface of the simulator.
3. Open the example simulation environment in `simulation/random_4.ttt`. Double check the _Vortex Studio_ is running properly.
4. Run the demo provided with 
```
rosrun puzzlebot_control run.py [NUMBER_OF_ROBOTS]
```
Note that if you would like to run different number of robots, you may also need to modify the target configuration parameters in [`src/puzzlebot_control/controller.py#L266`](src/puzzlebot_control/controller.py#L266).

## Run the Demo on Hardware Platform
Please follow the same procedure as in [this](https://github.com/ZoomLab-CMU/puzzlebot).