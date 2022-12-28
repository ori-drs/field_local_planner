# Locally Reactive Controller

This package implements 2D position controllers, some of them aware of the environment.
It currently has 3 implementations:

- `trackline`: Implements a *blind* controller similar to the original position controller. **Not aware of the environment**
- `falco`: Implements a *perceptive* controller based on the [FALCO system from CMU](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/noetic/src/local_planner)
- `rmp`: Implements a *perceptive* controller using Riemannian Motion Policies (RMP) based on [this paper](https://arxiv.org/abs/1904.01762) from the University of Washington and NVIDIA.

Both `falco` and `rmp` require access to an elevation map as a local representation of the environment.

## Dependencies

The package should work with the usual DRS setup. Additionally, you need:

- [`rmp` package](https://github.com/ori-drs/rmp): Implements the basic accelerations used to define the optimization problem
- [`grid_map_filters_drs` package](https://github.com/ori-drs/grid_map_filters_drs)
- ['teleop_twist_joy']: `sudo apt install ros-noetic-teleop-twist-joy`. Required if you want to use the controller using twists as input.

## Running the controller

The controllers implement **the same interfaces as the original position controller**. All the input and output signals, including visualizations, preserve the same names, so as to make the integration easier.

To run a controller, you only need to run:

```sh
roslaunch field_local_planner controller.launch controller:=<controller_name>
```

**The default controller is RMP**. This will launch the main node using the controller specified by `<controller_name>`. If the controller is not `trackline` it is assummed to be *perceptive*, so a *filter chain* (from `grid_map_filters_drs`) will be also launched to compute filtered layers on the elevation map.

If you're running the ANYmal simulator, you can easily send position commands using the `2D Nav Goal` above.

## Running the controller in twist mode

The controller also can be used to send velocity commands and correct them to be collision-free. This only requires to launch the controller and the following:

```sh
roslaunch field_local_planner joystick_teleop.launch
```

By default it uses the configuration of a [Xbox controller](config/teleop_twist_joy/xbox.config.yaml). `RB` must be pressed to send commands and the left and right joysticks are used to move forward/backward or rotate in place, respectively.

## Visualizations

Depending on the controller, some extra visualizations will be available:

- `trackline` shares the same visualizations as the original position controller.
- `falco` also has a visualization of the free paths using the precomputed trajectories
- `rmp` shows the control spheres and computed accelerations for each control point.

The `controller.rviz` file can be used as a reference.

## Configuring controllers

### Common parameters

By default the launchfile is configured for ANYmal C. If you want to setup the main signals for other robot, you can check the [launchfile for extrm](/launch/field_local_planner_extrm.launch)

The launchfile includes a description of the main common parameters required by the controller, such as the robot's specifications, as well as elevation map processing (voxel filters, maximum size of the elevation map, and traversability threshold).

### Controller-specific parameters

Since FALCO and RMP use way different approaches, their parameters are set in YAML files available in the [config/controllers](config/controllers/) folder. They are loaded to the parameter server when you run the launchfile, please refer to the files for further details.

## Implementation aspects

All the controllers share the structure implemented in [`controller_base.cpp`](src/field_local_planner/controllers/controller_base.cpp), which implements the interfaces to set goals, current pose, and elevation map data when required. It also implements the state machine of the controller, which has the following states:

- `UNKNOWN`: Starting state
- `FINISHED`: When the goal was reached according to the distance threshold
- `TURN_TO_GOAL`: before moving forward, the robot will rotate to head towards the goal
- `FORWARD`: Main method to make the robot approach the goal
- `TURN_TO_DESTINATION`: Once the robot got close to the goal, it will adjust its heading to match the final pose.
- `UNREACHABLE` (not fully implemented): When the robot is not able to do more progress for the current goal.

Each controller inherits this class and is required to implement virtual methods for each state of the state machine. For now, if you don't want to use the state machine, is just a matter of implementing the `FORWARD` state and call that method in the other states.

There are some slides explaining this [here](https://docs.google.com/presentation/d/1KO7pQUNO1Ck4Ubecnb5rPggUPdeabFRs3zTrNR90MZo/edit?usp=sharing) as well.
