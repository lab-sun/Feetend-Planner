# OCS2 - Optimal Control for Switched Systems

## Overview

OCS2 is a C++ toolbox developed by the Robotic Systems Lab at ETH Zurich for Optimal Control for Switched Systems. It provides efficient implementations of model predictive control (MPC) and other optimal control algorithms specifically designed for complex robotic systems, including legged robots.

## Repository

The official OCS2 repository can be found at:
- GitHub: [https://github.com/leggedrobotics/ocs2](https://github.com/leggedrobotics/ocs2)

## Key Features

- **Model Predictive Control (MPC)**: High-performance MPC solvers for real-time control
- **Switched Systems**: Support for systems with discrete modes (e.g., different contact configurations in legged locomotion)
- **Multiple Solvers**: Includes SLQ (Sequential Linear Quadratic), PISOC (Parallel Interior-point Sequential Optimal Control), and DDP (Differential Dynamic Programming)
- **ROS Integration**: Seamless integration with ROS for robotics applications
- **Legged Robotics Focus**: Specialized tools for legged robot control and locomotion

## Relevance to Feetend-Planner

This project builds upon OCS2's optimal control framework for:
- Trajectory optimization for quadruped robots
- Terrain-aware foot placement planning
- Stable locomotion planning considering terrain features at the feet

## Installation

For detailed installation instructions, please refer to the [OCS2 repository](https://github.com/leggedrobotics/ocs2).

## Citation

If you use OCS2 in your research, please cite:
```
@software{ocs2,
  author = {Farbod Farshidian and et al.},
  title = {{OCS2}: Optimal Control for Switched Systems},
  url = {https://github.com/leggedrobotics/ocs2},
  organization = {Robotic Systems Lab, ETH Zurich}
}
```

## Related Projects

- Main project: [Feetend-Planner](https://github.com/lab-sun/Feetend-Planner)
- Perceptive control: [legged_perceptive](https://github.com/qiayuanl/legged_perceptive)
