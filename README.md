# Feetend-Planner
This is the official implementation of the RAL paper Stable Trajectory Planning for Quadruped Robots using Terrain Features at Feet End.


https://github.com/user-attachments/assets/ae279b96-3f4c-4735-b2dc-c027624cdbd0

The perceptive control architecture please refer to https://github.com/qiayuanl/legged_perceptive

# Simulation
1. Install ocs2 following instructions from https://github.com/leggedrobotics/ocs2
2. Run the perceptive locomotion demo in ocs2. roslaunch ocs2_anymal_loopshaping_mpc perceptive_mpc_li.launch
3. Create your own terrain with grayscale. Assign the grayscale file path in the launch file
4. Put the trajtories compouted with compute_body_traj.py into "ocs2/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/"
5. Run roslaunch ocs2_anymal_loopshaping_mpc perceptive_mpc_li.launch

