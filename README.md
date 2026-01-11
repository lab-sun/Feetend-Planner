# Feetend-Planner
This is the official implementation of the RAL paper Stable Trajectory Planning for Quadruped Robots using Terrain Features at Feet End.



https://github.com/user-attachments/assets/ecc02974-a24c-4641-b019-2e7a767a27fd



https://github.com/user-attachments/assets/8ee721f1-823a-420b-87f8-0dcba56459e3



https://github.com/user-attachments/assets/48044db0-76d1-404e-8f27-195da204c810



https://github.com/user-attachments/assets/b509300d-83c3-4a8f-9729-2a8d6bf6d986


https://github.com/user-attachments/assets/ae279b96-3f4c-4735-b2dc-c027624cdbd0

The perceptive control architecture please refer to https://github.com/qiayuanl/legged_perceptive

# Simulation
1. Install ocs2 following instructions from https://github.com/leggedrobotics/ocs2
2. Run the perceptive locomotion demo in ocs2.
```
   roslaunch ocs2_anymal_loopshaping_mpc perceptive_mpc_li.launch
4. Create your own terrain with grayscale. Assign the grayscale file path in the launch file
5. Put the trajtories compouted with compute_body_traj.py into "ocs2/ocs2_robotic_examples/ocs2_perceptive_anymal/ocs2_anymal_loopshaping_mpc/data/"
6. Run roslaunch ocs2_anymal_loopshaping_mpc perceptive_mpc_li.launch

