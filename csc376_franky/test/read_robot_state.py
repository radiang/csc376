import numpy as np
import csc376_franky

# Create controller
controller = csc376_franky.FrankaJointTrajectoryController("192.168.1.107")

# Get current state
current_pos = controller.get_current_joint_positions()
robot_state = controller.get_current_robot_state()

print(f"Current joint positions: {current_pos}")
print(f"Current joint velocities: {robot_state.q}")
print(f"End-effector pose: {np.array(robot_state.O_T_EE).reshape(4,4).transpose()}")
# Read the python/bind_franka_joint_trajectory_controller.cpp for the robot state elements

