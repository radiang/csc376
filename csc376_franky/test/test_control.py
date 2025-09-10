import numpy as np
import csc376_franky

# Create controller
controller = csc376_franky.FrankaJointTrajectoryController("192.168.1.107")

# Get current state
current_pos = controller.get_current_joint_positions()
robot_state = controller.get_current_robot_state()

print(f"Current joint positions: {current_pos}")
print(f"Current joint velocities: {robot_state.dq}")
print(f"End-effector pose: {np.array(robot_state.O_T_EE).reshape(4,4)}")

# Create a simple trajectory
start_pos = np.array(current_pos)
end_pos = start_pos + np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Move joint 0

# Generate linear interpolation trajectory
num_waypoints = 50
trajectory = np.zeros((num_waypoints, 7))
for i in range(num_waypoints):
    alpha = i / (num_waypoints - 1)
    trajectory[i] = start_pos + alpha * (end_pos - start_pos)

# Execute trajectory
dt = 0.02  # 50Hz
result = controller.run_trajectory(trajectory, dt)

if result == csc376_franky.ErrorCodes.Success:
    print("Trajectory executed successfully!")
elif result == csc376_franky.ErrorCodes.JointVelocitiesAboveMaximumAllowed:
    print("Trajectory too fast - reduce speed or increase waypoints")
else:
    print(f"Trajectory failed with error: {result}")

# Check for collisions during execution
final_state = controller.get_current_robot_state()
if any(final_state.joint_collision) or any(final_state.cartesian_collision):
    print("Collision detected!")

# Compute trajectory properties
differences = csc376_franky.compute_row_differences(trajectory)
max_velocity = np.max(np.abs(differences / dt))
print(f"Maximum trajectory velocity: {max_velocity} rad/s")
