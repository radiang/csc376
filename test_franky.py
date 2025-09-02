from franky import *

def cb(
        robot_state: RobotState,
        time_step: Duration,
        rel_time: Duration,
        abs_time: Duration,
        control_signal: JointPositions):
    print(f"At time {abs_time},", 
          "The current joint positions are {robot_state.q}",
          "The elbow state is {robot_state.elbow.joint_3_pos}") # This might not work
        #   "The elbow state is {robot_state.elbow.joint_3_pos()}") # MAYBE THIS?


robot = Robot("192.168.1.107")  # Replace this with your robot's IP
# robot = Robot("172.16.0.2")  # Replace this with your robot's IP

# Let's start slow (this lets the robot use a maximum of 5% of its velocity, acceleration, and jerk limits)
robot.relative_dynamics_factor = 0.05

# Move the robot 10cm along the relative y-axis of its end-effector
motion = CartesianMotion(Affine([0.0, 0.1, 0.0]), ReferenceType.Relative)
motion.register_callback(cb)
robot.move(motion)

