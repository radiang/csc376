from franky import *

target_elbow_state = 0.0

def cb(
        robot_state: RobotState,
        time_step: Duration,
        rel_time: Duration,
        abs_time: Duration,
        control_signal: JointPositions):
    x = 0
    # print(f"At time {abs_time},\n" +
    #     #   f"The current joint positions are {robot_state.q}\n" +
    #       f"The elbow state is {robot_state.elbow.joint_3_pos}\n") # This might not work

def main():
    robot = Robot("192.168.1.107")  # Teachinglab IP
    # robot = Robot("172.16.0.2")  # Medcvr IP

    # Let's start slow (this lets the robot use a maximum of 5% of its velocity, acceleration, and jerk limits)
    robot.relative_dynamics_factor = 0.05

    cartesian_state = robot.current_cartesian_state
    robot_pose = cartesian_state.pose  # Contains end-effector pose and elbow position
    ee_pose = robot_pose.end_effector_pose
    elbow_pos = robot_pose.elbow_state.joint_3_pos
    joint_state = robot.current_joint_state
    joint_pos = joint_state.position

    print("current_elbow_pos ", elbow_pos)
    # print("Current joint_pos ", joint_pos)

    yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
    if yes_or_else != "yes":
        print("User did not type [yes], will not run on real robot")
        return 

    # Move the robot 10cm along the relative y-axis of its end-effector
    # Using current elbow state as target

    y_movement = -0.1

    motion = CartesianMotion(RobotPose(Affine([0.0, y_movement, 0.0]), ElbowState(elbow_pos)), ReferenceType.Relative) # Using elbow pos 
    # motion = CartesianMotion(RobotPose(Affine([0.0, y_movement, 0.0]))                       , ReferenceType.Relative)

    motion.register_callback(cb)
    robot.move(motion)

    new_elbow_pos = robot.current_cartesian_state.pose.elbow_state.joint_3_pos
    print("new_elbow_pos ", new_elbow_pos)

if __name__ == "__main__":
    main()
