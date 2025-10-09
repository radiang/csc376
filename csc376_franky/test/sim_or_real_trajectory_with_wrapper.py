import roboticstoolbox as rtb
from spatialmath import SE3

from csc376_franky.motion_generator import RuckigMotionGenerator
from csc376_franky.robot_wrapper import RobotWrapper
import numpy as np

def main():
    np.set_printoptions(precision=4, suppress=True,)    

    panda_rtb_model = rtb.models.Panda()

    has_visualizer = False
    run_on_real = True
    q_start = panda_rtb_model.qr
    robot = RobotWrapper(has_visualizer, run_on_real, panda_rtb_model, q_start, "192.168.1.107")
    q_start = robot.get_current_joint_positions() # if it is real, will get real robot position

    relative_vel_factor = 0.02
    relative_acc_factor = 0.01
    relative_jerk_factor = 0.05

    motion_generator = RuckigMotionGenerator(relative_vel_factor, relative_acc_factor, relative_jerk_factor)
    
    # III. Calculate SE3 goals
    se3_targets = []
    se3_start   = panda_rtb_model.fkine(q_start)

    se3_target = SE3.Ty(-0.10) * se3_start
    se3_targets.append(se3_target)
    
    se3_target = SE3.Tx(-0.10) * se3_target
    se3_targets.append(se3_target)

    se3_target = SE3.Ty(0.10) * se3_target
    se3_targets.append(se3_target)

    se3_target = SE3.Tx(0.10) * se3_target
    se3_targets.append(se3_target)

    print("q_start", q_start)
    print("se3_start", se3_start)
    print("se3_targets", se3_targets)
    
    # IV. Calculate trajectories
    q_trajs = []
    se3_current = se3_start
    q_current = q_start
    for se3_target in se3_targets:
        cartesian_traj, dt = motion_generator.calculate_cartesian_pose_trajectory(se3_current, se3_target) 
        q_traj = motion_generator.cartesian_pose_to_joint_trajectory(panda_rtb_model, q_current, cartesian_traj)
        q_trajs.append(q_traj)
        se3_current = se3_target
        q_current = q_traj[-1]

    # V. Run on robot (sim, real, or both)
    for q_traj in q_trajs:
        robot.run_joint_trajectory(q_traj, dt)
    return robot

if __name__ == "__main__":
    robot = main()
    robot.stop() # Makes sure render thread ends

