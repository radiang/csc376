import roboticstoolbox as rtb
from spatialmath import SE3

from csc376_franky.motion_generator import RuckigMotionGenerator
import numpy as np

def main():
    np.set_printoptions(precision=4, suppress=True,)    

    simulation = False # Radian: I don't like using sim bool
    panda_rtb_model = rtb.models.Panda()

    if simulation:
        from csc376_franky.vizualizer import RtbVisualizer
        q_start = panda_rtb_model.qr
        robot = RtbVisualizer(panda_rtb_model, q_start)
    else:
        import csc376_bind_franky
        robot = csc376_bind_franky.FrankaJointTrajectoryController("192.168.1.107")
        robot.setupSignalHandler()
        q_start = robot.get_current_joint_positions()

    relative_vel_factor = 0.02
    relative_acc_factor = 0.01
    relative_jerk_factor = 0.05

    motion_generator = RuckigMotionGenerator()
    
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
        cartesian_traj, dt = motion_generator.calculate_cartesian_pose_trajectory(se3_current, se3_target,
                                                                                  relative_vel_factor, relative_acc_factor, relative_jerk_factor) 
        q_traj = motion_generator.cartesian_pose_to_joint_trajectory(panda_rtb_model, q_current, cartesian_traj)
        q_trajs.append(q_traj)
        se3_current = se3_target
        q_current = q_traj[-1]

    # V. Run on robot (sim or real)
    for q_traj in q_trajs:
        yes_or_else = input("To run on the sim/real robot, type [yes], then press enter\n")
        if yes_or_else != "yes":
            print("User did not type [yes], will not run on sim/real robot")
            return robot
        robot.run_joint_trajectory(q_traj, dt)
    
    return robot

if __name__ == "__main__":
    robot = main()
    robot.stop() # Makes sure render thread ends

