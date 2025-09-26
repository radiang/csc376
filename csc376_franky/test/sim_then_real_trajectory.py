import roboticstoolbox as rtb
from csc376_franky.motion_generator import RuckigMotionGenerator
import numpy as np

def main():
    np.set_printoptions(precision=4, suppress=True,)    

    simulation = False # I actually don't like using a sim bool, makes code complex
    # Prefer to just rewrite in individual scripts

    if simulation:
        from csc376_franky.test.vizualizer import RtbVisualizer
        visualizer = RtbVisualizer(panda_rtb_model, q_start)
    else:
        import csc376_franky
        csc376_franky_robot = csc376_franky.FrankaJointTrajectoryController("192.168.1.107")
        csc376_franky_robot.setupSignalHandler()
        q_start = csc376_franky_robot.get_current_joint_positions()

    # I. Speed factor settings
    relative_vel_factor = 0.02
    relative_acc_factor = 0.01
    relative_jerk_factor = 0.05

    # II. RTB, Ruckig, csc376_franky, and Visualizer setup
    panda_rtb_model = rtb.models.Panda()
    motion_generator = RuckigMotionGenerator(relative_vel_factor, relative_acc_factor, relative_jerk_factor)
    
    # III. Calculate your goal
    se3_start   = panda_rtb_model.fkine(q_start)
    se3_target = SE3.Ty(-0.10) * se3_start # Relative to start position, pre-multiply for world frame reference
    print("q_start", q_start)
    print("se3_start", se3_start)
    print("se3_target", se3_target)
    
    # IV. Visualize the trajectory in simulation
    # The following 3-step process is what happens when we call franky_robot.move(cartesian_motion)
    cartesian_traj, dt = motion_generator.calculate_cartesian_pose_trajectory(se3_start, se3_target) # Interpolation and trajectory parameterization
    q_traj = motion_generator.cartesian_pose_to_joint_trajectory(panda_rtb_model, q_start, cartesian_traj)
    input("Press enter, to run in visualizer\n")
    visualizer.move_gripper(0.07, 0.1)
    visualizer.run_joint_trajectory(q_traj, dt)

    # TODO helper function for projects:
    # calculate_joint_trajectory_and_visualize(panda_rtb_model, vizualizer, q_start, se3_target)

    # V. Run on real robot
    yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
    if yes_or_else != "yes":
        print("User did not type [yes], will not run on real robot")
        return visualizer

    csc376_franky_robot.run_trajectory(q_traj, dt)
    return visualizer

if __name__ == "__main__":
    visualizer = main()
    visualizer.stop() # Makes sure render thread ends

