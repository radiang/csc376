import numpy as np

class RobotWrapper:
    def __init__(self, has_visualizer: bool, run_on_real: bool,
                 panda_rtb_model, q_start: np.ndarray, ip_address: str):
        self.vizualizer = None
        if has_visualizer:
            from csc376_franky.vizualizer import RtbVisualizer
            self.vizualizer = RtbVisualizer(panda_rtb_model, q_start)    
        self.real_robot = None
        if run_on_real:
            import csc376_bind_franky
            self.real_robot = csc376_bind_franky.FrankaJointTrajectoryController(ip_address)

    def get_current_joint_positions(self):
        if self.real_robot:
            return self.real_robot.get_current_joint_positions()
        if self.vizualizer:
            return self.vizualizer.rtb_robot_model.q

    def run_joint_trajectory(self, q_traj: np.ndarray, dt: float):
        if self.vizualizer:
            input("To run on the sim robot, press enter\n")
            self.vizualizer.run_joint_trajectory(q_traj, dt)
        if self.real_robot:
            yes_or_else = input("To run on the real robot, type [yes], then press enter\n")
            if yes_or_else != "yes":
                print("User did not type [yes], will not run on sim/real robot")
                return False
            self.real_robot.run_joint_trajectory(q_traj, dt)
        return True

    def stop(self):
        if self.vizualizer:
            self.vizualizer.stop()
        if self.real_robot:
            self.real_robot.stop()
