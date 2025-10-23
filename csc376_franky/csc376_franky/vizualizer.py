import swift
import os
import threading
import numpy as np
import time
from typing import List

import roboticstoolbox as rtb
from spatialmath import SE3

# Code taken mostly from: https://github.com/Patrick15Yao/franky_toolbox_bridge,
# Please credit Patrick Yao, github id Patrick15Yao for this work.
class RtbVisualizer:
    def __init__(self, rtb_robot_model, q_start,
                 base_pose = SE3(0, 0, 0), gripper_q = [0.01, 0.01]):
        # self.rtb_robot_model = copy.deepcopy(rtb_robot_model) # Copy makes robot destroyed in visualizer? odd.
        self.rtb_robot_model = rtb.models.Panda()
        self.rtb_robot_model.q = q_start
        self.rtb_robot_model.base = base_pose
        self.rtb_robot_model.grippers[0].q = gripper_q
        try:
            self.env = swift.Swift()
            self.env.launch()
            self.env.add(self.rtb_robot_model)
        except Exception as e:
            print(f"[RTB Process {os.getpid()}] Error launching Swift: {e}")
            return

        self.render_period = 0.02 # 50 hz, should be faster than run_joint_traj dt
        self._is_render_running = True
        self._rtb_rendering_thread = threading.Thread(target=self.__run_render_loop, daemon=True)
        self._rtb_rendering_thread.start()

    def move_gripper(self, target_width: float, speed: float):
        """ Moves the simulations Franka gripper to the target_width.

        Parameters
        ----------
        target_width: float
            Target width of the Franka gripper.
        speed: float
            Delay to when the gripper finishes moving
        """
        # TODO, Radian: I am too lazy to make this propely move
        # For now the gripper will just jump

        # Get current gripper state (assuming gripper width is stored in first gripper)
        if hasattr(self.rtb_robot_model, 'grippers') and len(self.rtb_robot_model.grippers) > 0:
            # Current gripper finger positions (each finger is half the total width)
            current_finger_pos = self.rtb_robot_model.grippers[0].q[0] if len(self.rtb_robot_model.grippers[0].q) > 0 else 0.02
            current_width = current_finger_pos * 2.0

            # Calculate movement distance and time
            distance = abs(target_width - current_width)
            movement_time = distance / speed if speed > 0 else 0.1
            movement_time +=0.5

            # Update gripper directly to target width
            finger_pos = target_width / 2.0
            self.rtb_robot_model.grippers[0].q = [finger_pos, finger_pos]

            # Sleep for the calculated movement time to simulate realistic timing
            time.sleep(movement_time)
            return True
        else:
            return False

    def run_joint_trajectory(self, q_trajectory: List[np.ndarray], dt: float):
        """ Updates the rtb_robot_model's (that was passed to the env) joint positions
        with the joint trajectory (q_trajectory) with a time delay of (dt), 
        mimicking what will happen on the real robot control.
        
        Parameters
        -----------
        q_trajectory: List[np.ndarray] 
            List of np.ndarray of joint positions with size of joint_dof of robot.
        self.dt: float 
            Delta time between trajectory points of the cartsian_pose_traj.
        """
        for q in q_trajectory:
            self.rtb_robot_model.q = q
            time.sleep(dt)

    def stop(self):
        """ Cleanly stops the rendering thread. """
        self._is_render_running = False
        self._rtb_rendering_thread.join()

    def __run_render_loop(self):
        while self._is_render_running:
            try:
                self.env.step(self.render_period)
                time.sleep(self.render_period)
            except Exception as e:
                print(f"[RTB Process] Error in rendering loop: {e}")
                break
