import copy
import numpy as np
import time

import roboticstoolbox as rtb
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from scipy.spatial.transform import Rotation
from spatialmath import SE3

# Ruckig Motion Generator code taken mostly from: https://github.com/Patrick15Yao/franky_toolbox_bridge/blob/main/franky_toolbox_bridge/rtb_backend.py, 
# and tailored in a more calculate traj -> then visualize way, 
# also made more functional programming style. 

def normalize_rx_angle(rv_vector):
    """
    Normalize the rx angle of the rotation vector to be between -pi and pi(deal with rx angle not continuous issue
    """
    rv_normalized = rv_vector.copy()  # Make a copy to avoid modifying original
    # Apply the rule to the 4th element (index 3)
    rx = rv_normalized[3]
    if rx > 0:
        rv_normalized[3] = np.pi - rx 
    else:
        rv_normalized[3] = -np.pi - rx
    return rv_normalized

def se3_to_rotation_vector(se3_matrix, elbow_angle = None):
    if hasattr(se3_matrix, 'A'):
        se3_array = se3_matrix.A  # SE3 object
    else:
        se3_array = np.array(se3_matrix)  # Already numpy array

    translation = se3_array[:3, 3]
    rotation_matrix = se3_array[:3, :3] 

    rotation = Rotation.from_matrix(rotation_matrix)
    rotvec = rotation.as_rotvec()  # [rx, ry, rz] in radians
    if elbow_angle is None:
        rotation_vector = np.concatenate([translation, rotvec, [0.0]])
    else:
        rotation_vector = np.concatenate([translation, rotvec, [elbow_angle]])
    return rotation_vector

def rotation_vector_to_joint_config(model, q_current, RV_new):
    """
    Convert 7D rotation vector back to 7 joint configuration.
    
    Args:
        RV_new: 7D vector [x, y, z, rx, ry, rz, elbow_angle]
    
    Returns:
        q_new: 7D joint configuration
    """
    translation = RV_new[:3]  # [x, y, z]
    rotvec = RV_new[3:6]     # [rx, ry, rz] 
    # elbow_angle = RV_new[6]  # elbow_angle
    
    rotation = Rotation.from_rotvec(rotvec)
    rotation_matrix = rotation.as_matrix()
    
    se3_matrix = np.eye(4)
    se3_matrix[:3, :3] = rotation_matrix
    se3_matrix[:3, 3] = translation       
    
    se3_pose = SE3(se3_matrix)
    q_new = model.ikine_LM(se3_pose, q0=q_current).q
    # q_new[2] = elbow_angle
    
    return q_new
    
# Ruckig is used because the internal franky uses ruckig to generate trajectories, and we
# want to visualize the franka robot's trajectory in swift before running on the real robot. 
# HOWEVER, the rucking parameters might not be the same so trajectories might slightly be different in
# sim and in real
class RuckigMotionGenerator:
    def __init__(self, relative_vel_factor = 0.02, relative_acc_factor = 0.01, relative_jerk_factor = 0.05):

        self.dt = 0.05
        self.ruckig = Ruckig(dofs=7, delta_time=self.dt)
        self.PANDA_VEL_LIM_CARTESIAN = np.array([3.0, 3.0, 3.0, 2.5, 2.5, 2.5, 2.62])   # m/s
        self.PANDA_ACC_LIM_CARTESIAN =  np.array([9.0, 9.0, 9.0, 17.0, 17.0, 17.0, 10.0]) # m/s²
        self.PANDA_JERK_LIM_CARTESIAN = np.array([4500.0, 4500.0, 4500.0, 8500.0, 8500.0, 8500.0, 5000.0]) # m/s³
        self.relative_vel_factor = relative_vel_factor
        self.relative_acc_factor = relative_acc_factor
        self.relative_jerk_factor = relative_jerk_factor

    def calculate_cartesian_pose_trajectory(self,
                       se3_start: SE3,
                       se3_target: SE3, 
                       rv = 0.02, ra=0.01, rj=0.05):
        RV_start    = se3_to_rotation_vector(se3_start)
        RV_target   = se3_to_rotation_vector(se3_target)

        N_RV_start  = normalize_rx_angle(RV_start)
        N_RV_target = normalize_rx_angle(RV_target)

        inp = InputParameter(7)
        out = OutputParameter(7)
        inp.current_position     = N_RV_start
        inp.current_velocity     = [0, 0, 0, 0, 0, 0, 0]
        inp.current_acceleration = [0, 0, 0, 0, 0, 0, 0]
        inp.target_position     = N_RV_target
        inp.target_velocity     = [0, 0, 0, 0, 0, 0, 0]
        inp.target_acceleration = [0, 0, 0, 0, 0, 0, 0]
        inp.max_velocity     = self.PANDA_VEL_LIM_CARTESIAN * self.relative_vel_factor
        inp.max_acceleration = self.PANDA_ACC_LIM_CARTESIAN * self.relative_acc_factor
        inp.max_jerk         = self.PANDA_JERK_LIM_CARTESIAN * self.relative_jerk_factor
        inp.enabled = [True, True, True, True, True, True, True]
        res = Result.Working
    
        cartesian_pose_traj = []
        while res == Result.Working:
            res = self.ruckig.update(inp, out) # interpolates to robot vel, acc, and jerk constraints
            cartesian_pose_traj.append(out.new_position)
            out.pass_to_input(inp)
        return cartesian_pose_traj, self.dt

    def cartesian_pose_to_joint_trajectory(self, model, q_start, cartesian_pose_traj):
        q_current = copy.deepcopy(q_start)
        q_traj = []
        for cartesian_pose in cartesian_pose_traj:
            q_current = rotation_vector_to_joint_config(
                model, q_current, normalize_rx_angle(cartesian_pose)) # convert to joint traj
            q_traj.append(q_current)
        return q_traj

import swift
import os
import threading 

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
            
    def move_gripper(self, target_width, speed):
        # TODO, Radian I am too lazy to make this propely move
    
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

    def run_joint_trajectory(self, q_trajectory: np.array, dt: float):
        for q in q_trajectory:
            self.rtb_robot_model.q = q
            time.sleep(dt)

    def stop(self):
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