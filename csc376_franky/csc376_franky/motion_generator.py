import copy
import numpy as np
import time
from typing import List, Tuple

import roboticstoolbox as rtb
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from scipy.spatial.transform import Rotation
from spatialmath import SE3

# Ruckig Motion Generator code taken mostly from: https://github.com/Patrick15Yao/franky_toolbox_bridge/blob/main/franky_toolbox_bridge/rtb_backend.py, 
# and tailored in a more calculate traj -> then visualize way, 
# also made more functional programming style. 
# Please credit Patrick Yao, github id Patrick15Yao for this work.

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
    
class RuckigMotionGenerator:
    def __init__(self):
        self.dt = 0.03
        self.ruckig = Ruckig(dofs=7, delta_time=self.dt)
        self.PANDA_VEL_LIM_CARTESIAN = np.array([3.0, 3.0, 3.0, 2.5, 2.5, 2.5, 2.62])   # m/s
        self.PANDA_ACC_LIM_CARTESIAN =  np.array([9.0, 9.0, 9.0, 17.0, 17.0, 17.0, 10.0]) # m/s²
        self.PANDA_JERK_LIM_CARTESIAN = np.array([4500.0, 4500.0, 4500.0, 8500.0, 8500.0, 8500.0, 5000.0]) # m/s³

    def calculate_cartesian_pose_trajectory(self,
                       se3_start : SE3,
                       se3_target: SE3, 
                       relative_vel_factor : float=0.02,
                       relative_acc_factor : float=0.01,
                       relative_jerk_factor: float=0.05) -> Tuple[List[List], float]: 
        """ Interpolates and creates a smooth cartesian trajectory (smooth vel, acc, jerk maybe) 
        starting from se3_start and ending at se3_target. The trajectory also follows
        the given cartesian constraints: relative_vel_factor, relative_acc_factor, and relative_jerk_factor
        which implicitly converts to a joint constraint :). 

        Parameters
        ----------
        se3_start:  SE3 
            The start end-effector pose of the trajectory. Usually the current ee pose of the robot.
        se3_target: SE3
            The end end-effector pose of the trajectory.
        relative_vel_factor : float
            0-1 multiplication factor to multiply PANDA_VEL_LIM_CARTESIAN.
            Limits the cartesian velocity during the entire trajectory.
        relative_acc_factor : float
            0-1 multiplication factor to multiply PANDA_ACC_LIM_CARTESIAN.
            Limits the cartesian acceleration during the entire trajectory.
        relative_jerk_factor: float
            0-1 multiplication factor to multiply PANDA_JERK_LIM_CARTESIAN.
            Limits the cartesian jerk during the entire trajectory.
        
        Returns
        ----------     
        cartesian_pose_traj: List[List] 
            List of a List: [[x, y, z, rx, ry, rz, elbow_angle], ... ]. 
            Cartesian pose trajectory of the end effector. 
        self.dt: float 
            Delta time between trajectory points of the cartsian_pose_traj.
        """
        
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
        inp.max_velocity     = self.PANDA_VEL_LIM_CARTESIAN * relative_vel_factor
        inp.max_acceleration = self.PANDA_ACC_LIM_CARTESIAN * relative_acc_factor
        inp.max_jerk         = self.PANDA_JERK_LIM_CARTESIAN * relative_jerk_factor
        inp.enabled = [True, True, True, True, True, True, True]
        res = Result.Working
    
        cartesian_pose_traj = []
        while res == Result.Working:
            res = self.ruckig.update(inp, out) # interpolates to robot vel, acc, and jerk constraints
            cartesian_pose_traj.append(out.new_position)
            out.pass_to_input(inp)
        return cartesian_pose_traj, self.dt

    def cartesian_pose_to_joint_trajectory(self, model, q_start: np.ndarray, cartesian_pose_traj: List[List]) -> List[np.ndarray]:
        """ Converts the cartesian pose trajectory to a joint trajectory using inverse kinematics
        from the model.
                
        Parameters
        ----------
        model: robotics toolbox model type 
            Robot model for computing IK.
        q_start : np.ndarray
            The robot's current joint positions to determine redundant solutions in the IK.
            The size is joint_dof of robot.
        cartesian_pose_traj:  List[List] 
            List of a List: [[x, y, z, rx, ry, rz, elbow_angle], ... ]. 
            Cartesian pose trajectory of the end effector. 

        Returns
        ----------
        q_traj: List[np.ndarray] 
            List of np.ndarray of joint positions with size of joint_dof of robot.
        """

        q_current = copy.deepcopy(q_start)
        q_traj = []
        for cartesian_pose in cartesian_pose_traj:
            q_current = rotation_vector_to_joint_config(
                model, q_current, normalize_rx_angle(cartesian_pose)) # convert to joint traj
            q_traj.append(q_current)
        return q_traj
