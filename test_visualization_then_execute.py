
import copy
import numpy as np
import time

from spatialmath import SE3
import roboticstoolbox as rtb
from ruckig import InputParameter, OutputParameter, Result, Ruckig

PANDA_VEL_LIM_CARTESIAN = np.array([3.0, 3.0, 3.0, 2.5, 2.5, 2.5, 2.62])   # m/s
PANDA_ACC_LIM_CARTESIAN =  np.array([9.0, 9.0, 9.0, 17.0, 17.0, 17.0, 10.0]) # m/s²
PANDA_JERK_LIM_CARTESIAN = np.array([4500.0, 4500.0, 4500.0, 8500.0, 8500.0, 8500.0, 5000.0]) # m/s³

# Ruckig is used because the internal franky uses ruckig to generate trajectories, and we
# want to visualize the franka robot's trajectory in swift before running on the real robot. 
# HOWEVER, the rucking parameters might not be the same so trajectories might slightly be different in
# sim and in real
class RuckigMotionGenerator:
    def __init__(self):

        self.dt = 0.05
        self.ruckig = Ruckig(dofs=7, delta_time=self.dt)

    def cartesian_pose(self, model,
                       q_start: np.array, 
                       se3_target: SE3, 
                       rv = 0.02, ra=0.01, rj=0.05):
        se3_start   = model.fkine(q_start)
        RV_start    = self.se3_to_rotation_vector(se3_start, q_start[2])
        RV_target   = self.se3_to_rotation_vector(se3_target, q_start[2])
        N_RV_start  = self.normalize_rx_angle(RV_start)
        N_RV_target = self.normalize_rx_angle(RV_target)

        inp = InputParameter(7)
        out = OutputParameter(7)
        inp.current_position = N_RV_start
        inp.current_velocity = [0,0,0,0,0,0,0]
        inp.current_acceleration = [0,0,0,0,0,0,0]
        inp.target_position = N_RV_target
        inp.target_velocity = [0,0,0,0,0,0,0]
        inp.target_acceleration = [0,0,0,0,0,0,0]
        inp.max_velocity = PANDA_VEL_LIM_CARTESIAN * rv
        inp.max_acceleration = PANDA_ACC_LIM_CARTESIAN * ra
        inp.max_jerk = PANDA_JERK_LIM_CARTESIAN * rj
        inp.enabled = [True, True, True, True, True, True, True]
        res = Result.Working
        last_iteration_time = time.time()
    
        q_new = copy.deepcopy(q_start)
        q_traj = []
        while res == Result.Working:
            # iteration_start_time = time.time()
            res = self.ruckig.update(inp, out)
            RV_new = out.new_position
            N_RV_new = self.normalize_rx_angle(RV_new)
            q_new = self.rotation_vector_to_joint_config(N_RV_new)
            q_traj.append(q_new)
            out.pass_to_input(inp)
            # computation_time = time.time() - iteration_start_time
            # sleep_time = max(0, self.dt - computation_time)
            # current_time = time.time()
            # actual_dt = current_time - last_iteration_time
            # last_iteration_time = current_time
            
        return q_traj

def main():
    rmg = RuckigMotionGenerator()
    panda_rtb_model = rtb.models.Panda()
    panda_rtb_model.q = panda_rtb_model.qr
    se3_target = SE3(0.5, 0.5, 0.5) * SE3.RPY(0, 0, 0)

    traj = rmg.cartesian_pose(panda_rtb_model, panda_rtb_model.q, se3_target)
    print(traj)

if __name__ == "__main__":
    main()
