import pytest
import math
from ros_sim.sim import JointPosCtrlParams, PositionJointController

import matplotlib.pyplot as plt


def test_joint_pos_controller():
    params = JointPosCtrlParams()

    ctrl = PositionJointController(params)

    cur_joint_pos = [0.0 for i in range(params.n_dof)]
    target_joint_pos = [math.pi/2 for i in range(params.n_dof)]
    joint_vel = [0.0 for i in range(params.n_dof)]

    cur_time = 0.0

    joint_pos = []
    times = []
    for i in range(params.n_dof):
        joint_pos.append([])
    
    for i in range(int(1.0/ params.dt)):

        torques = ctrl.get_torques(target_joint_pos, cur_joint_pos)

        for j in range(len(torques)):
            cur_joint_pos[j] += joint_vel[j] * params.dt
            joint_vel[j] += torques[j] * params.dt
            joint_pos[j].append(cur_joint_pos[j])
        
        cur_time += params.dt
        times.append(cur_time)

    do_plot = False

    if do_plot:
        joint_to_plot = 0
        fig, ax = plt.subplots()

        ax.plot(times, joint_pos[joint_to_plot])
        ax.hlines(target_joint_pos[joint_to_plot], xmin=times[0], xmax=times[-1])

        plt.show()

    for i in range(params.n_dof):
        cur_pos = cur_joint_pos[i]
        target = target_joint_pos[i]
        assert abs(cur_pos - target) < 0.1, f"joint {i}, cur_pos: {cur_pos}, taget: {target}"
