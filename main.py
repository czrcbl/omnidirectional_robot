import numpy as np
import os
import time
from serial_com import RobotCom
from simulation import SimulationAdapter
from utils import (DataLogger, update_pose, load_square_traj, Traj,
                   make_S_traj, make_quad_traj, load_square_traj2, load_8_traj,
                   load_8_traj2, load_controler, make_ref, make_inv_ref,
                   load_circle_J_traj)


def wheel2states(wheel_vel):
    """Convert an array of wheel speeds (rad/s) to the defined states"""
    wheel_vel = wheel_vel.reshape(3, 1)
    r = 0.0505
    b = 0.1

    TM = np.array([[0, np.sqrt(3) * r / 3.0, -np.sqrt(3) * r / 3.0],
                   [-2.0 * r / 3.0, r / 3.0, r / 3.0],
                   [r / (3.0 * b), r / (3.0 * b), r / (3.0 * b)]])
    states = TM.dot(wheel_vel)

    return states


def main():
    global com

    Ts = 0.06
    time.sleep(5)
    controller = 'controller1'

    is_ref = False
    if is_ref:
        # reference = make_ref([0.6, 0.0, 0], 100)
        # reference = make_ref([0.0, 0.6, 0], 40)
        # reference = make_ref([0.0, 0.0, 2], 40)
        # reference = make_S_traj()
        # reference = make_quad_traj()
        reference = make_inv_ref([0.6, 0.0, 0], [-0.6, 0.0, 0], 100)
    else:
        # traj = Traj(load_square_traj(), 0.6)
        # traj = Traj(load_square_traj2(), 0.6)
        traj = Traj(load_square_traj2(), 0.3)
        # traj = Traj(load_8_traj2(), 0.6)
        # traj = Traj(load_8_traj2(), 0.4)
        # traj = Traj(load_8_traj2(), 0.3)
        # traj = Traj(load_square_traj(), 0.3)

        # traj = Traj(load_circle_J_traj(), 0.3)

    # Init Serial
    try:
        com = RobotCom()
        suffix = controller + '_test'
    except Exception as e:
        print(e)
        suffix = controller + '_sim'
        com = SimulationAdapter('sys_data.mat', Ts)

    # Init Variables
    K = load_controler(os.path.join('controllers', controller + '.mat'))
    aug_states_old = np.zeros(shape=(6, 1))
    aug_states = np.zeros(shape=(6, 1))
    control_signal = np.zeros(shape=(3, 1))
    pose_old = np.zeros(3)
    logger = DataLogger()

    com.init_serial()

    i = 0
    while True:

        print('Sample {}'.format(i))

        data = com.receive_message()
        wheels_vel = np.array([data.m1_vel, data.m2_vel, data.m3_vel])
        print('wheels_vel', wheels_vel)

        tic = time.time()

        states = wheel2states(wheels_vel)
        print('States', states)

        pose = update_pose(states, pose_old, Ts)
        print('pose', pose)

        if is_ref:
            try:
                ref = reference[i, :]
            except IndexError:
                ref = None
        else:
            ref = traj.update(pose)

        if ref is None:
            break

        aug_states[:3] = states - ref.reshape(3, 1)
        aug_states[3:] = aug_states_old[3:] + aug_states_old[:3]

        control_signal = K.dot(aug_states)
        print('Control Signal:', control_signal)

        print('delay', time.time() - tic)

        com.send_control_signal(control_signal)

        try:
            logger.update(control_signal, states, wheels_vel, pose, ref)
        except IndexError:
            break

        aug_states_old = aug_states
        pose_old = pose
        i += 1

    com.send_control_signal(np.array([0, 0, 0]))
    logger.close(suffix)


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        com.send_control_signal(np.array([0, 0, 0]))
        raise e

