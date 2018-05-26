import time
from scipy.io import loadmat, savemat
import numpy as np
import os
import robot.config as cfg


def now():
    """
    Returns system current time (until seconds)
    """
    return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())


def wheel2states(wheel_vel):
    """Convert an array of wheel speeds (rad/s) to the defined states"""
    wheel_vel = wheel_vel.reshape(3, 1)
    r = cfg.r
    b = cfg.b

    TM = np.array([[0, np.sqrt(3) * r / 3.0, -np.sqrt(3) * r / 3.0],
                   [-2.0 * r / 3.0, r / 3.0, r / 3.0],
                   [r / (3.0 * b), r / (3.0 * b), r / (3.0 * b)]])
    states = TM.dot(wheel_vel)

    return states


def load_controler(file_path, pprint=False):
    controller = loadmat(file_path)
    if pprint:
        print('Controller Loaded:')
        print(controller)
    return controller['K']


def make_ref(ref, N):
    return np.tile(ref, (N, 1))


def make_inv_ref(ref1, ref2,  N):
    a = np.tile(ref1, (int(N/2), 1))
    b = np.tile(ref2, (int(N/2), 1))

    return np.vstack((a, b))


class DataLogger:

    def __init__(self, data_folder=cfg.results_folder):
        N = 5000
        self.control_signal_vec = np.empty(shape=(N, 3))
        self.states_vec = np.empty(shape=(N, 3))
        self.wheels_vel_vec = np.empty(shape=(N, 3))
        self.pose_vec = np.empty(shape=(N, 3))
        self.ref_vec = np.empty(shape=(N, 3))
        self.iter = 0
        self.data_folder = data_folder

    def update(self, control_signal, states, wheels_vel, pose, ref):

        i = self.iter
        self.control_signal_vec[i, :] = control_signal.flatten()
        self.states_vec[i, :] = states.flatten()
        self.wheels_vel_vec[i, :] = wheels_vel.flatten()
        self.pose_vec[i, :] = pose.flatten()
        self.ref_vec[i, :] = ref.flatten()
        self.iter += 1

    def close(self, suffix):
        i = self.iter
        test_name = os.path.join(self.data_folder, now() + '_' + suffix)
        savemat(test_name, {'states': self.states_vec[:i],
                            'wheels_vel': self.wheels_vel_vec[:i],
                            'control_signal': self.control_signal_vec[:i],
                            'reference': self.ref_vec[:i],
                            'pose': self.pose_vec[:i]
                            })


def update_pose(states, pose_old, Ts):

    d = states[0]*Ts
    dn = states[1]*Ts
    dtheta = states[2]*Ts

    xm_ = pose_old[0]
    ym_ = pose_old[1]
    theta_ = pose_old[2]

    theta = theta_ + dtheta

    if dtheta == 0:
        xm = xm_ + d*np.cos(theta_) - dn*np.sin(theta_)
        ym = ym_ + d*np.sin(theta_) + dn*np.cos(theta_)

    else:
        xsin = np.sin(theta_ + dtheta/2.0)/dtheta
        xcos = np.cos(theta_ + dtheta/2.0)/dtheta

        xm = xm_ + (d*np.sin(dtheta) + dn*(np.cos(dtheta) - 1))*xcos\
            - (d*(1-np.cos(dtheta)) + dn*np.sin(dtheta))*xsin
        ym = ym_ + (d*np.sin(dtheta) + dn*(np.cos(dtheta) - 1))*xsin\
            + (d*(1-np.cos(dtheta)) + dn*np.sin(dtheta))*xcos

    pose = np.array([xm, ym, theta])

    return pose


class Traj:
    def __init__(self, traj, v_nav, radius=0.05):
        self.traj = traj
        self.ind = 0
        self.v_nav = v_nav
        self.radius = radius

    def update(self, pose):

        next_point = self.traj[self.ind, :]
        d = np.sqrt((next_point[0] - pose[0])**2+(next_point[1] - pose[1])**2)
        if d < self.radius:
            self.ind = self.ind + 1
            if self.ind >= self.traj.shape[0]:
                return None
            else:
                next_point = self.traj[self.ind, :]

        xr = pose[0]
        yr = pose[1]
        theta = pose[2]
        R = np.array([
            [np.cos(theta), np.sin(theta), 0],
            [-np.sin(theta), np.cos(theta), 0],
            [0, 0, 1]
            ])
        phi = np.arctan2(next_point[1] - yr, next_point[0] - xr)
        e = np.array([self.v_nav*np.cos(phi),
                      self.v_nav*np.sin(phi),
                      next_point[2] - theta])
        ref = R.dot(e)

        return ref


def load_circle_J_traj():
    """Circle Trajectory used by Jessivaldo on his work."""
    return loadmat('trajs/circle_traj_J.mat')['traj']


def load_square_traj():
    """One mark in each vertice"""
    traj = np.array([
                    [0, 0, 0],
                    [1, 0, 0],
                    [1, 1, 0],
                    [0, 1, 0],
                    [0, 0, 0]
                    ])
    return traj


def load_square_traj2():
    """One meter side, 20cm spaced marks"""
    traj = np.array([
                    [0, 0, 0],
                    [0.2, 0, 0],
                    [0.4, 0, 0],
                    [0.6, 0, 0],
                    [0.8, 0, 0],
                    [1, 0, 0],
                    [1, 0.2, 0],
                    [1, 0.4, 0],
                    [1, 0.6, 0],
                    [1, 0.8, 0],
                    [1, 1, 0],
                    [0.8, 1, 0],
                    [0.6, 1, 0],
                    [0.4, 1, 0],
                    [0.2, 1, 0],
                    [0, 1, 0],
                    [0, 0.8, 0],
                    [0, 0.6, 0],
                    [0, 0.4, 0],
                    [0, 0.2, 0],
                    [0, 0, 0]
                    ])
    return traj


def load_8_traj():

    angles = np.array([0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165,
                       180])
    angles1 = np.hstack((angles, np.flip(-angles[:-1], 0)))
    angles1 = np.pi/180*angles1
    angles2 = np.hstack((np.flip(angles, 0), -angles[1:]))
    angles2 = np.pi/180 * angles2
    circle1 = np.zeros(shape=(len(angles1), 3))
    circle1[:, 0] = np.cos(angles1)
    circle1[:, 1] = np.sin(angles1)

    circle2 = np.zeros(shape=(len(angles2), 3))
    circle2[:, 0] = np.cos(angles2)
    circle2[:, 1] = np.sin(angles2)

    circle1 = circle1 + np.array([1, 0, 0])
    circle2 = circle2 + np.array([3, 0, 0])
    traj = np.vstack((np.array([0, 0, 0]), circle2[0:-1], circle1))

    # traj = np.array([
    #                 [0, 0, 0],
    #                 [2, 0, 0],
    #                 [2 + np.sqrt(2)/2, np.sqrt(2)/2, 0],
    #                 [3, 1, 0],
    #                 [3 + np.sqrt(2)/2, np.sqrt(2)/2, 0],
    #                 [4, 0, 0],
    #                 [3 + np.sqrt(2)/2, - np.sqrt(2)/2, 0],
    #                 [3, -1, 0],
    #                 [2 + np.sqrt(2)/2, - np.sqrt(2)/2, 0],
    #                 [2, 0, 0],
    #                 [1 + np.sqrt(2)/2, np.sqrt(2)/2, 0],
    #                 [1, 1, 0],
    #                 [np.sqrt(2)/2, np.sqrt(2)/2, 0],
    #                 [0, 0, 0],
    #                 [np.sqrt(2)/2, - np.sqrt(2)/2, 0],
    #                 [1, -1, 0],
    #                 [1 + np.sqrt(2)/2, - np.sqrt(2)/2, 0],
    #                 [2, 0, 0]
    #                 ])
    return traj


def load_8_traj2():

    angles = np.array([0, 15, 30, 45, 60, 75, 90, 105, 120, 135, 150, 165,
                       180])
    angles1 = np.hstack((angles, -angles[-2::-1]))
    angles1 = np.pi/180*angles1
    angles2 = np.hstack((angles[::-1], -angles[1:]))
    angles2 = np.pi/180 * angles2
    circle1 = np.zeros(shape=(len(angles1), 3))
    circle1[:, 0] = np.cos(angles1)
    circle1[:, 1] = np.sin(angles1)

    circle2 = np.zeros(shape=(len(angles2), 3))
    circle2[:, 0] = np.cos(angles2)
    circle2[:, 1] = np.sin(angles2)

    circle1 = circle1 + np.array([1, 0, 0])
    circle2 = circle2 + np.array([3, 0, 0])
    traj = np.vstack((np.array([0, 0, 0]), circle2[0:-1], circle1))

    return traj


def make_S_traj():
    reference = np.empty(shape=(300, 3))
    reference[:50] = np.tile([0.6, 0, 0], (50, 1))
    reference[50:150] = np.tile([0.6, 0, 0.6], (100, 1))
    reference[150:250] = np.tile([0.6, 0, -0.6], (100, 1))
    reference[250:] = np.tile([0.6, 0, 0], (50, 1))
    return reference


def make_quad_traj():
    reference = np.empty(shape=(200, 3))
    reference[:50] = np.tile([0.6, 0, 0], (50, 1))
    reference[50:100] = np.tile([0, 0.6, 0], (50, 1))
    reference[100:150] = np.tile([-0.6, 0, 0], (50, 1))
    reference[150:] = np.tile([0, -0.6, 0], (50, 1))
    return reference


if __name__ == '__main__':

    # savemat('traj8', {'traj8': load_8_traj2()})

    # savemat('traj_quad', {'traj_quad': load_square_traj2()})

    print(load_circle_J_traj())
    # print(now())
    # import matplotlib.pyplot as plt
    # traj = load_8_traj2()
    # plt.scatter(traj[:, 0], traj[:, 1])
    # for i in range(len(traj)):
    #     plt.text(traj[i, 0], traj[i, 1], str(i))
    # plt.show()
    #
    # traj = load_8_traj2()
    # print(traj)
