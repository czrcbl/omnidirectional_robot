from scipy.integrate import odeint
from scipy.io import loadmat
import numpy as np
from serial_com import RecvData
import config as cfg


class SimulationAdapter:

    def __init__(self, lti_file_path, Ts):
        self.sim = Simulation(lti_file_path, Ts)

    def send_control_signal(self, control_signal):
        self.sim.simulation_step(control_signal)

    def receive_message(self):
        wheels_vel = self.sim.wheel_vel
        data = RecvData()
        data.m1_vel = wheels_vel[0]
        data.m2_vel = wheels_vel[1]
        data.m3_vel = wheels_vel[2]

        return data

    def init_serial(self):
        print('Simulation Initialized!')


class LTISystem:
    pass


def system(y, t, u, sys):
    y = y.reshape(3, 1)
    u = u.reshape(3, 1)
    dydt = sys.A.dot(y) + sys.B.dot(u)
    dydt = dydt.flatten()  # makes dydt one dimensional

    return dydt


def load_lti_system(lti_file_path):
    data_dict = loadmat(lti_file_path)
    sys = LTISystem()
    sys.A = data_dict['Ac']
    sys.B = data_dict['Bc']
    sys.C = data_dict['Cc']
    sys.D = data_dict['Dc']
    return sys


class Simulation:

    def __init__(self, lti_file_path, Ts):
        self.Ts = Ts
        self.y0 = np.zeros(3)
        self.wheel_vel = np.zeros(3)
        self.sys = load_lti_system(lti_file_path)

    def simulation_step(self, u):
        t = [0, self.Ts]
        y1 = odeint(system, self.y0, t, args=(u, self.sys))
        y0 = y1[1, :]
        self.y0 = y0

        b = cfg.b
        r = cfg.r
        M = np.array([[0, -1, b],
                      [np.sqrt(3)/2, 1/2, b],
                      [-np.sqrt(3)/2, 1/2, b]])/r
        y0 = y0.reshape(3, 1)
        wheel_vel = M.dot(y0)
        self.wheel_vel = wheel_vel


if __name__ == '__main__':

    sim = Simulation('sys_data.mat', 0.05)
    y0 = sim.simulation_step(np.ones(3))
