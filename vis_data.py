import numpy as np
from scipy.io import loadmat
import matplotlib
import matplotlib.pyplot as plt
from utils import load_8_traj


def triple_plot(data, title):
    x = np.arange(len(data))
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.step(x, data[:, 0])
    plt.grid(True)
    # ax.grid(color='k', linestyle='--', linewidth=1)
    ax = fig.add_subplot(312)
    ax.step(x, data[:, 1])
    plt.grid(True)
    ax = fig.add_subplot(313)
    ax.step(x, data[:, 2])
    fig.suptitle(title)
    plt.grid(True)


def triple_plot2(data1, data2, title1, title2):
    x = np.arange(len(data1))
    fig = plt.figure()
    ax = fig.add_subplot(311)
    ax.step(x, data1[:, 0], label=title1)
    ax.step(x, data2[:, 0], label=title2)
    plt.legend()
    plt.grid(True)
    # ax.grid(color='k', linestyle='--', linewidth=1)
    ax = fig.add_subplot(312)
    ax.step(x, data1[:, 1])
    ax.step(x, data2[:, 1])
    plt.legend()
    plt.grid(True)
    ax = fig.add_subplot(313)
    ax.step(x, data1[:, 2])
    ax.step(x, data2[:, 2])
    # fig.suptitle(title)
    plt.legend()
    plt.grid(True)


if __name__ == '__main__':

    # file_name = 'data/2017-10-06-18-21-03_controller3_sim.mat'
    # file_name = 'data/2017-11-21-19-24-30_controller3_sim.mat'
    file_name = 'data/2017-11-21-19-28-00_controller3_sim.mat'

    data = loadmat(file_name)
    control = data['control_signal']
    states = data['states']
    wheels = data['wheels_vel']
    pose = data['pose']
    ref = data['reference']

    # triple_plot(states, 'States')
    triple_plot(control, 'Control Signals')
    triple_plot(wheels, 'Wheels Velocities')
    # triple_plot(ref, 'reference')
    triple_plot2(states, ref, 'states', 'reference')

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(pose[:, 0], pose[:, 1])
    fig.suptitle('Position')
    plt.grid(True)

    plt.show()
