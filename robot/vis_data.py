import numpy as np
import os
import matplotlib.pyplot as plt
import robot.config as cfg
from scipy.io import loadmat


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


def get_recent():
    results_folder = cfg.results_folder
    files = os.listdir(results_folder)
    files.sort()
    return os.path.join(results_folder, files[-1])

def vis_experiment(file_name=None):
    if file_name is None:
        file_name = get_recent()

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


if __name__ == '__main__':
    vis_experiment()
