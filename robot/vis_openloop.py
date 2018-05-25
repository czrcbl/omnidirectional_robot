import numpy as np
from scipy.io import loadmat
import matplotlib
import matplotlib.pyplot as plt
from robot.vis_data import triple_plot
from robot.tests.tests0 import load_p
matplotlib.style.use('ggplot')

if __name__ == '__main__':

    file_name = 'openloop/2017-08-22-16-49-58_normal.mat'

    data = loadmat(file_name)
    control = data['control']
    states = data['states']
    wheels = data['wheel_vel']

    triple_plot(states, 'States')
    triple_plot(control, 'Control Signals')
    triple_plot(wheels, 'Wheels Velocities')

    plt.show()
