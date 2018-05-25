from robot.vis_data import get_recent, triple_plot, triple_plot2
from scipy.io import loadmat
import matplotlib.pyplot as plt

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