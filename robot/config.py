import os

# Robot Params
Ts = 0.06
r = 0.0505
b = 0.1

# Project Folders
results_folder = 'results'
data_folder = 'data'
controllers_folder = 'controllers'
state_space_filepath = os.path.join(data_folder, 'sys_data.mat')