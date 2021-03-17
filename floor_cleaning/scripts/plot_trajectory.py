import numpy as np
from plot_functions import plot_cleaning

path_to_file = '/home/user/catkin_ws/src/shsriniv_module4/floor_cleaning/scripts/'
trajectory = np.load(path_to_file+'trajectory.npy')
plot_cleaning(trajectory)