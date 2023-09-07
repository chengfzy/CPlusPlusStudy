# Plot the results from the 2D pose graph optimization.
# It will draw a line between consecutive vertices, the command line expects two optional filenames:
#
#   ./plot_slam_2d.py --initial_poses file1 --optimized_poses file2 --ground_truth file3
#
# The initial pose and optimized pose file have the following format:
#   ID x y yaw
# The ground truth pose file have the following format:
#   x y yaw

import numpy as np
import matplotlib.pyplot as plot
from optparse import OptionParser

parser = OptionParser()
parser.add_option('--initial_poses',
                  dest='initial_poses',
                  default='./poses_2d_original.txt',
                  help='the filename that contains the original poses')
parser.add_option('--optimized_poses',
                  dest='optimized_poses',
                  default='./poses_2d_optimized.txt',
                  help='the filename that contains the optimized poses')
parser.add_option('--ground_truth',
                  dest='ground_truth',
                  default='./ceres/data/manhattan/groundTruth/manhattanOlson3500_nodes_groundTruth.dat',
                  help='the filename that contains the ground truth poses')
(options, args) = parser.parse_args()

# read the original and optimized poses files
pose_original = None
if options.initial_poses != '':
    pose_original = np.genfromtxt(options.initial_poses, usecols=(1, 2, 3))
pose_optimized = None
if options.optimized_poses != '':
    pose_optimized = np.genfromtxt(options.optimized_poses, usecols=(1, 2, 3))
ground_truth = None
if options.ground_truth != '':
    ground_truth = np.genfromtxt(options.ground_truth, usecols=(0, 1, 2))

# plot the results in XY plane
fig = plot.figure('Pose Graph 2D')
ax = fig.add_subplot(111)
if pose_original is not None:
    ax.plot(pose_original[:, 0], pose_original[:, 1], '-', label='Original', color='green')
if pose_optimized is not None:
    ax.plot(pose_optimized[:, 0], pose_optimized[:, 1], '-', label='Optimized', color='blue')
if ground_truth is not None:
    ax.plot(ground_truth[:, 0], pose_optimized[:, 1], '-', label='Ground Truth', color='red')
ax.axis('equal')
ax.set_title('Pose Graph 2D')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.legend(loc='best', shadow=True)
plot.show()
