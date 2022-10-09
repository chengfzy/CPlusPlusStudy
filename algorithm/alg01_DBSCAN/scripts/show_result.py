"""
Read and show result from file

The file format is
X,Y,Z,ClusterID
"""

import argparse
from pathlib import Path
import logging
import coloredlogs
import numpy as np
import matplotlib.pyplot as plt

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Ultrasonic RADAR Receiver')
    parser.add_argument('--file',
                        default='./algorithm/alg01_DBSCAN/data/benchmark_hepta.dat',
                        help='file name for result(default: %(default)s)')
    args = parser.parse_args()
    print(args)

    # config logging
    logging.basicConfig(level=logging.INFO)
    coloredlogs.install(fmt="[%(asctime)s %(levelname)s %(filename)s:%(lineno)d] %(message)s")

    # read file
    file = args.file
    logging.info(f'read data from file {file}')
    data = np.loadtxt(file, comments='#', delimiter=',')

    # plot
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_title('DBSCAN')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.scatter(data[:, 0], data[:, 1], data[:, 2], c=data[:, 3], cmap='magma')
    plt.show()
