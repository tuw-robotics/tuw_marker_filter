'''
Created on Jul 25, 2016

@author: Markus Bader
'''

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.pyplot import imshow, pause
from tuw.vehicle_plot import VehiclePlot
# from tuw.ekf_slam import Vehicle
from tuw.particle_filter_localization import Vehicle
import time

fig, ax = plt.subplots()
plt.draw()
plt.ion()
ax.axis('equal')
ax.grid(True)
ax.set_xticks(np.arange(-10, 10, 1))
ax.set_yticks(np.arange(-10, 10, 1))
ax.set_xlim([-12, 12])
plt.xlabel('x')
ax.set_ylim([-12, 12])
plt.ylabel('y')
plt.show()


def loop(vehicle, filename):
    skip = 10
    loop_counter = 0
    with open(filename) as f:
        for line in f:
            loop_counter = loop_counter + 1
            elements = line.split(':')
            header = elements[0].strip()
            print("processing: {}".format(header))
            if ('odom' == header):
                pose = np.matrix(list(map(float, elements[1].split(",")))).reshape(3, -1)
                vehicle.set_odom(pose)
            if ('true_pose' == header):
                data = np.matrix(list(map(float, elements[1].split(","))))
                vehicle.PoseArrowTruePose.set_pose(data)
                vehicle.PoseArrowTruePose.set_zorder(len(vehicle.vehicle.samples))
            if ('cmd' == header):
                data = np.matrix(list(map(float, elements[1].split(",")))).reshape(2, -1)
                vehicle.prediction(data)
            if ('marker' == header):
                t = np.matrix(map(int, elements[1].split(",")))
                z = np.matrix(list(map(float, elements[2].split(",")))).reshape(-1, 4)
                for i in range(z.shape[0]):  # Fix for wrong log file
                    if z[i, 0] > 0:
                        z[i, 0] = z[i, 0] - 1
                vehicle.measurments(z)
                # print (line)
            if ('map' == header):
                t = np.matrix(map(int, elements[1].split(",")))
                m = np.matrix(list(map(float, elements[2].split(",")))).reshape(-1, 4)
                vehicle.define_map(m)
            if (loop_counter % skip) == 0:
                pass
                # plt.pause(0.001)
                # print (loop_counter)
            fig.canvas.draw()


if __name__ == '__main__':
    vehicle = VehiclePlot(ax, Vehicle())
    loop(vehicle, "log01.txt")

    print ("Good-by")
