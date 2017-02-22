import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import scene_pb2
import irispy
import scipy
import matplotlib.animation as animation
import sys, os
import mpl_toolkits.mplot3d as a3
import os, sys
import subprocess

fig = plt.figure(figsize=(8,8), dpi=80)

ax = a3.Axes3D(fig)

ax.set_xlim([-2,2])
ax.set_ylim([-2,2])
ax.set_zlim3d([-2,2])

points = np.array([
    [0,0,0],
    [1,0,0],
    [1,1,0],
    [0,1,0],
    [0,0,1],
    [1,0,1],
    [1,1,1],
    [0,1,1],
    ])


artists = irispy.drawing.get_3d_convhull(points, ax)

for artist in artists:
    ax.add_collection3d(artist)

points[0,0] = -1

plt.savefig('000000.jpg')

plt.cla()

artists = irispy.drawing.get_3d_convhull(points, ax)

for artist in artists:
    ax.add_collection3d(artist)

plt.savefig('000001.jpg')

cmd = ['convert', '-delay', '100', '*.jpg', 'hhh.gif']
subprocess.call(cmd)


