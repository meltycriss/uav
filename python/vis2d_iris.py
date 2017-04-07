import irispy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os, sys, shutil
import random
from matplotlib.ticker import NullLocator

def plot_obstacle(point, side_length, *args, **kwargs):
    global ax
    ax.add_patch(patches.Rectangle((point[0], point[1]), side_length, side_length, *args, **kwargs))

outputFile = 'iris_res'
if(len(sys.argv)==2):
    outputFile = sys.argv[1]
if os.path.isdir(outputFile):
    shutil.rmtree(outputFile)
os.mkdir(outputFile)

ratio = 1.01
side_length = 1
x_min = -10
x_max = 10
y_min = -10
y_max = 10
numObs = 20
obstacles = []
for i in range(numObs):
    pos = np.array([random.uniform(x_min+0.5*side_length, x_max-0.5*side_length), random.uniform(y_min+0.5*side_length, y_max-0.5*side_length)])
    obs = np.ones((4,2))
    obs[0] = pos + np.array([-0.5*side_length, -0.5*side_length])
    obs[1] = pos + np.array([0.5*side_length, -0.5*side_length])
    obs[2] = pos + np.array([0.5*side_length, 0.5*side_length])
    obs[3] = pos + np.array([-0.5*side_length, 0.5*side_length])
    obs = obs.T
    obstacles.append(obs)

bounds = irispy.Polyhedron.fromBounds([x_min,y_min], [x_max,y_max])
seed_point = np.array([0.0, 0.0])

region, info = irispy.inflate_region(obstacles, seed_point, bounds, return_debug_data=True)

fig = plt.figure(figsize=(8,8), dpi=80)
ax = plt.gca()

for i in range(2*len(info.polyhedron_history)):
    ax.clear()
    ax.axis('off')
    ax.xaxis.set_major_locator(NullLocator())
    ax.yaxis.set_major_locator(NullLocator())
    ax.set_xlim([ratio*x_min, ratio*x_max])
    ax.set_ylim([ratio*y_min, ratio*y_max])
    bounds.draw(edgecolor='r')
    plt.scatter([0], [0], color='g')
    for obs in obstacles:
        plot_obstacle(obs.T[0], side_length, facecolor='grey', edgecolor='none')
    if(i%2==0):
        info.ellipsoid_history[i/2].draw(edgecolor='b')
        if(i/2>0):
            info.polyhedron_history[i/2-1].draw(edgecolor='g')
    else:
        info.polyhedron_history[i/2].draw(edgecolor='g')
        if(i/2>0):
            info.ellipsoid_history[i/2].draw(edgecolor='b')
    filename = '%06d' % i
    outputPath = os.path.join(outputFile, filename)
    #plt.show()
    plt.savefig(outputPath, dpi=fig.dpi, bbox_inches='tight', pad_inches=0)
