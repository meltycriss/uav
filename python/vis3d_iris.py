import irispy
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import os, sys, shutil
import random
import mpl_toolkits.mplot3d as a3

def toCube(coord, radius):
    cube = np.empty((0,3))
    for i in [-radius,radius]:
        for j in [-radius,radius]:
            for z in [-radius,radius]:
                temp_coord = np.copy(coord)
                temp_coord[0]+=i
                temp_coord[1]+=j
                temp_coord[2]+=z
                cube = np.insert(cube, cube.shape[0], temp_coord, 0)
    return cube

# draw convex hull
def draw(convhull, ax, **kwargs):
    irispy.drawing.draw_3d_convhull(convhull, ax, **kwargs)

def plot_obstacle(point, side_length, *args, **kwargs):
    global ax
    ax.add_patch(patches.Rectangle((point[0], point[1]), side_length, side_length, *args, **kwargs))


outputFile = 'iris_res'
if(len(sys.argv)==2):
    outputFile = sys.argv[1]
if os.path.isdir(outputFile):
    shutil.rmtree(outputFile)
os.mkdir(outputFile)

ratio = 1.1
side_length = 1
x_min = -10
x_max = 10
y_min = -10
y_max = 10
z_min = -10
z_max = 10
numObs = 5
obstacles = []
for i in range(numObs):
    pos = np.array([
        random.uniform(x_min+0.5*side_length, x_max-0.5*side_length), 
        random.uniform(y_min+0.5*side_length, y_max-0.5*side_length), 
        random.uniform(z_min+0.5*side_length, z_max-0.5*side_length)
        ])
    obs = toCube(pos, 0.5*side_length)
    obs = obs.T
    obstacles.append(obs)

bounds = irispy.Polyhedron.fromBounds([x_min,y_min,z_min], [x_max,y_max,z_max])
seed_point = np.array([0.0, 0.0, 0.0])

region, info = irispy.inflate_region(obstacles, seed_point, bounds, return_debug_data=True)

fig = plt.figure(figsize=(8,8), dpi=80)
ax = a3.Axes3D(fig)
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_zlim3d(z_min, z_max)
#ax.w_xaxis.set_pane_color((1,1,1,0))
#ax.w_yaxis.set_pane_color((1,1,1,0))
#ax.w_zaxis.set_pane_color((1,1,1,0))
#ax.w_xaxis.line.set_color((1,1,1,0))
#ax.w_yaxis.line.set_color((1,1,1,0))
#ax.w_zaxis.line.set_color((1,1,1,0))
#ax.set_xticks([])
#ax.set_yticks([])
#ax.set_zticks([])

for obs in obstacles:
    draw(obs.T, ax)

info.polyhedron_history[2].draw(ax=ax)
info.ellipsoid_history[2].draw(ax=ax)

#plt.gca().set_xlim(x_min, x_max)
#plt.gca().set_ylim(y_min, y_max)
#plt.gca().set_zlim3d(z_min, z_max)

plt.show()
#
#for i in range(2*len(info.polyhedron_history)):
#    ax.clear()
#    ax.axis('off')
#    ax.set_xlim([ratio*x_min, ratio*x_max])
#    ax.set_ylim([ratio*y_min, ratio*y_max])
#    bounds.draw(edgecolor='r')
#    plt.scatter([0], [0], color='g')
#    for obs in obstacles:
#        plot_obstacle(obs.T[0], side_length, facecolor='grey', edgecolor='none')
#    if(i%2==0):
#        info.ellipsoid_history[i/2].draw(edgecolor='b')
#        if(i/2>0):
#            info.polyhedron_history[i/2-1].draw(edgecolor='g')
#    else:
#        info.polyhedron_history[i/2].draw(edgecolor='g')
#        if(i/2>0):
#            info.ellipsoid_history[i/2].draw(edgecolor='b')
#    filename = '%06d' % i
#    outputPath = os.path.join(outputFile, filename)
#    plt.savefig(outputPath, dpi=fig.dpi)
