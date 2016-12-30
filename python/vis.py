import numpy as np

# num * dim
uavs = np.array([
        [-2. ,  2.],
        [-2. , -2.],
        [ 2. , -2.],
        [ 2. ,  2.]
    ])
# dim * num
sos = np.array([
        [5., 5., 8., 8.],
        [4., 1., 1., 4.]
    ])
dos = np.array([
        [10., 10., 11., 11.],
        [10.,  9.,  9., 10.]
    ])
# num * dim
uavsDir = np.array([
        [-2. , 12.],
        [-2. ,  8.],
        [ 2. ,  8.],
        [ 2. , 12.]
    ])

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import scene_pb2

fig = plt.figure(figsize=(8,8), dpi=80)

ax = fig.add_subplot(111)
ax.spines['right'].set_color('none')
ax.spines['top'].set_color('none')
ax.xaxis.set_ticks_position('bottom')
ax.spines['bottom'].set_position('zero')
ax.yaxis.set_ticks_position('left')
ax.spines['left'].set_position(('data',0))

x_min = -4
x_max = 14
y_min = -4
y_max = 14
ax.set_xlim(x_min,x_max), ax.set_xticks(np.linspace(x_min,x_max,x_max-x_min+1,endpoint=True))
ax.set_ylim(y_min,y_max), ax.set_yticks(np.linspace(y_min,y_max,y_max-y_min+1,endpoint=True))

uavRadius = 1
uavsAx = []
for pos in uavs:
    circle = plt.Circle(pos, uavRadius, color='r')
    uavsAx.append(circle)
    ax.add_artist(circle)

gDirRadius = 0.2
for pos in uavsDir:
    circle = plt.Circle(pos, gDirRadius, color='g')
    ax.add_artist(circle)

sos = np.array([
        [5., 5., 8., 8.],
        [4., 1., 1., 4.]
    ])
polygon = Polygon(sos.T, True, color='b')
patchSos = ax.add_patch(polygon)

dos = np.array([
        [10., 10., 11., 11.],
        [10.,  9.,  9., 10.]
    ])
polygon = Polygon(dos.T, True, color='y')
patchDos = ax.add_patch(polygon)


# Scatter plot
#scat = ax.scatter(uavs[:,0], uavs[:,1], s=100, lw = 0.5,
 #                 edgecolors = C, facecolors='None')

f = open('./scenes.txt', 'r')

    
def update(frame):
    global dos
#    print(dos)

    s = ''
    for line in f:
        if(line[:-1]=='###'):
            scene = scene_pb2.Scene()
            scene.ParseFromString(s[:-1])
            do = scene.dos.do[0]
            dos = np.zeros((len(do.point), 2))
            for i, point in enumerate(do.point):
                dos[i,0] = point.x
                dos[i,1] = point.y
#            dos = dos.T
            break;
        else:
            s += line

#    dos = patchDos.get_xy()
    temp_dos = dos
    print( "temp dos = ")
    print(temp_dos)
    temp_dos = temp_dos[:1] - (frame % 10)
    dos[:,1] = dos[:,1] - 1
    patchDos.set_xy(dos)

    for i, uavAx in enumerate(uavsAx):
        uavAx.center = uavs[i] + (frame % 5)
    #uavsAx[0].center = uavs[0] + (frame % 5)

    # Return the modified object
    return patchDos, uavsAx,

def init():
    pass

import matplotlib.animation as animation
animation = animation.FuncAnimation(fig, update, interval=100, blit=False, frames=range(10))
# animation.save('rain.gif', writer='imagemagick', fps=30, dpi=40)
    
f.close()
    
# Show result on screen
plt.show()

