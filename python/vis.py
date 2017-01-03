import numpy as np

# num * dim
uavs = np.array([
        [-2. ,  2.],
        [-2. , -2.],
        [ 2. , -2.],
        [ 2. ,  2.]
    ])
# num * dim
sos = [np.array([
        [5., 4.],
        [5., 1.],
        [8., 1.],
        [8., 4.]
    ])]
dos = [np.array([
        [10., 10.],
        [10., 9. ],
        [11., 9. ],
        [11., 10.]
    ])]
# num * dim
uavsDir = np.array([
        [-2. , 12.],
        [-2. ,  8.],
        [ 2. ,  8.],
        [ 2. , 12.]
    ])
# a
a = np.array([[          1, 2.02739e-05],
  [          1,           0],
  [          0,           1],
  [         -1,          -0],
  [         -0,          -1],
  [   0.994616,   -0.103633],
  [          1,           0],
  [          0,           1],
  [         -1,          -0],
  [         -0,          -1]])
# b
b = np.array([[3.99996],
  [  10000],
  [  10000],
  [  10000],
  [  10000],
  [3.46031],
  [  10000],
  [  10000],
  [  10000],
  [  10000]])

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import scene_pb2
import irispy
import scipy

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

# uavs
uavRadius = 1
uavsAx = []
for pos in uavs:
    circle = plt.Circle(pos, uavRadius, color='r')
    uavsAx.append(circle)
    ax.add_artist(circle)
# uavsDir
gDirRadius = 0.2
uavsDirAx = []
for pos in uavsDir:
    circle = plt.Circle(pos, gDirRadius, color='g')
    uavsDirAx.append(circle)
    ax.add_artist(circle)
# sos
sosAx = []
for so in sos:
    polygon = Polygon(so, True, color='b')    
    patchSo = ax.add_patch(polygon)
    sosAx.append(patchSo)
# dos
dosAx = []
for do in dos:
    polygon = Polygon(do, True, color='y')
    patchDo = ax.add_patch(polygon)
    dosAx.append(patchDo)
# lcp
lcp = irispy.Polyhedron(a, b)
lcpPoints = lcp.getDrawingVertices()
lcpHull = scipy.spatial.ConvexHull(lcpPoints)
polygon = Polygon(lcpPoints[lcpHull.vertices], True, color='g', alpha=0.3)
lcpAx = ax.add_patch(polygon)

f = open('./scenes.txt', 'r')

DIM = 2
    
def update(frame):
    global uavs, dos, sos, uavsDir, gDir, a, b

    # update data
    s = ''
    for line in f:
        if(line[:-1]=='###'):
            scene = scene_pb2.Scene()
            scene.ParseFromString(s[:-1])
            #print (scene)
            #uavs
            uavsPb = scene.uavs.uav
            uavs = np.ones((len(uavsPb), DIM))
            for i, uavPb in enumerate(uavsPb):
                uavs[i, 0] = uavPb.x
                uavs[i, 1] = uavPb.y
            #dos
            dosPb = scene.dos.do
            dos = []
            for doPb in dosPb:
                do_temp = np.ones((len(doPb.point), DIM))
                for i, pPb in enumerate(doPb.point):
                    do_temp[i, 0] = pPb.x
                    do_temp[i, 1] = pPb.y
                dos.append(do_temp)
            #sos
            sosPb = scene.sos.so
            sos = []
            for soPb in sosPb:
                so_temp = np.ones((len(soPb.point), DIM))
                for i, pPb in enumerate(soPb.point):
                    so_temp[i, 0] = pPb.x
                    so_temp[i, 1] = pPb.y
                sos.append(so_temp)
            #uavsDir
            uavsDirPb = scene.uavsDir.uavDir
            uavsDir = np.ones((len(uavsDirPb), DIM))
            for i, pPb in enumerate(uavsDirPb):
                uavsDir[i, 0] = pPb.x
                uavsDir[i, 1] = pPb.y
            #gDir
            gDirPb = scene.gDir
            gDir = np.ones(DIM)
            gDir[0] = gDirPb.x
            gDir[1] = gDirPb.y

            #a
            aPb = scene.a
            a = np.ones(aPb.row * aPb.col)
            for i, dataPb in enumerate(aPb.data):
                a[i] = dataPb
            a.resize((aPb.row, aPb.col))
            #b
            bPb = scene.b
            b = np.ones(bPb.row)
            for i, dataPb in enumerate(bPb.data):
                b[i] = dataPb
            b.resize((bPb.row, 1))
            break;
        else:
            s += line

    #dos
    for i, doAx in enumerate(dosAx):
        doAx.set_xy(dos[i])
    #sos
    for i, soAx in enumerate(sosAx):
        soAx.set_xy(sos[i])
    #uavs
    for i, uavAx in enumerate(uavsAx):
        uavAx.center = uavs[i]
    #uavsDir
    for i, uavDirAx in enumerate(uavsDirAx):
        uavDirAx.center = uavsDir[i]
    #lcp
    lcp = irispy.Polyhedron(a, b)
    lcpPoints = lcp.getDrawingVertices()
    lcpHull = scipy.spatial.ConvexHull(lcpPoints)
    lcpAx.set_xy(lcpPoints[lcpHull.vertices])

    # Return the modified object
    return dosAx, sosAx, uavsAx, uavsDirAx, lcpAx

import matplotlib.animation as animation
animation = animation.FuncAnimation(fig, update, interval=100, blit=False, frames=range(10))
# animation.save('rain.gif', writer='imagemagick', fps=30, dpi=40)
    
# Show result on screen
plt.show()

f.close()
