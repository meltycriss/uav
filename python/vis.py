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

x_min = -8
x_max = 28
y_min = -8
y_max = 28
ax.set_xlim(x_min,x_max), ax.set_xticks(np.linspace(x_min,x_max,x_max-x_min+1,endpoint=True))
ax.set_ylim(y_min,y_max), ax.set_yticks(np.linspace(y_min,y_max,y_max-y_min+1,endpoint=True))

uavRadius = 1
uavsDirRadius = 0.2
gDirRadius = 0.2
uavsAx = []
uavsDirAx = []
sosAx = []
dosAx = []
gDirAx = None
lcpAx = None

f = open('./scenes.txt', 'r')

DIM = 2
count = 0
    
def update(frame):
    global count, DIM
    global uavs, dos, sos, uavsDir, gDir, a, b
    global uavsAx, uavsDirAx, gDirAx, lcpAx, sosAx, dosAx

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
    
    if(count==0):
        # uavs
        for pos in uavs:
            circle = plt.Circle(pos, uavRadius, color='r')
            uavAx = ax.add_artist(circle)
            uavsAx.append(uavAx)
        # uavsDir
        for pos in uavsDir:
            circle = plt.Circle(pos, uavsDirRadius, color='g')
            uavDirAx = ax.add_artist(circle)
            uavsDirAx.append(uavDirAx)
        # gDir
        circle = plt.Circle(gDir, gDirRadius, color='r')
        gDirAx = ax.add_artist(circle)
        # sos
        for so in sos:
            polygon = Polygon(so, True, color='b')    
            patchSo = ax.add_patch(polygon)
            sosAx.append(patchSo)
        # dos
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
    #gDirAx
    gDirAx.center = gDir

    count += 1
    print(count)
    # Return the modified object
    return dosAx, sosAx, uavsAx, uavsDirAx, lcpAx, gDirAx

import matplotlib.animation as animation
animation = animation.FuncAnimation(fig, update, interval=100, blit=False, frames=range(10))
# animation.save('rain.gif', writer='imagemagick', fps=30, dpi=40)
    
# Show result on screen
plt.show()

f.close()
