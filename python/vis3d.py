import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import scene3d_pb2
import irispy
import scipy
import matplotlib.animation as animation
import sys, os
import mpl_toolkits.mplot3d as a3
import os, sys
import subprocess
import shutil

# default scenes file name
input_file_name = './scenes.txt'
if(len(sys.argv)==2):
    input_file_name = sys.argv[1]

fig = plt.figure(figsize=(8,8), dpi=80)

ax = a3.Axes3D(fig)

x_min = -15
x_max = 15
y_min = -10
y_max = 40
z_min = -15
z_max = 15

ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_zlim3d(z_min, z_max)


uavRadius = 1
uavDirRadius = 0.2
gDirRadius = 0.2

obsColor = 'b'
uavColor = 'r'
uavDirColor = 'g'
gDirColor = 'r'

topPath = 'top'
leftPath = 'left'
frontPath = 'front'
normalPath = 'normal'

if os.path.exists(topPath):
    shutil.rmtree(topPath)
if os.path.exists(leftPath):
    shutil.rmtree(leftPath)
if os.path.exists(frontPath):
    shutil.rmtree(frontPath)
if os.path.exists(normalPath):
    shutil.rmtree(normalPath)

os.mkdir(topPath)
os.mkdir(leftPath)
os.mkdir(frontPath)
os.mkdir(normalPath)

# data Var
uavs = None
uavsDir = None
gDir = None
a = None
b = None
dos = None
sos = None
currCentroid = None

fd = open(input_file_name, 'r')

DIM = 3

def updateData():
    global fd
    global DIM
    global uavs, uavsDir, gDir, a, b, dos, sos, currCentroid
    # update data
    s = ''
    for line in fd:
        if(line[:-1]=='###'):
            scene = scene3d_pb2.Scene()
            scene.ParseFromString(s[:-1])
            #print (scene)
            #uavs
            uavsPb = scene.uavs.uav
            uavs = np.ones((len(uavsPb), DIM))
            for i, uavPb in enumerate(uavsPb):
                uavs[i, 0] = uavPb.x
                uavs[i, 1] = uavPb.y
                uavs[i, 2] = uavPb.z
            #uavsDir
            uavsDirPb = scene.uavsDir.uavDir
            uavsDir = np.ones((len(uavsDirPb), DIM))
            for i, pPb in enumerate(uavsDirPb):
                uavsDir[i, 0] = pPb.x
                uavsDir[i, 1] = pPb.y
                uavsDir[i, 2] = pPb.z
            #gDir
            gDirPb = scene.gDir
            gDir = np.ones(DIM)
            gDir[0] = gDirPb.x
            gDir[1] = gDirPb.y
            gDir[2] = gDirPb.z
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
            #if top is equal with bottom, only leave one 
            if(a.shape[0]%2==0):
                ab = np.hstack((a,b))
                abTop = np.split(ab,2)[0]
                abBot = np.split(ab,2)[1]
                if (np.allclose(abTop,abBot)):
                    ab = abTop
                    ab = np.hsplit(ab, np.array([2]))
                    a = ab[0]
                    b = ab[1]
            #sos
            sosPb = scene.sos.so
            sos = []
            for soPb in sosPb:
                so_temp = np.ones((len(soPb.point), DIM))
                for i, pPb in enumerate(soPb.point):
                    so_temp[i, 0] = pPb.x
                    so_temp[i, 1] = pPb.y
                    so_temp[i, 2] = pPb.z
                sos.append(so_temp)
            #dos
            dosPb = scene.dos.do
            dos = []
            for doPb in dosPb:
                do_temp = np.ones((len(doPb.point), DIM))
                for i, pPb in enumerate(doPb.point):
                    do_temp[i, 0] = pPb.x
                    do_temp[i, 1] = pPb.y
                    do_temp[i, 2] = pPb.z
                dos.append(do_temp)
            #currCentroid
            currCentroidPb = scene.currCentroid
            currCentroid = np.ones(DIM)
            currCentroid[0] = currCentroidPb.x
            currCentroid[1] = currCentroidPb.y
            currCentroid[2] = currCentroidPb.z
            break;
        else:
            s += line
    if s=='':
        return False
    else:
        return True

def draw(convhull, **kwargs):
    irispy.drawing.draw_3d_convhull(convhull, plt.gca(), **kwargs)

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


counter = 0

while(updateData()):
    if counter%50==0:
        print counter
    plt.cla()

    for i in range(len(sos)):
        draw(sos[i], facecolor=obsColor)
    for i in range(len(uavs)):
        draw(toCube(uavs[i],uavRadius), facecolor=uavColor)
    for i in range(len(uavsDir)):
        draw(toCube(uavsDir[i],uavDirRadius), facecolor=uavDirColor)
    draw(toCube(gDir,gDirRadius), facecolor=gDirColor)

    # normal view
    ax.view_init(20, 60)
    plt.savefig(os.path.join(normalPath, '%06d.jpg' % counter))
    # top view
    ax.view_init(90, 90)
    plt.savefig(os.path.join(topPath, '%06d.jpg' % counter))
    # left view
    ax.view_init(0, 0)
    plt.savefig(os.path.join(leftPath, '%06d.jpg' % counter))
    # front view
    ax.view_init(0, 90)
    plt.savefig(os.path.join(frontPath, '%06d.jpg' % counter))

    counter += 1

#cmd = ['convert', '-delay', '1', os.path.join(jpgPath, '*.jpg'), 'anim.gif']
#subprocess.call(cmd)

fd.close()
