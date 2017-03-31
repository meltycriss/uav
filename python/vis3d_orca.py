import numpy as np
import os, sys
import shutil
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from optparse import OptionParser
import mpl_toolkits.mplot3d as a3

def updateData():
    global fd, timeStamp, pos, do, paths
    line = fd.readline()
    if(line==''):
        return False;

    # skip STRIDE-1 line, if fd ends in the process, use the last line
    lastLine = line
    for i in range(STRIDE-1):
        currLine = fd.readline()
        if(currLine==''):
            line = lastLine
            break
        lastLine = currLine
    
    data = line.split()
    timeStamp = data[0]
    posAux = data[1:]
    posAux = [str(p)[1:-1].split(',') for p in posAux]
    posAux = [map(float, p) for p in posAux]
    pos = np.array(posAux)
    paths.append(pos)
    return True;

def updateAx():
    global ax
    ax.clear()
    ax.set_xlim(-ratio*RANGE, ratio*RANGE)
    ax.set_ylim(-ratio*RANGE, ratio*RANGE)
    ax.set_zlim3d(-ratio*RANGE, ratio*RANGE)
    #ax.w_xaxis.set_pane_color((1,1,1,0))
    #ax.w_yaxis.set_pane_color((1,1,1,0))
    #ax.w_zaxis.set_pane_color((1,1,1,0))
    ax.w_xaxis.line.set_color((1,1,1,0))
    ax.w_yaxis.line.set_color((1,1,1,0))
    ax.w_zaxis.line.set_color((1,1,1,0))
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])

    # get an ellipsoid
    #coefs = (1, 2, 2)  # Coefficients in a0/c x**2 + a1/c y**2 + a2/c z**2 = 1 
    coefs = (1/(RADIUS**2), 1/(RADIUS**2), 1/(RADIUS**2))  # Coefficients in a0/c x**2 + a1/c y**2 + a2/c z**2 = 1 
    # Radii corresponding to the coefficients:
    rx, ry, rz = 1/np.sqrt(coefs)
    
    # Set of all spherical angles:
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    
    # Cartesian coordinates that correspond to the spherical angles:
    # (this is the equation of an ellipsoid):
    x = rx * np.outer(np.cos(u), np.sin(v))
    y = ry * np.outer(np.sin(u), np.sin(v))
    z = rz * np.outer(np.ones_like(u), np.cos(v))


    for i in range(pos.shape[0]):
        # Plot:
        currColor = COLORS[i%len(COLORS)]
        ax.plot_surface(x+pos[i][0], y+pos[i][1], z+pos[i][2],  rstride=4, cstride=4, color=currColor, linewidth=0.1)

        path = np.vstack([p[i] for p in paths])
        ax.plot(path[:,0], path[:,1], path[:,2], ls='dashdot', c=currColor)

DIM = 3
STRIDE = 25
ratio = 1.2
viewFiles = ['normal', 'top', 'front', 'side']
viewParams = [
        [20, 60],
        [90, 90],
        [0, 90],
        [0, 0],
        ]

timeStamp = None
pos = None
paths = []

COLORS = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
RANGE = 10
RADIUS = 1.5

inputFile = sys.argv[1]
outputFile = os.path.splitext(inputFile)[0]
if os.path.isdir(outputFile):
    shutil.rmtree(outputFile)
os.mkdir(outputFile)
for i in range(len(viewFiles)):
    os.mkdir(os.path.join(outputFile, viewFiles[i]))

fig = plt.figure(figsize=(8,8), dpi=80)
ax = a3.Axes3D(fig)

fd = open(inputFile, 'r')

counter = 0
while(updateData()):
    print counter
    updateAx()
    filename = ('%07.2f' % float(timeStamp)).replace('.','')
    for i in range(len(viewFiles)):
        outputPath = os.path.join(outputFile, viewFiles[i])
        outputPath = os.path.join(outputPath, filename)
        ax.view_init(viewParams[i][0], viewParams[i][1])
        plt.savefig(outputPath, dpi=fig.dpi)
    counter = counter + 1

fd.close()

