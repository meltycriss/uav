import numpy as np
import os, sys
import shutil
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from optparse import OptionParser

def updateData():
    global fd, timeStamp, pos, do
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
    if(options.scene=='dynamic'):
        do = posAux[0]
        pos = np.array(posAux[1:])
    else:
        pos = np.array(posAux)
    if(drawPath):
        paths.append(pos)
    return True;

def updateAx():
    global ax
    ax.clear()
    ax.set_xlim(-RANGE, RANGE)
    ax.set_ylim(-RANGE, RANGE)
    ax.axis('off')
    for i in range(pos.shape[0]):
        currColor = COLORS[i%len(COLORS)]
        ax.add_patch(plt.Circle((pos[i][0],pos[i][1]), RADIUS, color=currColor))

        if(drawPath):
            path = np.vstack([p[i] for p in paths])
            ax.plot(path[:,0], path[:,1], ls='dashdot', c=currColor)
    for so in sos:
        # bottom left to top right
        ax.add_patch(patches.Rectangle((so[0], so[1]), WIDTH, LENGTH, facecolor='grey', edgecolor='none'))
    if do is not None:
        ax.add_patch(patches.Rectangle((do[0], do[1]), WIDTH, LENGTH, facecolor='grey', edgecolor='none'))


parser = OptionParser()
parser.add_option('-b', '--blocks', action='store_const', const='blocks', dest='scene', default='blocks')
parser.add_option('-c', '--circle', action='store_const', const='circle', dest='scene')
parser.add_option('-d', '--dynamic', action='store_const', const='dynamic', dest='scene')
parser.add_option('-f', '--four_agent', action='store_const', const='four_agent', dest='scene')

options, args = parser.parse_args()

# common
DIM = 2
STRIDE = 100
timeStamp = None
pos = None

COLORS = []
RANGE = None
RADIUS = None
WIDTH = None
LENGTH = None
sos = []
do = None
drawPath = False
paths = []

if(options.scene=='blocks'):
    # blocks
    COLORS = ['b', 'g', 'r', 'y']
    RANGE = 100
    RADIUS = 2
    WIDTH = 30
    LENGTH = WIDTH
    sos = []
    for i in [0, 50]:
        for j in [0, 50]:
            so = np.ones((DIM))
            x = -40
            y = -40
            so[0] = x + i
            so[1] = y + j
            sos.append(so)
elif(options.scene=='circle'):
    # circle
    COLORS = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    RANGE = 210
    RADIUS = 1.5
elif(options.scene=='dynamic'):
    # dynamic
    STRIDE = 10
    WIDTH = 3 * 2
    LENGTH = 4 * 2
    COLORS = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    RANGE = 70
    RADIUS = 2
elif(options.scene=='four_agent'):
    STRIDE = 10
    COLORS = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
    RANGE = 25
    RADIUS = 2
    drawPath = True



inputFile = args[0]
outputFile = os.path.splitext(inputFile)[0]
if os.path.isdir(outputFile):
    shutil.rmtree(outputFile)
os.mkdir(outputFile)

fig = plt.figure(figsize=(8,8), dpi=160)
ax = plt.gca()

fd = open(inputFile, 'r')

counter = 0
while(updateData()):
    print counter
    updateAx()
    filename = ('%07.2f' % float(timeStamp)).replace('.','')
    outputPath = os.path.join(outputFile, filename)
    plt.savefig(outputPath, dpi=fig.dpi)
    counter = counter + 1

fd.close()

