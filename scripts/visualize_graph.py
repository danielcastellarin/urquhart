import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch


def keypress(event):
    global frameID, numObs
    # print('press', event.key)
    if event.key == "escape":
        plt.close()
    else:
        # plt.clf()   # clear
        for a in ax.flatten(): a.cla()
        if event.key == 'left' and 1 <= frameID-1:
            frameID -= 1
        elif event.key == 'right' and frameID+1 < len(globalErrors):
            frameID += 1
        display_things()


def getGraphAtFrame(dirname, frameID):
    poseNodes, ldmkNodes = {}, {}
    for line in open(f'{dirname}/graph_nodes/{frameID}.txt'):
        vals = line.split(' ')
        if vals[0][0] == 'P': poseNodes[vals[0]] = tuple(map(float, vals[1:]))
        else: ldmkNodes[vals[0]] = tuple(map(float, vals[1:]))

    ppEdges, plEdges = [], []
    for line in open(f'{dirname}/graph_edges/{frameID}.txt'):
        vals = line.split(' ')
        if vals[1][0] == 'P': ppEdges.append(tuple(vals[0], vals[1], tuple(map(float, vals[2:]))))
        else: plEdges.append(tuple(vals[0], vals[1], tuple(map(float, vals[2:]))))
    
    return poseNodes, ldmkNodes, ppEdges, plEdges, globalErrors[frameID-1] # frameIDs are base-1 this time


def display_things(plt_axis, dirname, frameID):
    poseNodes, ldmkNodes, ppEdges, plEdges, gError = getGraphAtFrame(dirname, frameID)
    plt_axis.set_title(f"{len(ldmkNodes)} Trees | Frame {frameID} | Error {gError}")

    # Draw cyan lines between robot poses
    for src, dst, distance in ppEdges:
        plt_axis.plot([poseNodes[src][0], poseNodes[dst][0]], [poseNodes[src][1], poseNodes[dst][1]], 'c,-' )
    
    # Draw green lines to represent tree observations
    for src, dst, distance in plEdges:
        plt_axis.plot([poseNodes[src][0], ldmkNodes[dst][0]], [poseNodes[src][1], ldmkNodes[dst][1]], 'g,-' )
    
    # Show blue arrows for poses
    for x,y,radians in poseNodes.values(): plt_axis.plot(x, y, 'b', marker=(3,0, 180*radians/np.pi))

    # Show red dots for trees
    for x,y in ldmkNodes.values(): plt_axis.plot(x, y, 'ro')

    # Matplotlib should autoscale graph to fit everything
    fig.canvas.draw()


#######################
# Initialization Script
#######################

parser = argparse.ArgumentParser()
parser.add_argument("dirname", help="The name of the directory with observations")
parser.add_argument("-f", "--frameID", help="Which frame to display", type=int, default=1)
args = parser.parse_args()

if not os.path.isdir(args.dirname):
    print(f'The given directory "{args.dirname}" does not exist.')
    sys.exit(0)


directory = args.dirname
frameID = args.frameID

# Get all global errors beforehand
globalErrors = [float(val) for val in open(f'{directory}/!gError.txt')]

# fig=plt.figure()
fig, ax = plt.subplots(nrows=1, ncols=1)
fig.set_figheight(4)
fig.set_figwidth(4)
fig.canvas.mpl_connect('key_press_event', keypress)
display_things()
print('Press left and right to view different frames. Press "escape" to exit')
plt.show()
