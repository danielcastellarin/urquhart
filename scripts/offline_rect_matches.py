#!/usr/bin/env python3

# NOTE: uncomment when using visualization remotely (must be executed in terminal, not vs code)
import matplotlib
matplotlib.use('GTK3Agg')

import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch
import math

def keypress(event):
    global frameID, toggleAllMatches
    # print('press', event.key)
    if event.key == "escape":
        plt.close()
    else:
        if event.key == 'left' and 1 <= frameID-1:  # base-1 this time
            for a in ax.flatten(): a.cla()
            frameID -= 1
            draw_vis()
        elif event.key == 'right' and frameID+1 < len(globalErrors):
            for a in ax.flatten(): a.cla()
            frameID += 1
            draw_vis()
        elif event.key == ' ':
            for a in ax.flatten(): a.cla()
            toggleAllMatches = not toggleAllMatches
            draw_vis()

def getRot(dirname, frameID):
    poseNodes = {}
    for line in open(f'{dirname}/global/graph_nodes/{frameID}.txt'):
        vals = line.split(' ')
        if vals[0][0] == 'P': poseNodes[int(vals[0][1:])+1] = tuple(map(float, [n for n in vals[1:] if n != ""]))
    return poseNodes[frameID][2]
    

def get_global_obs(dirname, frameID):
    polygons = []
    for line in open(f'{dirname}/global/p/{frameID}.txt'):
        polygons.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    triangles = []
    for line in open(f'{dirname}/global/t/{frameID}.txt'):
        triangles.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    trees = [tuple(map(float, [n for n in line.split(' ')[1:] if n != ""])) for line in open(f'{dirname}/global/graph_nodes/{frameID}.txt') if line.split(' ')[0][0] == 'L']
    return polygons, triangles, trees

def get_local_obs(dirname, frameID):
    polygons = []
    for line in open(f'{dirname}/local/p/{frameID}.txt'):
        polygons.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    triangles = []
    for line in open(f'{dirname}/local/t/{frameID}.txt'):
        triangles.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    trees = [tuple(map(float, line.split(' '))) for line in open(f'{dirname}/local/pts/{frameID}.txt')]
    return polygons, triangles, trees


def display_triangulation(plt_axis, dirname, frameID, getObsFunc, headTitle, s, c):
    _, triangles, trees = getObsFunc(dirname, frameID)
    plt_axis.set_title(f"{headTitle} Triangulation - {frameID}")
    sv,cv = np.empty(3), np.empty(3)
    sv.fill(s), cv.fill(c)
    for i,x,y in triangles:
        plt_axis.fill(np.array(x)*c - np.array(y)*s, np.array(x)*s + np.array(y)*c, facecolor='none', edgecolor='black', linewidth=2)
        # plt_axis.fill(x*c-y*s, x*s+y*c, facecolor='none', edgecolor='black', linewidth=2)
        # plt_axis.fill(x.dot(c)-y.dot(s), x.dot(s)+y.dot(c), facecolor='none', edgecolor='black', linewidth=2)
        # plt_axis.fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
        # plt_axis.text(sum(x)/3, sum(y)/3, int(i[0]))
    
    
    for x, y in trees: plt_axis.plot(x*c-y*s, x*s+y*c, 'ro')


def display_polygons(plt_axis, dirname, frameID, getObsFunc, headTitle):
    polygons, _, _ = getObsFunc(dirname, frameID)
    plt_axis.set_title(f"{headTitle} Polygons - {frameID}")
    for i,x,y in polygons:
        plt_axis.fill(x, y, facecolor=np.random.rand(3,))
        # plt_axis.text(sum(x)/len(x), sum(y)/len(y), int(i[0]))

def draw_vis():
    np.random.seed(10)
    rads = getRot(directory, frameID)
    s,c = math.sin(rads), math.cos(rads)

    # First row
    display_triangulation(ax[0][0], directory, frameID+1, get_local_obs, "Local", s, c)
    display_triangulation(ax[0][1], directory, frameID, get_global_obs, "Global", 0, 1)

    # Second row
    display_polygons(ax[1][0], directory, frameID+1, get_local_obs, "Local")
    display_polygons(ax[1][1], directory, frameID, get_global_obs, "Global")

    if toggleAllMatches:
        for line in open(f'{directory}/match/{frameID}.txt'):
            if line == '': continue
            # p1, p2 = [tuple(map(float, c.split(','))) for c in line.split('|')]
            (p1x, p1y), (p2x, p2y) = [tuple(map(float, c.split(','))) for c in line.split('|')]
            p1, p2 = (p1x*c-p1y*s, p1x*s+p1y*c), (p2x, p2y)
            ax[0][1].add_artist(ConnectionPatch(xyA=p1, xyB=p2, coordsA="data", coordsB="data", axesA=ax[0][0], axesB=ax[0][1], color="blue"))
    else:
        for line in open(f'{directory}/finalAssoc/{frameID}m.txt'):
            if line == '': continue
            # p1, p2 = [tuple(map(float, c.split(','))) for c in line.split('|')]
            (p1x, p1y), (p2x, p2y) = [tuple(map(float, c.split(','))) for c in line.split('|')]
            p1, p2 = (p1x*c-p1y*s, p1x*s+p1y*c), (p2x, p2y)
            ax[0][1].add_artist(ConnectionPatch(xyA=p1, xyB=p2, coordsA="data", coordsB="data", axesA=ax[0][0], axesB=ax[0][1], color="g"))
            ax[0][1].plot(p2[0], p2[1], color="g", marker="o")
            ax[0][0].plot(p1[0], p1[1], color="g", marker="o")
        
        for line in open(f'{directory}/finalAssoc/{frameID}u.txt'):
            if line == '': continue
            # p1, p2 = [tuple(map(float, c.split(','))) for c in line.split('|')]
            (p1x, p1y), (p2x, p2y) = [tuple(map(float, c.split(','))) for c in line.split('|')]
            p1, p2 = (p1x*c-p1y*s, p1x*s+p1y*c), (p2x, p2y)
            ax[0][1].add_artist(ConnectionPatch(xyA=p1, xyB=p2, coordsA="data", coordsB="data", axesA=ax[0][0], axesB=ax[0][1], color="purple"))
            ax[0][1].plot(p2[0], p2[1], color="purple", marker="o")
            ax[0][0].plot(p1[0], p1[1], color="purple", marker="o")

    
    fig.canvas.draw()

#######################
# Initialization Script
#######################

parser = argparse.ArgumentParser()
parser.add_argument("dirname", help="The name of the directory with observations")
parser.add_argument("-f", "--frameID", help="Which frame to display", type=int, default=1) # base-1 this time
args = parser.parse_args()

if not os.path.isdir(args.dirname):
    print(f'The given directory "{args.dirname}" does not exist.')
    sys.exit(0)


directory = args.dirname
frameID = args.frameID
toggleAllMatches = True

# Get all global errors beforehand
globalErrors = [float(val) for val in open(f'{directory}/global/!gError.txt')]

# fig=plt.figure()
fig, ax = plt.subplots(nrows=2, ncols=2)
fig.set_figheight(9)
fig.set_figwidth(9)
fig.canvas.mpl_connect('key_press_event', keypress)
draw_vis()
print('Press left and right to view different frames. Press "escape" to exit')
plt.show()
