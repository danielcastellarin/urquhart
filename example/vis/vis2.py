import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch

parser = argparse.ArgumentParser()
parser.add_argument("num", help="header of obs")
args = parser.parse_args()


def get_obs(frame):
    polygons = []
    for line in open(frame+args.num+'-p.txt'):
        polygons.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    triangles = []
    for line in open(frame+args.num+'-t.txt'):
        triangles.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    trees = [tuple(map(float, line.split(' '))) for line in open(frame+args.num+'-pts.txt')]
    return polygons, triangles, trees


def display_triangulation(plt_axis, frame):
    _, triangles, trees = get_obs(frame)
    plt_axis.set_title(frame+" Triangulation")
    if frame == "1":
        plt_axis.set_xlim(900,1000)
        plt_axis.set_ylim(-50,50)
    else:
        plt_axis.set_xlim(770,880)
        plt_axis.set_ylim(420,530)
    for i,x,y in triangles:
        plt_axis.fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
        # plt_axis.text(sum(x)/3, sum(y)/3, int(i[0]))
    for x, y in trees: plt_axis.plot(x, y, 'ro')


def display_polygons(plt_axis, frame):
    polygons, _, _ = get_obs(frame)
    plt_axis.set_title(frame+" Polygons")
    if frame == "1":
        plt_axis.set_xlim(900,1000)
        plt_axis.set_ylim(-50,50)
    else:
        plt_axis.set_xlim(770,880)
        plt_axis.set_ylim(420,530)
    for i,x,y in polygons:
        plt_axis.fill(x, y, facecolor=np.random.rand(3,))
        # plt_axis.text(sum(x)/len(x), sum(y)/len(y), int(i[0]))

def draw_vis():
    # First row
    display_triangulation(ax[0][0], "1")
    display_triangulation(ax[0][1], "2")

    # Second row
    display_polygons(ax[1][0], "1")
    display_polygons(ax[1][1], "2")
    
    
#######################
# Initialization Script
#######################
    


np.random.seed(10)

# fig=plt.figure()
fig, ax = plt.subplots(nrows=2, ncols=2)
fig.set_figheight(9)
fig.set_figwidth(9)
draw_vis()
plt.show()
