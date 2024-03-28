import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch

parser = argparse.ArgumentParser()
parser.add_argument("num", help="header of obs")
args = parser.parse_args()


def get_obs():
    polygons = []
    for line in open(args.num+'-p.txt'):
        polygons.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    triangles = []
    for line in open(args.num+'-t.txt'):
        triangles.append(zip(*[tuple(map(float, c.split(' '))) for c in line.split('|')[:-1]]))

    trees = [tuple(map(float, line.split(' '))) for line in open(args.num+'-pts.txt')]
    return polygons, triangles, trees


def display_triangulation(plt_axis):
    _, triangles, trees = get_obs()
    plt_axis.set_title("Triangulation")
    plt_axis.set_xlim(900,1000)
    plt_axis.set_ylim(-50,50)
    for i,x,y in triangles:
        plt_axis.fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
        # plt_axis.text(sum(x)/3, sum(y)/3, int(i[0]))
    for x, y in trees: plt_axis.plot(x, y, 'ro')


def display_polygons(plt_axis):
    polygons, _, _ = get_obs()
    plt_axis.set_title("Polygons")
    plt_axis.set_xlim(900,1000)
    plt_axis.set_ylim(-50,50)
    for i,x,y in polygons:
        plt_axis.fill(x, y, facecolor=np.random.rand(3,))
        # plt_axis.text(sum(x)/len(x), sum(y)/len(y), int(i[0]))

def draw_vis():
    # First row
    display_triangulation(ax[0])

    # Second row
    display_polygons(ax[1])
    
    
#######################
# Initialization Script
#######################
    


np.random.seed(10)

# fig=plt.figure()
fig, ax = plt.subplots(nrows=2, ncols=1)
fig.set_figheight(9)
fig.set_figwidth(4)
draw_vis()
plt.show()
