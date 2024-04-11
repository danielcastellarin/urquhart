#!/usr/bin/env python3

# NOTE: uncomment when using visualization remotely (must be executed in terminal, not vs code)
import matplotlib
matplotlib.use('GTK3Agg')

import rospy
import matplotlib as plt
from std_msgs.msg import String

import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch

# def keypress(event):
#     global frameID, numObs
#     # print('press', event.key)
#     if event.key == "escape":
#         plt.close()
#     else:
#         plt.clf()   # clear
#         # for a in ax.flatten(): a.cla()
#         if event.key == 'left' and frameID > 0:
#             frameID -= 1
#         elif event.key == 'right' and frameID < len(kfs)-1:
#             frameID += 1
#         draw_vis()


def draw_frame(id, polygons, triangles, points):
    np.random.seed(10)
    for a in ax.flatten(): a.cla()
    plt.title(f"Frame {id}")
    
    display_triangulation(ax[0], id, triangles, points)
    display_polygons(ax[1], id, polygons)
    
    plt.tight_layout()
    plt.draw()


def display_triangulation(plt_axis, frameID, triangles, trees):
    plt_axis.set_title(f"Triangulation - {frameID}")
    for x,y in triangles: plt_axis.fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
    for x,y in trees: plt_axis.plot(x, y, 'ro')


def display_polygons(plt_axis, frameID, polygons):
    plt_axis.set_title(f"Polygons - {frameID}")
    for x,y in polygons: plt_axis.fill(x, y, facecolor=np.random.rand(3,))



fig, ax = plt.subplots(nrows=2, ncols=1)
fig.set_figheight(9)
fig.set_figwidth(4.5)
numFrames = 0


def read_hierarchy(data: String):
    global numFrames
    id = data.data.split("!")[0]    # get frame ID
    polygons, triangles, points = tuple(data.data.split("!")[1].split("$"))

    polys = []  # Deserialize polygons
    for poly in polygons.split("|"):
        polys.append(zip(*[tuple(map(float, pt.split(' '))) for pt in poly.split(':')]))

    tris = []   # Deserialize triangles
    for tri in triangles.split("|"):
        tris.append(zip(*[tuple(map(float, pt.split(' '))) for pt in tri.split(':')]))

    # Deserialize points
    pts = [tuple(map(float, pt.split(' '))) for pt in points.split(":")]

    draw_frame(id, polys, tris, pts)
    numFrames += 1


################
# Initialization
################

def listener():
    # NOTE: namespace should be set externally
    rospy.init_node('gh_display')
    rospy.Subscriber("hierarchy", String, read_hierarchy)

    # obsRange = rospy.get_param("/sim_path/observationRange", obsRange) * 1.25

    fig.canvas.set_window_title(f"Geometric Hierarchy")

    # plt.ion()
    # plt.show(block=False)
    plt.show()
    rospy.spin()


    plt.close()
    print(f"Number of frames seen: {numFrames}")

if __name__ == '__main__':
    listener()