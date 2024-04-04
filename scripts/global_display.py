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




def draw_frame(id):
    np.random.seed(10)
    for a in ax.flatten(): a.cla()
    plt.title(f"Frame {id}")
    
    display_triangulation(ax[0], id, allTriangles[id], allPoints[id])
    display_polygons(ax[1], id, allPolygons[id])
    
    plt.draw()


def display_triangulation(plt_axis, frameID, triangles, trees):
    plt_axis.set_title(f"Triangulation - {frameID}")
    for x,y in triangles: plt_axis.fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
    for x,y in trees: plt_axis.plot(x, y, 'ro')


def display_polygons(plt_axis, frameID, polygons):
    plt_axis.set_title(f"Polygons - {frameID}")
    for x,y in polygons: plt_axis.fill(x, y, facecolor=np.random.rand(3,))



allPolygons = {}
allTriangles = {}
allPoints = {}
fig, ax = plt.subplots(nrows=2, ncols=1)
fig.set_figheight(9)
fig.set_figwidth(4.5)



def read_polygons(data: String):
    id = data.data.split("!")[0]    # get frame ID

    polys = []  # Deserialize polygons
    for poly in data.data.split("!")[1].split(":")[:-1]:
        polys.append(zip(*[tuple(map(float, pt.split(' '))) for pt in poly.split('|')]))
    
    allPolygons[id] = polys # draw frame if we have all the data
    if id in allTriangles and id in allPoints:
        draw_frame(id)


def read_triangles(data: String):
    id = data.data.split("!")[0]    # get frame ID

    tris = []   # Deserialize triangles
    for tri in data.data.split("!")[1].split(":")[:-1]:
        tris.append(zip(*[tuple(map(float, pt.split(' '))) for pt in tri.split('|')]))
    
    allTriangles[id] = tris # draw frame if we have all the data
    if id in allPolygons and id in allPoints:
        draw_frame(id)


def read_points(data: String):
    id = data.data.split("!")[0]    # get frame ID
    pts = [tuple(map(float, pt.split(' '))) for pt in data.data.split("!")[1].split("|")]
    
    allPoints[id] = pts # draw frame if we have all the data
    if id in allPolygons and id in allTriangles:
        draw_frame(id)


################
# Initialization
################

def listener():
    rospy.init_node('global_display')
    rospy.Subscriber("/graph_builder/polygons", String, read_polygons)
    rospy.Subscriber("/graph_builder/triangles", String, read_triangles)
    rospy.Subscriber("/graph_builder/points", String, read_points)
    # obsRange = rospy.get_param("/sim_path/observationRange", obsRange) * 1.25

    # plt.ion()
    # plt.show(block=False)
    plt.show()
    rospy.spin()


    plt.close()
    print(f"Number of frames seen: {len(allPoints)}")

if __name__ == '__main__':
    listener()