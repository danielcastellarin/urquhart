#!/usr/bin/env python3

# NOTE: uncomment when using visualization remotely (must be executed in terminal, not vs code)
import matplotlib
matplotlib.use('GTK3Agg')

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose2D
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
# fig, ax = plt.subplots(nrows=2, ncols=1)
fig=plt.figure()
# fig.set_figheight(9)
# fig.set_figwidth(4.5)

# Static drawing variables
theta = np.linspace( 0 , 2 * np.pi , 100 )
cos, sin = np.cos(theta), np.sin(theta)
obsRange = 30
cx, cy = None, None
ox, oy = None, None

# Current pose dynamic storage
# currentPose = None

# # Marker storage (for visualization)
# tree_markers = []
# collBound = None
# obsBound = None
# poseMark = None

# my_trees = []
# trees_x = []
# trees_y = []
# tree_mark = None


def draw_trees(data: pc2.PointCloud2):
    global tree_markers, my_trees, trees_x, trees_y, tree_mark 
    # fig.clf()

    # clear existing points before placing new ones

    # for e in tree_markers: fig.gca().lines.remove(e[0])
    # tree_markers.clear()
    if tree_mark is not None: fig.gca().lines.remove(tree_mark[0])

    # my_trees = [(x,y) for x,y in pc2.read_points(data, field_names=("x", "y"))]
    # my_trees = zip(*[(x,y) for x,y in pc2.read_points(data, field_names=("x", "y"))])
    trees_x, trees_y = zip(*[(x,y) for x,y in pc2.read_points(data, field_names=("x", "y"))])

    tree_mark = fig.gca().plot(trees_x, trees_y, 'ro')
    
    # for x,y in pc2.read_points(data, field_names=("x", "y")): 
    #     fig.gca().plot(x, y, 'ro')
        # tree_markers.append(fig.gca().plot(x, y, 'ro'))
        # tree_markers.append(plt.plot(x, y, 'ro'))

    if (currentPose is not None):
        fig.gca().set_xlim(currentPose.x-obsRange,currentPose.x+obsRange)
        fig.gca().set_ylim(currentPose.y-obsRange,currentPose.y+obsRange)
        # plt.xlim(currentPose.x-obsRange,currentPose.x+obsRange)
        # plt.ylim(currentPose.y-obsRange,currentPose.y+obsRange)


    plt.draw()

def draw_trees_hack(data: pc2.PointCloud2):
    fig.clf()

    # plot trees
    trees_x, trees_y = zip(*[(x,y) for x,y in pc2.read_points(data, field_names=("x", "y"))])
    fig.gca().plot(trees_x, trees_y, 'ro')

    # get pose

    grX, grY, grT, numTrees = tuple(map(float, data.header.frame_id.split(' ')))
    
    # plot pose related stuff
    if cx is not None:
        fig.gca().plot(grX+cx, grY+cy)       # Collision boundary
    fig.gca().plot(grX+ox, grY+oy, 'k')  # Observation Radius
    fig.gca().plot(grX, grY, 'b', marker=(3, 0, 180*grT/np.pi)) # expects degrees

    # Set plot boundaries
    view_range = obsRange*1.5
    fig.gca().set_xlim(grX-view_range,grX+view_range)
    fig.gca().set_ylim(grY-view_range,grY+view_range)

    fig.gca().set_title(f"{int(numTrees)} trees observed")


    plt.draw()


def update_pose(data: Pose2D):
    global currentPose, collBound, obsBound, poseMark
    currentPose = data
    
    # remove elements from figure
    # if collBound is not None and collBound[0] in fig.gca().lines:
    #     fig.gca().lines.remove(collBound[0])
    # if obsBound is not None and obsBound[0] in fig.gca().lines:
    #     fig.gca().lines.remove(obsBound[0])
    # if poseMark is not None and poseMark[0] in fig.gca().lines:
    #     fig.gca().lines.remove(poseMark[0])
    if collBound is not None:
        fig.gca().lines.remove(collBound[0])
    if obsBound is not None:
        fig.gca().lines.remove(obsBound[0])
    if poseMark is not None:
        fig.gca().lines.remove(poseMark[0])

    if cx is not None:
        collBound = fig.gca().plot(currentPose.x+cx, currentPose.y+cy)       # Collision boundary
    obsBound = fig.gca().plot(currentPose.x+ox, currentPose.y+oy, 'k')  # Observation Radius
    poseMark = fig.gca().plot(currentPose.x, currentPose.y, 'b', marker=(3, 0, 180*currentPose.theta/np.pi)) # expects degrees

    # if cx is not None:
    #     collBound = plt.plot(currentPose.x+cx, currentPose.y+cy)       # Collision boundary
    # obsBound = plt.plot(currentPose.x+ox, currentPose.y+oy, 'k')  # Observation Radius
    # poseMark = plt.plot(currentPose.x, currentPose.y, 'b', marker=(3, 0, 180*currentPose.theta/np.pi)) # expects degrees

    plt.draw()


################
# Initialization
################

def listener():
    global obsRange, cx, cy, ox, oy
    rospy.init_node('truth_display')
    rospy.Subscriber("/sim_path/global_points", pc2.PointCloud2, draw_trees_hack)
    # rospy.Subscriber("/sim_path/global_points", pc2.PointCloud2, draw_trees)
    # rospy.Subscriber("/sim_path/global_pose", Pose2D, update_pose)
    obsRange = rospy.get_param("/sim_path/observationRange", obsRange) #* 1.5
    cW = rospy.get_param("/sim_path/collisionRadius", 0)

    ox, oy = obsRange*cos, obsRange*sin
    if cW: cx, cy = cW*cos, cW*sin

    # plt.ion()
    # plt.show(block=False)
    plt.show()
    rospy.spin()


    plt.close()

if __name__ == '__main__':
    listener()