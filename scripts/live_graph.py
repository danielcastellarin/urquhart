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
#         # plt.clf()   # clear
#         # for a in ax.flatten(): a.cla()
#         ax.cla()
#         if event.key == 'left' and 1 <= frameID-1:
#             frameID -= 1
#         elif event.key == 'right' and frameID+1 < len(globalErrors)+1:
#             frameID += 1
#         display_things(ax, directory, frameID)


# def getGraphAtFrame(dirname, frameID):
#     poseNodes, ldmkNodes = {}, {}
#     for line in open(f'{dirname}/global/graph_nodes/{frameID}.txt'):
#         vals = line.split(' ')
#         if vals[0][0] == 'P': poseNodes[vals[0]] = tuple(map(float, [n for n in vals[1:] if n != ""]))
#         else: ldmkNodes[vals[0]] = tuple(map(float, [n for n in vals[1:] if n != ""]))

#     ppEdges, plEdges = [], []
#     for line in open(f'{dirname}/global/graph_edges/{frameID}.txt'):
#         vals = line.split(' ')
#         if vals[1][0] == 'P': ppEdges.append((vals[0], vals[1], tuple(map(float, [n for n in vals[2:] if n != ""]))))
#         else: plEdges.append((vals[0], vals[1], tuple(map(float, [n for n in vals[2:] if n != ""]))))
    
#     return poseNodes, ldmkNodes, ppEdges, plEdges, globalErrors[frameID-1] # frameIDs are base-1 this time
#     # return poseNodes, ldmkNodes, ppEdges, plEdges, 0 # frameIDs are base-1 this time


# def display_things(plt_axis, dirname, frameID):
#     poseNodes, ldmkNodes, ppEdges, plEdges, gError = getGraphAtFrame(dirname, frameID)
#     plt_axis.set_title(f"{len(ldmkNodes)} Trees | Frame {frameID} | Existing Error {gError}")

#     # Draw cyan lines between robot poses
#     for src, dst, distance in ppEdges:
#         plt_axis.plot([poseNodes[src][0], poseNodes[dst][0]], [poseNodes[src][1], poseNodes[dst][1]], 'c,-' )
    
#     # Draw green lines to represent tree observations
#     for src, dst, distance in plEdges:
#         plt_axis.plot([poseNodes[src][0], ldmkNodes[dst][0]], [poseNodes[src][1], ldmkNodes[dst][1]], 'g,-' )
    
#     # Show blue arrows for poses
#     for x,y,radians in poseNodes.values(): plt_axis.plot(x, y, 'b', marker=(3,0, 180*radians/np.pi))

#     # Show red dots for trees
#     for x,y in ldmkNodes.values(): plt_axis.plot(x, y, 'ro')

#     # Matplotlib should autoscale graph to fit everything
#     fig.canvas.draw()


fig=plt.figure()

varthing = 180 / np.pi
def draw_graph(data: String):
    global graphFrames
    fig.clf()

    # poses | landmark positions | existing ldmk refs | new ldmk refs | previous global error
    poses, ldmkPos, exLdmkRefs, newLdmkRefs, glError = tuple(data.data.split("|"))

    # Parse robot poses and landmarks
    poseList = [tuple(map(float, pose.split(" "))) for pose in poses.split(":")]
    ldmkList = [tuple(map(float, ldmk.split(" "))) for ldmk in ldmkPos.split(":")]

    # Prepare landmark references
    exLdmkRefs = set(map(int, exLdmkRefs.split(" "))) if exLdmkRefs else set()
    newLdmkRefs = set(map(int, newLdmkRefs.split(" "))) if newLdmkRefs else set()

    # rospy.loginfo(newLdmkRefs)

    # Draw lines to landmarks matched in this keyframe
    for i, (x,y) in enumerate(ldmkList):
        if i in exLdmkRefs:
            fig.gca().plot([poseList[-1][0], x], [poseList[-1][1], y], color='g', linestyle="-")
        elif i in newLdmkRefs:
            fig.gca().plot([poseList[-1][0], x], [poseList[-1][1], y], color='purple', linestyle="-")

    # Draw landmark markers
    for i, (x,y) in enumerate(ldmkList):
        if i in exLdmkRefs: fig.gca().plot(x, y, color='g', marker='o')
        elif i in newLdmkRefs: fig.gca().plot(x, y, color='purple', marker='o')
        else: fig.gca().plot(x, y, 'ro')

    # Draw latest poses
    for i, (poseX,poseY,radians) in enumerate(poseList):
        fig.gca().plot(poseX, poseY, ('b' if i<len(poseList)-1 else 'k'), marker=(3,0, varthing*radians))
    

    # Define plot title and draw
    fig.gca().set_title(f"Frame {len(poseList)} | {len(exLdmkRefs)} Assoc; {len(newLdmkRefs)} New | Existing Error: {glError}")
    plt.draw()




################
# Initialization
################

def listener():
    # global obsRange, cx, cy, ox, oy
    rospy.init_node('graph_display')
    rospy.Subscriber("/graph_builder/graph", String, draw_graph)
    # obsRange = rospy.get_param("/sim_path/observationRange", obsRange)
    # cW = rospy.get_param("/sim_path/collisionRadius", 0)
    fig.canvas.set_window_title("Global Graph")

    # plt.ion()
    # plt.show(block=False)
    plt.show()
    rospy.spin()


    plt.close()

if __name__ == '__main__':
    listener()