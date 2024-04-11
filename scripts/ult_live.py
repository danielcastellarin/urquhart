#!/usr/bin/env python3

# NOTE: uncomment when using visualization remotely (must be executed in terminal, not vs code)
import matplotlib
matplotlib.use('GTK3Agg')

import rospy
import matplotlib as plt
from std_msgs.msg import String
import sensor_msgs.point_cloud2 as pc2

import os
import sys
import argparse
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import ConnectionPatch


fig1=plt.figure(1)
fig1.canvas.set_window_title("Ground Truth")
fig1.canvas.manager.window.move(0, 0)
fig1.tight_layout()
# plt.get_current_fig_manager().window.setGeometry(0,0,400,400)
fig2=plt.figure(2)
fig2.canvas.set_window_title("Keyframes")
fig2.canvas.manager.window.move(0, 640)
fig2.tight_layout()
# plt.get_current_fig_manager().window.setGeometry(400,0,400,400)
fig3=plt.figure(3)
fig3.canvas.set_window_title("Global Hierarchy")
fig3.canvas.manager.window.move(640, 0)
fig3.tight_layout()
# plt.get_current_fig_manager().window.setGeometry(0,400,400,400)
fig4=plt.figure(4)
fig4.canvas.set_window_title("Global Graph")
fig4.canvas.manager.window.move(640, 640)
fig4.tight_layout()
# plt.get_current_fig_manager().window.setGeometry(400,400,400,400)

# Static drawing variables
theta = np.linspace( 0 , 2 * np.pi , 100 )
cos, sin = np.cos(theta), np.sin(theta)
obsRange = 30
cx, cy = None, None
ox, oy = None, None


def draw_ground_truth(data: pc2.PointCloud2):
    fig1.clf()

    # plot trees
    trees_x, trees_y = zip(*[(x,y) for x,y in pc2.read_points(data, field_names=("x", "y"))])
    fig1.gca().plot(trees_x, trees_y, 'ro')

    # get pose
    grX, grY, grT, numTrees = tuple(map(float, data.header.frame_id.split(' ')))
    
    # plot pose related stuff
    if cx is not None: fig1.gca().plot(grX+cx, grY+cy)           # Collision boundary
    fig1.gca().plot(grX+ox, grY+oy, 'k')                         # Observation Radius
    fig1.gca().plot(grX, grY, 'b', marker=(3, 0, 180*grT/np.pi)) # expects degrees

    # Set plot boundaries
    view_range = obsRange*1.5
    fig1.gca().set_xlim(grX-view_range,grX+view_range)
    fig1.gca().set_ylim(grY-view_range,grY+view_range)

    fig1.gca().set_title(f"{int(numTrees)} trees observed")
    fig1.tight_layout()
    fig1.canvas.draw()


def draw_keyframe_points(data: pc2.PointCloud2):
    fig2.clf()
    
    # Read and count points in the cloud
    numTrees = 0
    for x,y in pc2.read_points(data, field_names=("x", "y")): 
        fig2.gca().plot(x, y, 'ro')
        numTrees += 1

    # Draw plot
    fig2.gca().set_title(f"Keyframe ID: {data.header.seq} | Trees: {numTrees}")
    fig2.gca().set_xlim(-obsRange, obsRange)
    fig2.gca().set_ylim(-obsRange, obsRange)
    fig2.tight_layout()
    fig2.canvas.draw()


def draw_gh(data: String):
    id = data.data.split("!")[0]    # get frame ID
    polygons, triangles, _ = tuple(data.data.split("!")[1].split("$"))

    polys = []  # Deserialize polygons
    for poly in polygons.split("|"):
        polys.append(zip(*[tuple(map(float, pt.split(' '))) for pt in poly.split(':')]))

    tris = []   # Deserialize triangles
    for tri in triangles.split("|"):
        tris.append(zip(*[tuple(map(float, pt.split(' '))) for pt in tri.split(':')]))

    # Reset the plot frame
    np.random.seed(10)
    fig3.clf()
    fig3.gca().set_title(f"Global GeoHierarchy: {id}")

    # Overlay the polygons with the triangulation
    for x,y in polys: fig3.gca().fill(x, y, facecolor=np.random.rand(3,))
    for x,y in tris: fig3.gca().fill(x, y, facecolor='none', edgecolor='black', linewidth=2)
    fig3.tight_layout()
    fig3.canvas.draw()


varthing = 180 / np.pi
def draw_graph(data: String):
    global graphFrames
    fig4.clf()

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
            fig4.gca().plot([poseList[-1][0], x], [poseList[-1][1], y], color='m', linestyle="-")
        elif i in newLdmkRefs:
            fig4.gca().plot([poseList[-1][0], x], [poseList[-1][1], y], color='purple', linestyle="-")

    # Draw landmark markers
    for i, (x,y) in enumerate(ldmkList):
        if i in exLdmkRefs: fig4.gca().plot(x, y, color='m', marker='o')
        elif i in newLdmkRefs: fig4.gca().plot(x, y, color='purple', marker='o')
        else: fig4.gca().plot(x, y, 'ro')

    # Draw latest poses
    for i, (poseX,poseY,radians) in enumerate(poseList):
        fig4.gca().plot(poseX, poseY, ('b' if i<len(poseList)-1 else 'k'), marker=(3,0, varthing*radians))
    

    # Define plot title and draw
    fig4.gca().set_title(f"Frame {len(poseList)} | {len(exLdmkRefs)} Associations | Established {len(newLdmkRefs)} Trees | Existing Error: {glError}")
    fig4.tight_layout()
    fig4.canvas.draw()



################
# Initialization
################

def listener():
    rospy.init_node('ultimate_live_display')
    rospy.Subscriber("/sim_path/global_points", pc2.PointCloud2, draw_ground_truth)
    rospy.Subscriber("keyframe_maker/keyframe", pc2.PointCloud2, draw_keyframe_points)
    rospy.Subscriber("/graph_builder/hierarchy", String, draw_gh)
    rospy.Subscriber("/graph_builder/graph", String, draw_graph)

    global obsRange, cx, cy, ox, oy
    obsRange = rospy.get_param("/sim_path/observationRange", obsRange)
    cW = rospy.get_param("/sim_path/collisionRadius", 0)
    ox, oy = obsRange*cos, obsRange*sin
    if cW: cx, cy = cW*cos, cW*sin


    # plt.ion()
    # fig.canvas.set_window_title(f"Global Mapping Dashboard")
    # plt.tight_layout()
    # plt.show(block=False)
    plt.show()
    rospy.spin()


    plt.close()

if __name__ == '__main__':
    listener()