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


# Static drawing variables
fig=plt.figure()
theta = np.linspace( 0 , 2 * np.pi , 100 )
cos, sin = np.cos(theta), np.sin(theta)
obsRange = 30
cx, cy = None, None
ox, oy = None, None


def draw_trees_hack(data: pc2.PointCloud2):
    fig.clf()

    # plot trees
    trees_x, trees_y = zip(*[(x,y) for x,y in pc2.read_points(data, field_names=("x", "y"))])
    fig.gca().plot(trees_x, trees_y, 'ro')

    # get pose

    grX, grY, grT, numTrees = tuple(map(float, data.header.frame_id.split(' ')))
    
    # plot pose related stuff
    if cx is not None: fig.gca().plot(grX+cx, grY+cy)           # Collision boundary
    fig.gca().plot(grX+ox, grY+oy, 'k')                         # Observation Radius
    fig.gca().plot(grX, grY, 'b', marker=(3, 0, 180*grT/np.pi)) # expects degrees

    # Set plot boundaries
    view_range = obsRange*1.5
    fig.gca().set_xlim(grX-view_range,grX+view_range)
    fig.gca().set_ylim(grY-view_range,grY+view_range)

    fig.gca().set_title(f"{int(numTrees)} trees observed")


    plt.draw()



################
# Initialization
################

def listener():
    global obsRange, cx, cy, ox, oy
    rospy.init_node('truth_display')
    rospy.Subscriber("/sim_path/global_points", pc2.PointCloud2, draw_trees_hack)
    obsRange = rospy.get_param("/sim_path/observationRange", obsRange)
    cW = rospy.get_param("/sim_path/collisionRadius", 0)
    fig.canvas.set_window_title(f"Ground Truth")

    ox, oy = obsRange*cos, obsRange*sin
    if cW: cx, cy = cW*cos, cW*sin

    # plt.ion()
    # plt.show(block=False)
    plt.show()
    rospy.spin()


    plt.close()

if __name__ == '__main__':
    listener()