#!/usr/bin/env python3

# NOTE: uncomment when using visualization remotely (must be executed in terminal, not vs code)
# import matplotlib
# matplotlib.use('GTK3Agg')

import rospy
import matplotlib as plt
import sensor_msgs.point_cloud2 as pc2

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




# def draw_vis():
#     plt.title(f"Frame {frameID}")
#     plt.xlim(-25,25)
#     plt.ylim(-25,25)
#     if not kfs: 
#         print("No frames yet")
#     elif frameID > len(kfs):
#         print(f"Invalid frameID {frameID}")
#     for x,y in kfs[frameID]: plt.plot(x, y, 'ro')
    
#     fig.canvas.draw()




# fig=plt.figure()
# # fig, ax = plt.subplots(nrows=2, ncols=4)
# fig.set_figheight(9)
# fig.set_figwidth(4.5)
# fig.canvas.mpl_connect('key_press_event', keypress)
# draw_vis()
# print('Press left and right to view different frames. Press "escape" to exit')
# plt.show()
numFrames = 0
fig=plt.figure()


def callback(data: pc2.PointCloud2):
    global numFrames
    fig.clf()
    
    numTrees = 0
    for x,y in pc2.read_points(data, field_names=("x", "y")): 
        fig.gca().plot(x, y, 'ro')
        numTrees += 1

    fig.gca().set_title(f"Keyframe ID: {data.header.seq} | Trees: {numTrees}")
    plt.draw()
    numFrames += 1

################
# Initialization
################

def listener():
    global obsRange
    rospy.init_node('kf_display')
    rospy.Subscriber("/keyframe_maker/keyframe", pc2.PointCloud2, callback)

    # plt.ion()
    # plt.show(block=False)
    plt.show()
    rospy.spin()


    plt.close()
    print(f"Number of frames seen: {numFrames}")

if __name__ == '__main__':
    listener()