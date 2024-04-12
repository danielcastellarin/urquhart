#!/usr/bin/env python3

import os
import sys
import argparse
import math
import rospy
from rospkg import RosPack

# Get forest directory
rospy.init_node('avg_density')
forestDirectory = f"{RosPack().get_path('urquhart')}/forests/"
if not os.path.isdir(forestDirectory):
    print('You do not have any forests!')
    sys.exit(0)

# Parse input in case give forest name
parser = argparse.ArgumentParser()
parser.add_argument("-f", "--forest", help="The name of a specific forest to analyze")
args = parser.parse_args()

# Get the name of the forest to analyze
if args.forest is not None:
    pathToForest = f"{forestDirectory}{args.forest}.txt"
    if not os.path.exists(pathToForest):
        print(f'There is no forest with the name "{args.forest}".')
        sys.exit(0)

else:
    forests = [filename for filename in os.listdir(forestDirectory) if filename.endswith(".txt")]
    if not forests:
        print("There are no forests!")
        sys.exit(0)

    for i, name in enumerate(forests): print(f"{i+1}: {name}")
    cmd = input("Forest #: ")
    pathToForest = forestDirectory + forests[int(cmd)-1 if cmd.isdigit() and 1 <= int(cmd) <= len(forests) else 0]


# Get all the trees
print(f"Analyzing forest at path: {pathToForest}")
my_trees = []
with open(pathToForest) as f:
    for line in f:
        try:
            x,y,_ = tuple(map(float, line.split(" ")))
            my_trees.append((x, y))
        except: pass
print(f"Forest ingested: {len(my_trees)} trees.")


# Sum the distances of each tree to its nearest neighbor
nnDist = 0
for i, (treeX, treeY) in enumerate(my_trees):
    nearest = None
    for j, (x, y) in enumerate(my_trees):
        if i == j: continue
        xDiff, yDiff = treeX - x, treeY - y
        dist = xDiff*xDiff + yDiff*yDiff
        if nearest is None or dist < nearest: nearest = dist
    nnDist += math.sqrt(nearest)

print(f"Average distance between trees is {nnDist/len(my_trees)}m.")
