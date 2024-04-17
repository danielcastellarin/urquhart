#!/usr/bin/env python3

import os
import sys
import argparse
import math
import rospy
from rospkg import RosPack
import time

# Get forest directory
rospy.init_node('just_trees_norad', anonymous=True)
forestDirectory = f"{RosPack().get_path('urquhart')}/forests/"
mapDirectory = f"{RosPack().get_path('urquhart')}/maps/"
if not os.path.isdir(forestDirectory):
    print('You do not have any forests!')
    sys.exit(0)
if not os.path.isdir(mapDirectory):
    print('Creating "maps" directory.')
    os.mkdir(mapDirectory)

# Parse input in case give forest and/or map name
parser = argparse.ArgumentParser()
parser.add_argument("-f", "--forest", help="The name of a specific forest to analyze")
parser.add_argument("-m", "--mapName", help="The name of the map to print the given forest as")
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

# Define the map file
if args.mapName is not None:
    pathToMap = f"{mapDirectory}{args.mapName}.txt"
elif args.forest is not None:
    pathToMap = f"{mapDirectory}{args.forest}.txt"
else:
    pathToMap = f"{mapDirectory}{forests[int(cmd)-1 if cmd.isdigit() and 1 <= int(cmd) <= len(forests) else 0]}"

# Write tree positions to map file
with open(pathToMap, "w") as f:
    f.write(f"{my_trees[0][0]} {my_trees[0][1]}")
    for x,y in my_trees[1:]:
        f.write(f"\n{x} {y}")
print("Done")
