#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import math
import rospy
from rospkg import RosPack
import time

# Get output directory
rospy.init_node('eval_keyframe_accuracy', anonymous=True)
outputDirectory = f"{RosPack().get_path('urquhart')}/output/"
analyticsDirectory = f"{RosPack().get_path('urquhart')}/analytics/"
if not os.path.isdir(outputDirectory):
    print('You do not have any logged outputs!')
    sys.exit(0)
if not os.path.isdir(analyticsDirectory):
    print('Creating "analytics" directory.')
    os.mkdir(analyticsDirectory)

# Parse input in case given output name
parser = argparse.ArgumentParser()
parser.add_argument("-o", "--outputName", help="The name of a specific output log to analyze.")
args = parser.parse_args()

# Get the name of the output to analyze
if args.outputName is not None:
    pathToOutput = f"{outputDirectory}{args.outputName}.txt"
    if not os.path.exists(pathToOutput):
        print(f'There is no forest with the name "{args.forest}".')
        sys.exit(0)

else:
    outputOptions = [filename for filename in os.listdir(outputDirectory) if not filename.endswith(".txt")]
    if not outputOptions:
        print("There are no output results!")
        sys.exit(0)

    for i, name in enumerate(outputOptions): print(f"{i+1}: {name}")
    cmd = input("Output #: ")
    outputIdx = int(cmd)-1 if cmd.isdigit() and 1 <= int(cmd) <= len(outputOptions) else 0
    pathToOutput = outputDirectory + outputOptions[outputIdx]


print(f"Analyzing forest at path: {pathToOutput}")

# Get ground truth global poses of robot (to apply to all estimated map positions)
globalPoses = [tuple(map(float, line.strip('\n').split(" ")[:-2])) for line in open(pathToOutput+"/!gp.txt")]


# Get ground truth global observations from "global_obs" (x,y,radius)
globObsFiles = [filename for filename in os.listdir(pathToOutput+"/global_obs") if filename.endswith(".txt")]
globalObs = len(globObsFiles)*[None]
for filename in globObsFiles:
    globalObs[int(filename.split(".")[0])] = [tuple(map(float, line.split(" ")[:-1])) for line in open(pathToOutput+"/global_obs/"+filename)]


# Get observed trees in each keyframe and map each keyframe to a set of global observations
keyframeFiles = [filename for filename in os.listdir(pathToOutput+"/keyframe") if filename.endswith(".txt")]
keyframeToObsMap = {}
keyframes = len(keyframeFiles)*[None]
for filename in keyframeFiles:
    fileNum = filename.split(".")[0]
    idx = int(fileNum)-1
    
    thisKeyframe = []
    with open(pathToOutput+"/keyframe/"+filename) as f:
        for i, line in enumerate(f):
            if not i: keyframeToObsMap[idx] = tuple(map(int, line.split(" ")[:-1]))
            else: thisKeyframe.append(tuple(map(float, line.split(" "))))
    keyframes[idx] = thisKeyframe


# Aggregate ground truth observations by keyframe, convert into a local observation taken at the base of the keyframe 
groundTruths = len(keyframeFiles)*[None]
for kf, observs in keyframeToObsMap.items():
    baseFrame = observs[0]
    globSin, globCos = math.sin(-globalPoses[baseFrame][2]), math.cos(globalPoses[baseFrame][2])
    gTruth = []
    for x,y in set(ldmk for obs in observs for ldmk in globalObs[obs]):
        x -= globalPoses[baseFrame][0]
        y -= globalPoses[baseFrame][1]
        gTruth.append((x*globCos - y*globSin, x*globSin + y*globCos))
    groundTruths[kf] = gTruth


assert(len(groundTruths) == len(keyframes))


# Define the path to where the analytics should be saved
pathToAnalytics = f"{analyticsDirectory}{args.outputName if args.outputName is not None else outputOptions[outputIdx].split('.')[0]}"
if not os.path.isdir(pathToAnalytics):
    print(f'Creating subdirectory "{args.outputName if args.outputName is not None else outputOptions[outputIdx].split(".")[0]}".')
    os.mkdir(pathToAnalytics)
aggFile = open(f"{pathToAnalytics}/!kfAgg.txt", "w")
pathToAnalytics += "/keyframe"
if not os.path.isdir(pathToAnalytics):
    print('Creating subdirectory "keyframe".')
    os.mkdir(pathToAnalytics)


for i in range(len(keyframes)):
    thisKeyframe, thisGroundTruth = keyframes[i], groundTruths[i]
    ldmkDiff = len(thisKeyframe) - len(thisGroundTruth)
    closestLdmksToGroundTruth = []

    for gtX,gtY in thisGroundTruth:
        xDiff, yDiff = gtX - thisKeyframe[0][0], gtY - thisKeyframe[0][1]
        closestDist = (xDiff*xDiff)+(yDiff*yDiff)
        for kX,kY in thisKeyframe[1:]:
            xDiff, yDiff = gtX - kX, gtY - kY
            dist = (xDiff*xDiff)+(yDiff*yDiff)
            if dist < closestDist: closestDist = dist
        closestLdmksToGroundTruth.append(math.sqrt(closestDist))

    # Save aggregated data
    if i: aggFile.write("\n")
    aggFile.write(f"{i} {sum(closestLdmksToGroundTruth)} {ldmkDiff}")

    # Write individual nearest neighbor distances to separate file 
    with open(f"{pathToAnalytics}/{i}.txt", "w") as f:
        f.write(f"{closestLdmksToGroundTruth[0]}")
        for d in closestLdmksToGroundTruth[1:]:
            f.write(f"\n{d}")
    
aggFile.close()
    

