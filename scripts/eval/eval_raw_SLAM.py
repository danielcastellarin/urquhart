#!/usr/bin/env python3

import os
import sys
import argparse
import numpy as np
import math
import rospy
from rospkg import RosPack
import time
import pandas as pd

def gatherRunData(outputPath: str, hasGlobalMap = False):
    # Record which ground truth observation each keyframe as its base frame
    keyframeFiles = [filename for filename in os.listdir(outputPath+"/keyframe") if filename.endswith(".txt")]
    droppedKeyframes = set(int(fID.strip('\n')) for fID in open(outputPath+"/global/!droppedKeyframes.txt") if fID.strip('\n').isdigit()) if os.path.isfile(outputPath+"/global/!droppedKeyframes.txt") else set()
    keyframeToObsList = len(keyframeFiles)*[None]
    for filename in keyframeFiles:
        # the keyframe filename is logged as base-1 when it should be base 0
        keyframeID = int(filename.split(".")[0])-1
        if keyframeID not in droppedKeyframes:
            keyframeToObsList[keyframeID] = int(open(outputPath+"/keyframe/"+filename).readline())
    keyframeToObs = [id for id in keyframeToObsList if id is not None]
    # keyframe ID (from perspective of global map node) -> ground truth observation number


    # Get ground truth robot poses
    globalPoses = [tuple(map(float, line.strip('\n').split(" ")[:-2])) for line in open(outputPath+"/!gp.txt")]
    startX,startY,startTheta = globalPoses[keyframeToObs[0]]
    localPoses = [tuple(map(float, line.strip('\n').split(" ")[:-2])) for line in open(outputPath+"/!odom.txt")]
    locStartX,locStartY,locStartTheta = localPoses[keyframeToObs[0]]
    locSin, locCos = math.sin(-locStartTheta), math.cos(locStartTheta)
    finalLocalPoses = []
    for obsID in keyframeToObs:
        if hasGlobalMap: finalLocalPoses.append(globalPoses[obsID])
        else:
            currLocalX, currLocalY, currLocalTheta = localPoses[obsID]
            currLocalX -= locStartX
            currLocalY -= locStartY
            finalLocalPoses.append((currLocalX*locCos - currLocalY*locSin, currLocalX*locSin + currLocalY*locCos, currLocalTheta-locStartTheta))


    # Get ground truth global observations from "global_obs" (x,y,radius)
    globObsFiles = [filename for filename in os.listdir(outputPath+"/global_obs") if filename.endswith(".txt")]
    globalObs = len(globObsFiles)*[None]
    for filename in globObsFiles:
        globalObs[int(filename.split(".")[0])] = [tuple(map(float, line.split(" ")[:-1])) for line in open(outputPath+"/global_obs/"+filename)]

    # Place all of the global landmark positions relative to the robot's starting position
    groundTruthLocalLandmarks = []
    globSin, globCos = math.sin(-startTheta), math.cos(startTheta)
    for x,y in set(ldmk for i in range(len(globalObs)) for ldmk in globalObs[i]):
        if hasGlobalMap: groundTruthLocalLandmarks.append((x, y))
        else:  
            x -= startX
            y -= startY
            groundTruthLocalLandmarks.append((x*globCos - y*globSin, x*globSin + y*globCos))


    # Get robot pose and landmark positions over time
    graphFiles = [filename for filename in os.listdir(outputPath+"/global/graph_nodes") if filename.endswith(".txt")]
    graphInstanceErrors = len(graphFiles)*[None]
    TWOPI = 2*math.pi
    # Per keyframe processed, I want to know the historical error in the robot's pose and the summed error of landmark positions that have been discovered
    for filename in graphFiles:
        
        poseNodes, ldmkNodes = [], [] # nodes will be processed in discovery order
        for line in open(outputPath+"/global/graph_nodes/"+filename):
            if line: (poseNodes if line[0] == 'P' else ldmkNodes).append(tuple(map(float, [n for n in line.split(" ")[1:] if n != ""])))
        
        # Find error in each robot pose so far 
        poseErrors = []
        for i, (pX,pY,pT) in enumerate(poseNodes):
            gtX, gtY, gtTheta = finalLocalPoses[i]
            xDiff, yDiff = pX - gtX, pY - gtY
            normDeg = (pT - gtTheta) % TWOPI
            # Euclidean distance b/w positions and absolute difference between angles
            poseErrors.append((math.sqrt(xDiff*xDiff + yDiff*yDiff), min(TWOPI-normDeg, normDeg)))

        # Find distance to closest ground truth landmarks to what is on the map
        closestLdmksToMap = []
        for x,y in ldmkNodes:
            xDiff, yDiff = x - groundTruthLocalLandmarks[0][0], y - groundTruthLocalLandmarks[0][1]
            closestDist = (xDiff*xDiff)+(yDiff*yDiff)
            for gtX,gtY in groundTruthLocalLandmarks[1:]:
                xDiff, yDiff = x - gtX, y - gtY
                dist = (xDiff*xDiff)+(yDiff*yDiff)
                if dist < closestDist: closestDist = dist
            closestLdmksToMap.append(math.sqrt(closestDist))

        graphInstanceErrors[int(filename.split(".")[0])-1] = (poseErrors, closestLdmksToMap)

    return graphInstanceErrors, len(groundTruthLocalLandmarks)


def prepAnalyticType(thisAnalyticsPath: str, runName: str, analyticType: str):
    subAggFile = open(f"{thisAnalyticsPath}/!{analyticType}Agg.txt", "w")
    subfolderPath = f"{thisAnalyticsPath}/{analyticType}s"
    if not os.path.isdir(subfolderPath):
        print(f'Creating subdirectory "{runName}/slam/{analyticType}s".')
        os.mkdir(subfolderPath)
    else:
        for filename in os.listdir(subfolderPath):
            os.remove(f"{subfolderPath}/{filename}")

    return subAggFile

########
# PREP #
########

# Get output directory
rospy.init_node('eval_raw_SLAM', anonymous=True)
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
    runName = args.outputName
    thisOutputPath = outputDirectory + runName
    if not os.path.exists(thisOutputPath):
        print(f'There is no forest with the name "{args.forest}".')
        sys.exit(0)
    thisAnalyticsPath = analyticsDirectory + runName
else:
    outputOptions = [filename for filename in os.listdir(outputDirectory) if not filename.endswith(".txt")]
    if not outputOptions:
        print("There are no output results!")
        sys.exit(0)

    for i, name in enumerate(outputOptions): print(f"{i+1}: {name}")
    cmd = input("Output #: ")
    outputIdx = int(cmd)-1 if cmd.isdigit() and 1 <= int(cmd) <= len(outputOptions) else 0
    thisOutputPath = outputDirectory + outputOptions[outputIdx]
    thisAnalyticsPath = outputDirectory + outputOptions[outputIdx]


print(f"Analyzing forest at path: {thisOutputPath}")
isGivenMap = input("Type 'yes' if map used during run: ") == 'yes'


########
# EVAL #
########

# Define where the analytics will be saved
if not os.path.isdir(analyticsDirectory):
    print('Creating "analytics" directory.')
    os.mkdir(analyticsDirectory)
if not os.path.isdir(thisAnalyticsPath):
    print(f'Creating directory for "{runName}" analytics.')
    os.mkdir(thisAnalyticsPath)
aggFile = open(f"{thisAnalyticsPath}/!slamAgg.txt", "w")  # probably errors from final frame?
thisAnalyticsPath += "/slam"
if not os.path.isdir(thisAnalyticsPath):
    print(f'Creating subdirectory "{runName}/slam".')
    os.mkdir(thisAnalyticsPath)
poseAggFile = prepAnalyticType(thisAnalyticsPath, runName, "pose")
ldmkAggFile = prepAnalyticType(thisAnalyticsPath, runName, "ldmk")

# Aggregate analytic data
numPoses, numLdmks = 0, 0
positionErrorSums, angleErrorSums, ldmkErrorSums = [], [], []
# positionErrorMins, angleErrorMins, ldmkErrorMins = [], [], []
# positionErrorMaxs, angleErrorMaxs, ldmkErrorMaxs = [], [], []
graphErrors, numGTLandmarks = gatherRunData(thisOutputPath, isGivenMap)
for i, (poseErrors, closestLdmksToMap) in enumerate(graphErrors):
    if i: 
        poseAggFile.write("\n")
        ldmkAggFile.write("\n")
    
    # Poses (agg)
    positionErrs, angleErrs = zip(*poseErrors)
    positionErrorSums.append(sum(positionErrs))
    angleErrorSums.append(sum(angleErrs))
    numPoses += len(positionErrs)
    poseAggFile.write(f"{positionErrorSums[-1]} {angleErrorSums[-1]}")

    # Poses (individual)
    with open(f"{thisAnalyticsPath}/poses/{i}.txt", "w") as f:
        f.write("\n".join(f"{pE} {aE}" for pE, aE in poseErrors))
    
    # Landmarks (agg)
    ldmkErrorSums.append(sum(closestLdmksToMap))
    ldmkAggFile.write(f"{ldmkErrorSums[-1]}")
    numLdmks += len(closestLdmksToMap)

    # Landmarks (individual)
    with open(f"{thisAnalyticsPath}/ldmks/{i}.txt", "w") as f:
        f.write("\n".join(map(str, closestLdmksToMap)))

poseAggFile.close()
ldmkAggFile.close()

# Super aggregate results
if numPoses and numLdmks:
    totalPositionError, totalAngleError, totalLdmkError = sum(positionErrorSums), sum(angleErrorSums), sum(ldmkErrorSums)
    aggFile.write(f"Sum of Robot Position Error (all frames): {totalPositionError}\n")
    aggFile.write(f"Sum of Robot Orientation Error (all frames): {totalAngleError}\n")
    aggFile.write(f"Number of robot poses (across graph iterations): {numPoses}\n")
    aggFile.write(f"----------------------------------------------------------------\n")
    aggFile.write(f"Average Robot Position Error: {totalPositionError / numPoses}\n")
    aggFile.write(f"Average Robot Orientation Error: {totalAngleError / numPoses}\n")
    aggFile.write(f"================================================================\n")
    aggFile.write(f"Sum of Landmark Position Error (all frames): {totalLdmkError}\n")
    aggFile.write(f"Number of landmark observations (across graph iterations): {numLdmks}\n")
    aggFile.write(f"----------------------------------------------------------------\n")
    aggFile.write(f"Average Landmark Position Error: {totalLdmkError / numLdmks}\n")
    aggFile.close()
else:
    print(f'Run "{runName}" produced an invalid global map.')
