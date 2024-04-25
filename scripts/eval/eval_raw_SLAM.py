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


########
# EVAL #
########

# Get keyframe mappings to associate robot's pose per keyframe
keyframeFiles = [filename for filename in os.listdir(pathToOutput+"/keyframe") if filename.endswith(".txt")]
keyframeToObsMap = {int(filename.split(".")[0])-1: int(open(pathToOutput+"/keyframe/"+filename).readline())-1 for filename in keyframeFiles}
# keyframe ID (from perspective of global map node) -> ground truth observation number
# Both the filename (for keyframe ID) and first line (for ground truth index) are logged as base-1 when they should be base 0



# Get ground truth robot poses
globalPoses = [tuple(map(float, line.strip('\n').split(" ")[:-2])) for line in open(pathToOutput+"/!gp.txt")]
startX,startY,startTheta = globalPoses[keyframeToObsMap[0]]
localPoses = [tuple(map(float, line.strip('\n').split(" ")[:-2])) for line in open(pathToOutput+"/!odom.txt")]
locStartX,locStartY,locStartTheta = localPoses[keyframeToObsMap[0]]
locSin, locCos = math.sin(-locStartTheta), math.cos(locStartTheta)
finalLocalPoses = len(keyframeToObsMap)*[None]
for kfID, obsID in keyframeToObsMap.items():
    currLocalX, currLocalY, currLocalTheta = localPoses[obsID]
    currLocalX -= locStartX
    currLocalY -= locStartY
    finalLocalPoses[kfID] = (currLocalX*locCos - currLocalY*locSin, currLocalX*locSin + currLocalY*locCos, currLocalTheta-locStartTheta)

# for p in finalLocalPoses:
#     print(p)
# print(len(finalLocalPoses))


# Get ground truth global observations from "global_obs" (x,y,radius)
globObsFiles = [filename for filename in os.listdir(pathToOutput+"/global_obs") if filename.endswith(".txt")]
globalObs = len(globObsFiles)*[None]
for filename in globObsFiles:
    globalObs[int(filename.split(".")[0])] = [tuple(map(float, line.split(" ")[:-1])) for line in open(pathToOutput+"/global_obs/"+filename)]

# Place all of the global landmark positions relative to the robot's starting position
groundTruthLocalLandmarks = []
globSin, globCos = math.sin(-startTheta), math.cos(startTheta)
for x,y in set(ldmk for i in range(len(globalObs)) for ldmk in globalObs[i]):
    x -= startX
    y -= startY
    groundTruthLocalLandmarks.append((x*globCos - y*globSin, x*globSin + y*globCos))


# Get robot pose and landmark positions over time
graphFiles = [filename for filename in os.listdir(pathToOutput+"/global/graph_nodes") if filename.endswith(".txt")]
graphInstanceErrors = len(graphFiles)*[None]
TWOPI = 2*math.pi
biggest = 0
# Per keyframe processed, I want to know the historical error in the robot's pose and the summed error of landmark positions that have been discovered
for filename in graphFiles:
    
    poseNodes, ldmkNodes = [], [] # nodes will be processed in discovery order
    for line in open(pathToOutput+"/global/graph_nodes/"+filename):
        if line: (poseNodes if line[0] == 'P' else ldmkNodes).append(tuple(map(float, [n for n in line.split(" ")[1:] if n != ""])))
    
    # Find error in each robot pose so far 
    poseErrors = []
    for i, (pX,pY,pT) in enumerate(poseNodes):
        # gtX, gtY, gtTheta = localPoses[keyframeToObsMap[i]]
        gtX, gtY, gtTheta = finalLocalPoses[i]
        # if not i: print(pX,pY,pT, gtX, gtY, gtTheta)
        xDiff, yDiff = pX - gtX, pY - gtY
        normDeg = (pT - gtTheta) % TWOPI
        # Euclidean distance b/w positions and absolute difference between angles
        poseErrors.append((math.sqrt(xDiff*xDiff + yDiff*yDiff), min(TWOPI-normDeg, normDeg)))

    if i > biggest: biggest = i

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

# Define the path to where the analytics should be saved
analyticName = args.outputName if args.outputName is not None else outputOptions[outputIdx].split('.')[0]
pathToAnalytics = f"{analyticsDirectory}{analyticName}"
if not os.path.isdir(pathToAnalytics):
    print(f'Creating subdirectory "{analyticName}".')
    os.mkdir(pathToAnalytics)
aggFile = open(f"{pathToAnalytics}/!slamAgg.txt", "w")  # probably errors from final frame?
pathToAnalytics += "/slam"
if not os.path.isdir(pathToAnalytics):
    print(f'Creating subdirectory "{analyticName}/slam".')
    os.mkdir(pathToAnalytics)
poseAggFile = open(f"{pathToAnalytics}/!poseAgg.txt", "w")
if not os.path.isdir(pathToAnalytics+"/poses"):
    print(f'Creating subdirectory "{analyticName}/slam/poses".')
    os.mkdir(pathToAnalytics+"/poses")
else:
    for filename in os.listdir(pathToAnalytics+"/poses"):
        os.remove(f"{pathToAnalytics}/poses/{filename}")
ldmkAggFile = open(f"{pathToAnalytics}/!ldmkAgg.txt", "w")
if not os.path.isdir(pathToAnalytics+"/ldmks"):
    print(f'Creating subdirectory "{analyticName}/slam/ldmks".')
    os.mkdir(pathToAnalytics+"/ldmks")
else:
    for filename in os.listdir(pathToAnalytics+"/ldmks"):
        os.remove(f"{pathToAnalytics}/ldmks/{filename}")




# df = pd.DataFrame(finalLocalPoses, columns=["x", "y", "theta"])
# df.to_string(f"{pathToAnalytics}/!locPose.txt")

# poseNodes = [(round(float(n), 6) for n in line.split(" ")[1:] if n != "") for line in open(f"{pathToOutput}/global/graph_nodes/{biggest+1}.txt") if line and line[0] == 'P']
# df = pd.DataFrame(poseNodes, columns=["x", "y", "theta"])
# df.to_string(f"{pathToAnalytics}/!mappedLocPose.txt")




numPoses, numLdmks = 0, 0
positionErrorSums, angleErrorSums, ldmkErrorSums = [], [], []
for i, (poseErrors, closestLdmksToMap) in enumerate(graphInstanceErrors):
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
    with open(f"{pathToAnalytics}/poses/{i}.txt", "w") as f:
        f.write(f"{poseErrors[0][0]} {poseErrors[0][1]}")
        for pE, aE in poseErrors[1:]: f.write(f"\n{pE} {aE}")
    
    # Landmarks (agg)
    ldmkErrorSums.append(sum(closestLdmksToMap))
    ldmkAggFile.write(f"{ldmkErrorSums[-1]}")
    numLdmks += len(closestLdmksToMap)

    # Landmarks (individual)
    with open(f"{pathToAnalytics}/ldmks/{i}.txt", "w") as f:
        f.write(f"{closestLdmksToMap[0]}")
        for d in closestLdmksToMap[1:]: f.write(f"\n{d}")

    
poseAggFile.close()
ldmkAggFile.close()

# Super aggregate results
aggFile.write(f"Sum of Robot Position Error (all frames): {sum(positionErrorSums)}\n")
aggFile.write(f"Sum of Robot Orientation Error (all frames): {sum(angleErrorSums)}\n")
aggFile.write(f"Number of robot poses (across graph iterations): {numPoses}\n")
aggFile.write(f"----------------------------------------------------------------\n")
aggFile.write(f"Average Robot Position Error: {sum(positionErrorSums) / numPoses}\n")
aggFile.write(f"Average Robot Orientation Error: {sum(angleErrorSums) / numPoses}\n")
aggFile.write(f"================================================================\n")
aggFile.write(f"Sum of Landmark Position Error (all frames): {sum(ldmkErrorSums)}\n")
aggFile.write(f"Number of landmark observations (across graph iterations): {numLdmks}\n")
aggFile.write(f"----------------------------------------------------------------\n")
aggFile.write(f"Average Landmark Position Error: {sum(ldmkErrorSums) / numLdmks}\n")


aggFile.close()


