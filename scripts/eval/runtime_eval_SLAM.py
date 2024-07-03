#!/usr/bin/env python3

import os
import math
import time
import pandas as pd
# import matplotlib
# matplotlib.use('GTK3Agg')

import rospy
from rospkg import RosPack
from std_msgs.msg import String
# import matplotlib.pyplot as plt

dfCols = ["Name", "# RP", "RPE", "ROE", "# Unique Landmarks", "LPE"]
runtimeCols = ["Frame", "Local GH", "Polygon Matching", "TF Estimation", "Landmark Discovery", "Graph Structure", "Graph Update", "Postprocess"]
superDF = pd.DataFrame(columns=dfCols)
runtimeDF = pd.DataFrame(columns=runtimeCols)
isGivenMap = False

class TimingData:
    def __init__(self, runCode, localGHTime, newMapTime, polyMatchTime, \
            tfEstTime, ldmkDiscTime, updateGraphStructTime, \
                graphUpdateTime, postProcessTime) -> None:
        self.code = runCode
        self.localGH = localGHTime
        self.newMap = newMapTime
        self.polyMatch = polyMatchTime
        self.tfEst = tfEstTime
        self.ldmkDisc = ldmkDiscTime
        self.updateGraphStruct = updateGraphStructTime
        self.graphUpdate = graphUpdateTime
        self.postProcess = postProcessTime

def gatherTimingData(outputPath: str, analyticsPath: str):
    timings = {}
    runtimeDF = pd.DataFrame(columns=runtimeCols)
    for line in open(outputPath+"/global/!timings.txt"):
        data = line.strip('\n')
        if not line: continue
        # print(data)
        frameID, runCode, localGHTime, newMapTime, polyMatchTime, \
            tfEstTime, ldmkDiscTime, updateGraphStructTime, \
                graphUpdateTime, postProcessTime = tuple(map(int, data.split("|")))
        timings[frameID-1] = (runCode, localGHTime, newMapTime, polyMatchTime, tfEstTime, ldmkDiscTime, updateGraphStructTime, graphUpdateTime, postProcessTime)
        dfRow = [frameID, localGHTime, polyMatchTime]
        dfRow.append(tfEstTime if runCode != 1 else None)
        dfRow += [ldmkDiscTime, updateGraphStructTime, graphUpdateTime] if runCode != 2 else [None, None, None]
        dfRow.append(postProcessTime if runCode != 3 else None)
        runtimeDF.loc[runtimeDF.shape[0]+1] = dfRow

    runtimeDF.to_string(f"{analyticsPath}/!runtimes.txt", index=False)

    ghTimesX = []
    ghTimesY = []
    polyMatchTimeX = []
    polyMatchTimeY = []
    tfEstX = []
    tfEstY = []
    ldmkDiscX = []
    ldmkDiscY = []
    graphStructX = []
    graphStructY = []
    graphUpdateX = []
    graphUpdateY = []
    postprocessX = []
    postprocessY = []

    for frameID, (code, gh, newmap, polyMatch, tfEst, ldmkDisc, graphStruct, graphUpdate, postprocess) in timings.items():
        ghTimesX.append(frameID)
        ghTimesY.append(gh)
        polyMatchTimeX.append(frameID)
        polyMatchTimeY.append(polyMatch)
        if code != 1:
            tfEstX.append(frameID)
            tfEstY.append(tfEst)
        if code != 2:
            ldmkDiscX.append(frameID)
            ldmkDiscY.append(ldmkDisc)
            graphStructX.append(frameID)
            graphStructY.append(graphStruct)
            graphUpdateX.append(frameID)
            graphUpdateY.append(graphUpdate)
        if code != 3:
            postprocessX.append(frameID)
            postprocessY.append(postprocess)


    # fig = plt.figure(1)
    # plt.plot(ghTimesX, ghTimesY, label="Local GH")
    # plt.show()
    # fig.plot(polyMatchTimeX, polyMatchTimeY, label="Polygon Matching")
    # fig.plot(tfEstX, tfEstY, label="TF Estimation")
    # fig.plot(ldmkDiscX, ldmkDiscY, label="Ldmk Discovery")
    # fig.plot(graphStructX, graphStructY, label="Graph Structure")
    # fig.plot(graphUpdateX, graphUpdateY, label="Graph Update")
    # fig.plot(postprocessX, postprocessY, label="Postprocessing")
    # fig.legend(loc="upper left")
    # fig.savefig(f"{analyticsPath}/!runtimes.png", bbox_inches='tight')
    # plt.close(fig)
    # f = open(f"{analyticsPath}/!gh.txt", )
    
    return timings
    

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
        if isGivenMap: finalLocalPoses.append(globalPoses[obsID])
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

def callback(msg: String):
    global isGivenMap
    if (msg.data == "shutdown"):
        with pd.option_context('display.max_rows', None,
                       'display.max_columns', None,
                       'display.precision', 3,
                       ):
            print(superDF)
        # superDF.to_string()
        rospy.sleep(5)
        rospy.signal_shutdown("Simulation Complete")
    else:
        # Define important file paths
        thisOutputPath = msg.data
        runName = os.path.basename(thisOutputPath)
        packagePath = os.path.dirname(os.path.dirname(thisOutputPath))
        analyticsDirectory = f'{packagePath}/analytics/'
        thisAnalyticsPath = analyticsDirectory + runName


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
        runtimes = gatherTimingData(thisOutputPath, thisAnalyticsPath)


        # Aggregate analytic data
        numPoses, numLdmks, numUpdates = 0, 0, 0
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

            numUpdates += 1
        
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

            superDF.loc[superDF.shape[0]+1] = (runName, numUpdates, totalPositionError / numPoses, totalAngleError / numPoses, numGTLandmarks, totalLdmkError / numLdmks)
        else:
            print(f'Run "{runName}" produced an invalid global map.')


########
# PREP #
########

rospy.init_node('eval_raw_SLAM', anonymous=True)
aggAnalyticsPath = f"{RosPack().get_path('urquhart')}/agg_analytics/"
if not os.path.isdir(aggAnalyticsPath):
    print('Creating "agg_analytics" directory.')
    os.mkdir(aggAnalyticsPath)
aggAnalyticsPath += rospy.get_param("/outputDirName", "test")
isGivenMap = rospy.get_param("/givenMap", False)
rospy.Subscriber("graph_builder/doneFlag", String, callback)
rospy.spin()
superDF.to_string(f"{aggAnalyticsPath}.txt")


