#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver

robot = Robot ('puzzle') # object5
robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1., 1.])
#robot.setJointBounds('base_joint_xyz', [-0.6, 0.6, -0.6, 0.6, -0.3, 1.0])
ps = ProblemSolver (robot)
cl = robot.client

q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
#q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] # simpler
#q1 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0] # simpler2

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
#r.loadObstacleModel ("puzzle_description","decor_very_easy","decor_very_easy")
r.loadObstacleModel ("puzzle_description","decor_easy","decor_easy")
r(q2) # r(q1)

#q1bis = q2; q2bis = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
#ps.resetGoalConfigs (); ps.setInitialConfig (q1bis); ps.addGoalConfig (q2bis); ps.solve ()
# ps.resetGoalConfigs (); ps.setInitialConfig (q1); ps.addGoalConfig (q2bis); ps.solve ()
#i = ps.numberPaths()-1

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

ps.selectPathPlanner ("VisibilityPrmPlanner")
#ps.selectPathValidation ("Dichotomy", 0.)
#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/puzzle_veryEasy_PRM.rdm')
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/puzzle_easy_RRT.rdm')
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/puzzle_easy_PRM1.rdm') # srand # problem ?
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/puzzle_easy_PRM1.rdm') # srand 1453110445(909sec) [COLL!]
#ps.readRoadmap ('/local/mcampana/devel/hpp/data/puzzle_easy_PRM2.rdm') # srand  # just after solve, GB OK. But after readroadmap+solve, segfault quaternions....
ps.readRoadmap ('/local/mcampana/devel/hpp/data/puzzle_easy_PRM_test1.rdm') #srand 1454520599 working0.05
# srand 1454521537 (no RM saved) works 7 -> 5, best 0.2
ps.solve ()
ps.pathLength(0)
len(ps.getWaypoints (0))



r(q1)

import numpy as np
"""
ps.addPathOptimizer("Prune")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)
len(ps.getWaypoints (ps.numberPaths()-1))
"""
ps.clearPathOptimizers()
cl.problem.setAlphaInit (0.05)
ps.addPathOptimizer("GradientBased")
ps.optimizePath (0)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

tGB = cl.problem.getTimeGB ()
timeValuesGB = cl.problem.getTimeValues ()
gainValuesGB = cl.problem.getGainValues ()
newGainValuesGB = ((1-np.array(gainValuesGB))*100).tolist() #percentage of initial length-value

ps.clearPathOptimizers()
ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)

ps.clearPathOptimizers()
ps.addPathOptimizer('PartialShortcut')
ps.optimizePath (0)
ps.pathLength(ps.numberPaths()-1)


pp(ps.numberPaths()-1)


## -------------------------------------
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
# OPTIMIZE AND Concatenate RS PRS values:
globalTimeValuesRS = []; globalGainValuesRS = []
globalTimeValuesPRS = []; globalGainValuesPRS = []
nbOpt = 50 # number of launchs of RS and PRS
optAndConcatenate (cl, ps, 0, nbOpt, 'RandomShortcut', globalTimeValuesRS, globalGainValuesRS)
optAndConcatenate (cl, ps, 0, nbOpt, 'PartialShortcut', globalTimeValuesPRS, globalGainValuesPRS)

nbPoints = 100 # number of points in graph
tVec = np.arange(0,tGB,tGB/nbPoints)
moyVectorRS = []; sdVectorRS = []; moyVectorPRS = []; sdVectorPRS = [];
computeMeansVector (nbOpt, tVec, moyVectorRS, sdVectorRS, globalTimeValuesRS, globalGainValuesRS)
computeMeansVector (nbOpt, tVec, moyVectorPRS, sdVectorPRS, globalTimeValuesPRS, globalGainValuesPRS)

tReduceVectorRS = []; meanReduceVectorRS = []; sdReduceVectorRS = [];
tReduceVectorPRS = []; meanReduceVectorPRS = []; sdReduceVectorPRS = [];
reducedVectors (tVec, moyVectorRS, sdVectorRS, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS)
reducedVectors (tVec, moyVectorPRS, sdVectorPRS, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS)

# Plot lengthGain (t);
plt.axis([-.005, tGB+0.005, 56, 101])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvSdPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, sdReduceVectorRS, '0.55', 0.8, 0.001)
plt = curvSdPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, sdReduceVectorPRS, '0.55', 0.8, 0.001)
plt = curvPlot (plt, tGB, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
plt.plot([0,tReduceVectorRS[0]], [100,100], 'r', linewidth=1.1)
plt = curvPlot (plt, tGB, tReduceVectorRS, meanReduceVectorRS, '*', 'r', 1.5)
plt.plot([0,tReduceVectorPRS[0]], [100,100], 'g', linewidth=0.8)
plt = curvPlot (plt, tGB, tReduceVectorPRS, meanReduceVectorPRS, '+', 'g', 1.5)
plt.show()

# For different alpha_init
import matplotlib.pyplot as plt
from plotfunctions import optAndConcatenate, getValIndex, computeMeansVector, reducedVectors, curvPlot, curvSdPlot
tmax = max(max(tGB,tGB2),max(max(tGB3,tGB4),tGB5))
plt.axis([-.005, tmax+0.005, 57, 101])
plt.xlabel('t (s)'); plt.ylabel('Relative remaining length (%)')
vectorLengthGB = len (timeValuesGB)
#plt.plot([0,tGB], [newGainValuesGB[vectorLengthGB-1],newGainValuesGB[vectorLengthGB-1]], 'b--')
plt.plot(0, 100, 'bo'); plt.plot([0,timeValuesGB[0]], [100,100], 'b', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB, newGainValuesGB, 'o', 'b', 1.5)
vectorLengthGB2 = len (timeValuesGB2)
plt.plot(0, 100, 'g*'); plt.plot([0,timeValuesGB2[0]], [100,100], 'g', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB2, newGainValuesGB2, '*', 'g', 1.5)
vectorLengthGB3 = len (timeValuesGB3)
plt.plot(0, 100, 'r+'); plt.plot([0,timeValuesGB3[0]], [100,100], 'r', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB3, newGainValuesGB3, '+', 'r', 1.5)
vectorLengthGB4 = len (timeValuesGB4)
plt.plot(0, 100, 'c+'); plt.plot([0,timeValuesGB4[0]], [100,100], 'c', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB4, newGainValuesGB4, '+', 'c', 1.5)
vectorLengthGB5 = len (timeValuesGB5)
plt.plot(0, 100, 'y+'); plt.plot([0,timeValuesGB5[0]], [100,100], 'y', linewidth=1.5)
plt = curvPlot (plt, tmax, timeValuesGB5, newGainValuesGB5, '+', 'y', 1.5)
plt.show()

## -------------------------------------

ps.clearPathOptimizers(); ps.addPathOptimizer("GradientBased")
cl.problem.setAlphaInit (0.05)
ps.optimizePath (0); tGB2 = cl.problem.getTimeGB ()
timeValuesGB2 = cl.problem.getTimeValues (); gainValuesGB2 = cl.problem.getGainValues ()
newGainValuesGB2 = ((1-np.array(gainValuesGB2))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.3)
ps.optimizePath (0); tGB3 = cl.problem.getTimeGB ()
timeValuesGB3 = cl.problem.getTimeValues (); gainValuesGB3 = cl.problem.getGainValues ()
newGainValuesGB3 = ((1-np.array(gainValuesGB3))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.02)
ps.optimizePath (0); tGB4 = cl.problem.getTimeGB ()
timeValuesGB4 = cl.problem.getTimeValues (); gainValuesGB4 = cl.problem.getGainValues ()
newGainValuesGB4 = ((1-np.array(gainValuesGB4))*100).tolist() #percentage of initial length-value

cl.problem.setAlphaInit (0.1)
ps.optimizePath (0); tGB5 = cl.problem.getTimeGB ()
timeValuesGB5 = cl.problem.getTimeValues (); gainValuesGB5 = cl.problem.getGainValues ()
newGainValuesGB5 = ((1-np.array(gainValuesGB5))*100).tolist() #percentage of initial length-value

## -------------------------------------



# Add light to scene
lightName = "li1"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [-2,0,0,1,0,0,0])
#r.client.gui.applyConfiguration (lightName, [2,0,0,1,0,0,0])
r.client.gui.refresh ()


## Video recording
import time
pp.dt = 0.02
pp.speed=0.7
r(q1)
r.startCapture ("capture","png")
r(q1); time.sleep(0.2)
r(q1)
pp(0)
#pp(ps.numberPaths()-1) # 2363
r(q2); time.sleep(1);
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %04d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%04d.png -r 25 -vcodec libx264 video.mp4
mencoder video.mp4 -channels 6 -ovc xvid -xvidencopts fixed_quant=4 -vf harddup -oac pcm -o video.avi
ffmpeg -i untitled.mp4 -vcodec libx264 -crf 24 video.mp4


# Load box obstacle in HPP for collision avoidance
cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')
#cl.obstacle.loadObstacleModel('puzzle_description','decor_easy','')
#cl.obstacle.loadObstacleModel('puzzle_description','decor','')


## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('j_object_one')
robot.isConfigValid(q1)
robot.distancesToCollision()
r( ps.configAtDistance(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
from numpy import *
argmin(robot.distancesToCollision()[0])


## Debug Optimization Tools ##############
num_log = 26123
from parseLog import parseCollConstrPoints, parseNodes

collConstrNodes = parseNodes (num_log, '189: qFree_ = ')
collNodes = parseNodes (num_log, '182: qColl = ')

contactPoints = parseCollConstrPoints (num_log, '77: contact point = (')
x1_J1 = parseCollConstrPoints (num_log, '96: x1 in R0 = (')
x2_J1 = parseCollConstrPoints (num_log, '97: x2 in R0 = (')
x1_J2 = parseCollConstrPoints (num_log, '116: x1 in J2 = (')
x2_J2 = parseCollConstrPoints (num_log, '117: x2 in J2 = (') #x2_J2 <=> contactPoints


## same with viewer !
from viewer_display_library_OPTIM import transformInConfig, plotPoints, plotPointsAndLines, plot2DBaseCurvPath, plotDofCurvPath, plotPointBodyCurvPath, plotBodyCurvPath
contactPointsViewer = transformInConfig (contactPoints)
x1_J1Viewer = transformInConfig (x1_J1)
x2_J1Viewer = transformInConfig (x2_J1)
x1_J2Viewer = transformInConfig (x1_J2)
x2_J2Viewer = transformInConfig (x2_J2)

# Plot points
sphereNamePrefix = "sphereContactPoints_"
plotPoints (r, sphereNamePrefix, contactPointsViewer, 0.015)
sphereSize=0.01
lineNamePrefix = "lineJ1_"; sphereNamePrefix = "sphereJ1_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J1Viewer, x2_J1Viewer, sphereSize)
lineNamePrefix = "lineJ2_"; sphereNamePrefix = "sphereJ2_"
plotPointsAndLines (r, lineNamePrefix, sphereNamePrefix, x1_J2Viewer, x2_J2Viewer, sphereSize)

# Plot trajectories
from viewer_display_library_OPTIM import plotPointBodyCurvPath
from hpp.corbaserver import Client
cl = robot.client
dt = 0.02

jointName = 'base_joint_xyz'
plotPointBodyCurvPath (r, cl, robot, dt, 0, jointName, [0,0,0], 'pathPoint_'+jointName, [1,0.1,0.1,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-1, jointName, [0,0,0], 'pathPointPRS1_'+jointName, [0,0.9,0.9,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-2, jointName, [0,0,0], 'pathPointRS_'+jointName, [0.1,0.1,1,1])
plotPointBodyCurvPath (r, cl, robot, dt, ps.numberPaths ()-3, jointName, [0,0,0], 'pathPointGB_'+jointName, [0.1,1,0.1,1])

r.client.gui.removeFromGroup ('pathPointGB_'+jointName, r.sceneName)
r.client.gui.removeFromGroup ('pathPointRS_'+jointName, r.sceneName)
r.client.gui.removeFromGroup ('pathPointPRS_'+jointName, r.sceneName)


# test function in cl.robot
jointPosition = robot.getJointPosition ('base_joint_xyz')
pointInJoint = [0.3,0,0]
posAtester = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)

r(q1)
robot.setCurrentConfig (q1)
sphereName = "machin"
r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()

qColl = [0.106262,0.168985,0.157725,0.244271,0.314375,-0.193254,0.896746]
qcoll = [0.0993319,0.17051,0.151557,0.234071,0.294556,-0.197664,0.905194]


# test function in cl.robot
jointPosition = robot.getJointPosition ('wrist_3_joint')
pointInJoint = [0.3,0,0]
posAtester = cl.robot.computeGlobalPosition (jointPosition, pointInJoint)

r(q1)
robot.setCurrentConfig (q1)
sphereName = "machin"
r.client.gui.addSphere (sphereName,0.03,[0.1,0.1,0.1,1]) # black
configSphere = posAtester [::]
configSphere.extend ([1,0,0,0])
r.client.gui.applyConfiguration (sphereName,configSphere)
r.client.gui.addToGroup (sphereName, r.sceneName)
r.client.gui.refresh ()

