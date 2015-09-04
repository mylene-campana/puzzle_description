#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time
import numpy as np

robot = Robot ('puzzle') # object5
robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1.5, 1.5])
#robot.setJointBounds('base_joint_xyz', [-0.6, 0.6, -0.6, 0.6, -0.3, 1.0])
ps = ProblemSolver (robot)
cl = robot.client

# Configs : [x, y, z, qx1, qx2, qy1, qy2, qz1, qz2]
q1 = [0, 0, 0.9, 1, 0, 1, 0, 1, 0]; q2 = [0, 0, -0.9, 1, 0, 1, 0, 1, 0]
#q1 = [0, 0, 0.8, 1, 0, 1, 0, 1, 0]; q2 = [0, 0, 0, 1, 0, 1, 0, 1, 0] # DEBUG
q1 = [0, 0, 0.9, 0, 0, 0]; q2 = [0, 0, -0.9, 0, 0, 0]

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("puzzle_description","decor_very_easy","decor_very_easy")
r(q1)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load box obstacle in HPP for collision avoidance
cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')
#cl.obstacle.loadObstacleModel('puzzle_description','decor_easy','')
#cl.obstacle.loadObstacleModel('puzzle_description','decor','')

ps.selectPathPlanner ("VisibilityPrmPlanner") # 26min solve time
begin=time.time()
ps.solve ()
end=time.time()
print "Solving time: "+str(end-begin)
ps.pathLength(0)

ps.addPathOptimizer("GradientBased")
begin=time.time()
ps.optimizePath (0)
end=time.time()
print "Optim time: "+str(end-begin)
cl.problem.getIterationNumber ()
ps.pathLength(1)
#vPRM: 24 iter, 25.3s->21s, weighted Cost + comp linear error
#RRTc: 30 iter, 33.4s->14.9s, weighted Cost + comp linear error

begin=time.time()
ps.optimizePath (1)
end=time.time()
print "Optim time: "+str(end-begin)
cl.problem.getIterationNumber ()
ps.pathLength(2)

len(ps.getWaypoints (0))

ps.selectPathOptimizer('RandomShortcut')


r.startCapture ("capture","png")
pp(1)
r.stopCapture ()
#ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4


## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
robot.getJointOuterObjects('j_object_one')
robot.getJointOuterObjects('j_object_two')
robot.getJointOuterObjects('j_object_three')
robot.isConfigValid(q1)
robot.distancesToCollision()
r( ps.configAtDistance(0,5) )
ps.optimizePath (0)
ps.clearRoadmap ()
ps.resetGoalConfigs ()
from numpy import *
argmin(robot.distancesToCollision()[0])
robot.getJointNames ()
robot.getConfigSize ()

