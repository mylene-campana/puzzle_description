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
#robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1.2, 1.2])
robot.setJointBounds('base_joint_xyz', [-0.6, 0.6, -0.6, 0.6, -0.3, 1.0])
ps = ProblemSolver (robot)
cl = robot.client

#q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] # DEBUG
#q1 = [0.5, 0.3, 0.8, 0.7071067812, 0.0, 0.7071067812, 0.0]; q2 = [0.0, -0.6, -0.5, 1.0, 0.0, 0.0, 0.0]

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

begin=time.time()
ps.solve ()
end=time.time()
print "Solving time: "+str(end-begin)

begin=time.time()
ps.optimizePath (0)
end=time.time()
print "Optim time: "+str(end-begin)

begin=time.time()
ps.optimizePath (1)
end=time.time()
print "Optim time: "+str(end-begin)

len(cl.problem.nodes ())
len(ps.getWaypoints (0))
ps.pathLength(0)
ps.pathLength(1)
ps.pathLength(2)

qconstr0 = [0.0136688,0.409213,0.504901,-0.622972,-0.551659,-0.113669,-0.542824]
qconstr1 = [-0.000512377,0.397699,0.494112,-0.612723,-0.545536,-0.144922,-0.553135]
qconstr2 = [-0.393372,0.145117,0.404677,0.58047,0.129693,-0.801317,-0.0642316]
qconstr3 = [-0.357582,0.131914,0.367858,0.648702,0.121214,-0.748926,-0.0600321]

qconstr4 = [-0.407994,0.150511,0.419719,0.551315,0.132881,-0.821014,-0.0658105] # COLL
qconstr5 = [-0.407994,0.150511,0.419719,0.551315,0.132881,-0.821014,-0.0658105] # =4

qconstr6 = [-0.408971,0.150871,0.420724,0.549341,0.133089,-0.822294,-0.0659131]


qconstr1 = [-0.412662,0.160256,0.434482,0.505776,0.116219,-0.850206,-0.0885047]
qconstr2 = [-0.408678,0.150763,0.420422,0.549934,0.133026,-0.82191,-0.0658823] # COLL

qconstrx0 = np.array(ps.getWaypoints (0)[3])*(1-0.029617) + 0.029617*np.array(ps.getWaypoints (0)[4])

r([-0.408678,  0.150763,  0.420422,  0.53857 ,  0.131591, -0.813046, -0.065171])

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

