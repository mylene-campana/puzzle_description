#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time

robot = Robot ('puzzle') # object5
robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1.2, 1.2])
ps = ProblemSolver (robot)
cl = robot.client

q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
#q1 = [0.5, 0.3, 0.8, 0.7071067812, 0.0, 0.7071067812, 0.0]; q2 = [0.0, -0.6, -0.5, 1.0, 0.0, 0.0, 0.0]

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
print "Solving time: "+str(end-begin)

begin=time.time()
ps.optimizePath (1)
end=time.time()
print "Solving time: "+str(end-begin)

len(cl.problem.nodes ())
ps.pathLength(0)
ps.pathLength(1)
ps.pathLength(2)



from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
r.loadObstacleModel ("puzzle_description","decor_very_easy","decor_very_easy")
r(q1)

## DEBUG commands
cl.obstacle.getObstaclePosition('decor_base')
cl.robot.getJointOuterObjects('j_object_one')
cl.robot.getJointOuterObjects('j_object_two')
cl.robot.getJointOuterObjects('j_object_three')
cl.robot.setCurrentConfig(q1)
cl.robot.collisionTest()
cl.robot.distancesToCollision()
r( cl.problem.configAtDistance(0,5) )
cl.problem.optimizePath (0)
cl.problem.clearRoadmap ()
from numpy import *
argmin(cl.robot.distancesToCollision()[0])

