#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.puzzle_robot import Robot
from hpp.corbaserver import Client
import time
import sys
sys.path.append('/local/mcampana/devel/hpp/src/test-hpp/script')

robot = Robot ('puzzle_robot') # object5
robot.setJointBounds('base_joint_xyz', [-0.1, 0.1, -0.1, 0.1, -0.1, 0.1])

ps = ProblemSolver (robot)
r = Viewer (ps)
cl = robot.client
pp = PathPlayer (cl, r)

# Patchwork of path
q1 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)
cl.problem.solve (); cl.problem.resetGoalConfigs ()

q3 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]; q4 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
cl.problem.setInitialConfig (q3); cl.problem.addGoalConfig (q4)
cl.problem.solve (); cl.problem.resetGoalConfigs ()

q5 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]; q6 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
cl.problem.setInitialConfig (q5); cl.problem.addGoalConfig (q6)
cl.problem.solve ()

len(cl.problem.nodes ())
cl.problem.pathLength(0)
cl.problem.pathLength(1)

## DEBUG commands
begin=time.time(); end=time.time(); print "Solving time: "+str(end-begin)
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

