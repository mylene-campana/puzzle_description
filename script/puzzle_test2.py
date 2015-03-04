#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.gepetto import Viewer, PathPlayer
from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time

robot = Robot ('puzzle_robot') # object5
robot.setJointBounds('base_joint_xyz', [-0.1, 0.1, -0.1, 0.1, -0.1, 0.1])

ps = ProblemSolver (robot)
r = Viewer (ps)
cl = robot.client
pp = PathPlayer (cl, r)

# Patchwork of path
q1 = [0, 0, 0, 1, 0, 0, 0]
q2 = [0, 0, 0, 0.707106781, 0, 0, 0.707106781]
# equivalent to : [0, 0, 0, -0.707106781, 0, 0, -0.707106781] q7
ps.setInitialConfig (q1); ps.addGoalConfig (q2)
ps.solve (); ps.resetGoalConfigs ()

q3 = [0, 0, 0, 0, 0, 0, 1]
# equivalent to : [0, 0, 0, 0, 0, 0, -1] q8
ps.setInitialConfig (q2); ps.addGoalConfig (q3)
ps.solve (); ps.resetGoalConfigs ()

q4 = [0, 0, 0, -0.707106781, 0, 0, 0.707106781]
# equivalent to : [0, 0, 0, 0.707106781, 0, 0, -0.707106781] q9
ps.setInitialConfig (q3); ps.addGoalConfig (q4)
ps.solve (); ps.resetGoalConfigs ()

q5 = [0, 0, 0, -1, 0, 0, 0]
# equivalent to : [0, 0, 0, 1, 0, 0, 0] q1, q10
ps.setInitialConfig (q4); ps.addGoalConfig (q5)
ps.solve (); ps.resetGoalConfigs ()

"""
q6 = [0, 0, 0, -0.707106781, 0, 0, -0.707106781]
ps.setInitialConfig (q5); ps.addGoalConfig (q6)
ps.solve (); ps.resetGoalConfigs ()

q7 = [0, 0, 0, 0, 0, 0, -1]
ps.setInitialConfig (q6); ps.addGoalConfig (q7)
ps.solve (); ps.resetGoalConfigs ()

q8 = [0, 0, 0, 0.707106781, 0, 0, -0.707106781]
ps.setInitialConfig (q7); ps.addGoalConfig (q8)
ps.solve (); ps.resetGoalConfigs ()

q9 = [0, 0, 0, 1, 0, 0, 0]
ps.setInitialConfig (q8); ps.addGoalConfig (q9)
ps.solve (); ps.resetGoalConfigs ()
"""
# Only path to do a useless turn ! (Not the case if solved separately)
# r( ps.configAtParam(7,2.17) ) = [0.0, 0.0, 0.0, -0.9999559023922103, 0.0, 0.0, -0.00939112724262876]

ps.setInitialConfig (q1); ps.addGoalConfig (q5) # do a turn as expected
ps.solve ()

ps.optimizePath (4) # reduced to a point as expected

len(ps.getWaypoints (0))
ps.pathLength(5) # = 0


##############################"
# TEST quaternions x0 HRP2 + optimisation = meme resultat (-1) (un ou 2 tours)?
# Oui mais il faut imposer alpha = alpha_init et ne pas MàJ H1_
# les valeurs propres de la Hessienne sont bien >0
q1 = [0, 0, 0, 0.9999028455952614, -0.00643910090823595, -0.012362740316661774, 1.3620998722148461e-06]
q2 = [0, 0, 0, 0.8258496711518952, -0.2042733013060623, -0.19724860126354768, -0.4871732015735299]
ps.setInitialConfig (q1); ps.addGoalConfig (q2)
ps.solve (); ps.resetGoalConfigs ()

q3 = [0, 0, 0, 0.6659085913630002, -0.04107471639409278, 0.06948325706470262, -0.7416540248726288]
ps.setInitialConfig (q2); ps.addGoalConfig (q3)
ps.solve (); ps.resetGoalConfigs ()

q4 = [0, 0, 0, 0.8528688178462975, 0.1121872771500791, 0.09397345683903598, -0.5011963525414287]
ps.setInitialConfig (q3); ps.addGoalConfig (q4)
ps.solve (); ps.resetGoalConfigs ()

q5 = [0, 0, 0, 0.9999821377631646, -0.0058327069366009565, 0.0013052524286092148, 7.07493534969223e-07]
ps.setInitialConfig (q4); ps.addGoalConfig (q5)
ps.solve (); ps.resetGoalConfigs ()

ps.setInitialConfig (q1); ps.addGoalConfig (q5)
ps.solve ()
ps.pathLength(4)
pp(4)

ps.optimizePath (4)
ps.pathLength(5)
pp(5)

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

