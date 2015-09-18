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

#q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0] # simpler
#q1 = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0] # simpler2

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)
#r.loadObstacleModel ("puzzle_description","decor_very_easy","decor_very_easy")
r.loadObstacleModel ("puzzle_description","decor_easy","decor_easy")
r(q2)

ps.setInitialConfig (q1); ps.addGoalConfig (q2)


#ps.selectPathPlanner ("VisibilityPrmPlanner")
ps.selectPathValidation ("Dichotomy", 0.)
#ps.saveRoadmap ('/local/mcampana/devel/hpp/data/puzzle_veryEasy_PRM.rdm')
ps.saveRoadmap ('/local/mcampana/devel/hpp/data/puzzle_easy_RRT.rdm')

ps.solve ()

q1bis = q2; q2bis = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
ps.resetGoalConfigs ()
ps.setInitialConfig (q1bis); ps.addGoalConfig (q2bis)
ps.solve ()
ps.resetGoalConfigs (); ps.setInitialConfig (q1); ps.addGoalConfig (q2bis)
ps.solve ()
i = ps.numberPaths()-1

ps.pathLength(i)

ps.addPathOptimizer('RandomShortcut')
ps.optimizePath (i)
ps.pathLength(i+1)

ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (i)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)


len(ps.getWaypoints (i))

# Add light to scene
lightName = "li2"
r.client.gui.addLight (lightName, r.windowId, 0.005, [0.5,0.5,0.5,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
#r.client.gui.applyConfiguration (lightName, [-2,0,0,1,0,0,0])
r.client.gui.applyConfiguration (lightName, [2,0,0,1,0,0,0])
r.client.gui.refresh ()


## Video recording
pp.dt = 0.02
r.startCapture ("capture","png")
pp(ps.numberPaths()-1)
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4


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

