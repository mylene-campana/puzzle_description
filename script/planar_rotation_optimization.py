#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import math
import time

robot = Robot ('puzzle')
ps = ProblemSolver (robot)
cl = robot.client

from hpp.gepetto import Viewer, PathPlayer
r = Viewer (ps)
pp = PathPlayer (robot.client, r)

values = [1.0, 0.8573, 0.4700, -0.0513, -0.5581, -0.9056, -0.7409, 0.2445, 0.7088, 0.9708, 0.99]

robot.setJointBounds('base_joint_xy', [-1, 1, -1, 1])
Q = []
for i in range(0, 11):
    invSin = -1 if i > 5 else 1
    Q.append ( [0, 0, values[i], invSin * math.sqrt (1 - values[i]**2)] )
    print Q[i]

r(Q[0])
for i in range(0, 11):
    r(Q[i])
    time.sleep (0.5)

#robot.isConfigValid(Q[0])

for i in range(0, 10):
    ps.setInitialConfig (Q[i]); ps.addGoalConfig (Q[i+1]); ps.solve (); ps.resetGoalConfigs ()

ps.setInitialConfig (Q[0]); ps.addGoalConfig (Q[10]); ps.solve ();



nInitialPath = ps.numberPaths()-1 #8
ps.pathLength(nInitialPath)

#ps.addPathOptimizer('RandomShortcut') #9
#ps.optimizePath (nInitialPath)
#ps.pathLength(ps.numberPaths()-1)

#ps.clearPathOptimizers()
ps.addPathOptimizer("GradientBased")
ps.optimizePath (nInitialPath)
ps.numberPaths()
ps.pathLength(ps.numberPaths()-1)

pp(ps.numberPaths()-1)


r(ps.configAtParam(0,2))
ps.getWaypoints (0)
ps.getWaypoints (ps.numberPaths()-1)

# plot paths
import numpy as np
dt = 0.1
nPath = ps.numberPaths()-1
lineNamePrefix = "optimized_path"
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = lineNamePrefix+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0],[0,1,0.3,1])
    r.client.gui.addToGroup (lineName, r.sceneName)

nPath = nInitialPath
lineNamePrefix = "initial_path"
for t in np.arange(0., cl.problem.pathLength(nPath), dt):
    lineName = lineNamePrefix+str(t)
    r.client.gui.addLine(lineName,[cl.problem.configAtParam(nPath, t)[0],cl.problem.configAtParam(nPath, t)[1],0],[cl.problem.configAtParam(nPath, t+dt)[0],cl.problem.configAtParam(nPath, t+dt)[1],0],[1,0.3,0,1])
    r.client.gui.addToGroup (lineName, r.sceneName)


# Add light to scene
lightName = "li"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [xStone,yEmu-0.5,zEmu,1,0,0,0])
r.client.gui.refresh ()
lightName = "li2"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [xStone-2,yEmu+0.5,zEmu,1,0,0,0])
r.client.gui.refresh ()
lightName = "l3"
r.client.gui.addLight (lightName, r.windowId, 0.001, [0.4,0.4,0.4,0.5])
r.client.gui.addToGroup (lightName, r.sceneName)
r.client.gui.applyConfiguration (lightName, [xStone+2,yEmu+0.5,zEmu,1,0,0,0])
r.client.gui.refresh ()


## Video recording
pp.dt = 0.01
pp.speed = 1.5
r(q1)
r.startCapture ("capture","png")
pp(nInitialPath)
#pp(ps.numberPaths()-1)
r(q11)
r.stopCapture ()

## ffmpeg commands
ffmpeg -r 50 -i capture_0_%d.png -r 25 -vcodec libx264 video.mp4
x=0; for i in *png; do counter=$(printf %03d $x); ln "$i" new"$counter".png; x=$(($x+1)); done
ffmpeg -r 50 -i new%03d.png -r 25 -vcodec libx264 video.mp4


# Load obstacles in HPP
cl.obstacle.loadObstacleModel('gravity_description','gravity_decor','')
cl.obstacle.loadObstacleModel('gravity_description','emu','')

## DEBUG commands
cl.obstacle.getObstaclePosition('obstacle_base')
cl.robot.getJointOuterObjects('CHEST_JOINT1')
cl.robot.getCurrentConfig()
cl.robot.setCurrentConfig(q3)
cl.robot.collisionTest()
res = cl.robot.distancesToCollision()
cl.problem.pathLength(2)
r(cl.problem.configAtParam(1,2))
cl.problem.optimizePath (1)
cl.problem.clearRoadmap ()
cl.problem.resetGoalConfigs ()


# config with x and y translations
q1 = [2, 0, 0, 1.0, 0.0, 0.0, 0.0]
q2 = [1.5, 0.8, 0, 0.707106781, 0, 0, 0.707106781]
q3 = [1, 1.4, 0, 0, 0, 0, 1]
q4 = [0.5, 1.7, 0, -0.707106781, 0, 0, 0.707106781]
q5 = [+0, 2, 0, -1, 0, 0, 0] # local extremum
q5 = [0, 2, 0, -0.99, 0.14003571, 0.013, 0.011]
q6 = [-0.5, 1.7, 0, -0.707106781, 0, 0, -0.707106781]
q7 = [-1, 1.4, 0, 0, 0, 0, -1]
q8 = [-1.5, 0.8, 0, 0.707106781, 0, 0, -0.707106781]
q9 = [-2, 0, 0, 1, 0, 0, 0]


q1 = [0, 0, 0, 1.0, 0.0, 0.0, 0.0]
q2 = [0, 0, 0, 0.707106781, 0, 0, 0.707106781]
q3 = [0, 0, 0, 0.011, 0.14003571, 0.013, 0.99]
q4 = [0, 0, 0, -0.707106781, 0, 0, 0.707106781]
q5 = [0, 0, 0, -0.99, 0.14003571, 0.013, 0.011]
q6 = [0, 0, 0, -0.707106781, 0, 0, -0.707106781]
q7 = [0, 0, 0, 0.011, 0.013, 0.14003571, -0.99]
q8 = [0, 0, 0, 0.707106781, 0, 0, -0.707106781]
q9 = [0, 0, 0, 0.99, 0.14003571, 0.013, 0.011]

q1 = [0, 0, 0, 1.0, 0.0, 0.0, 0.0]
q2 = [0, 0, 0, 0.707106781, 0, 0, 0.707106781]
q3 = [0, 0, 0, 0, 0, 0, 1]
q4 = [0, 0, 0, -0.707106781, 0, 0, 0.707106781]
q5 = [0, 0, 0, -0.99, 0.14003571, 0.013, 0.011]
q6 = [0, 0, 0, -0.707106781, 0, 0, -0.707106781]
q7 = [0, 0, 0, 0, 0, 0, -1]
q8 = [0, 0, 0, 0.707106781, 0, 0, -0.707106781

]
q9 = [0, 0, 0, 0.99, 0.14003571, 0.013, 0.011]

q1 = [0, 0, 0, 0.0, 0.0, -2*Pi]
q2 = [0, 0, 0, 0, 0, -3*Pi/2]
q3 = [0, 0, 0, 0, 0, -Pi]
q4 = [0, 0, 0, 0, 0, -Pi/2]
q5 = [0, 0, 0, 0, 0, 0]
q6 = [0, 0, 0, 0, 0, Pi/2]
q7 = [0, 0, 0, 0, 0, Pi]
q8 = [0, 0, 0, 0, 0, 3*Pi/2]
q9 = [0, 0, 0, 0, 0, 2*Pi]


q1 = [0, 0, 0, 1.0, 0.0, 0.0, 0.0]
q2 = [0, 0, 0, 0.707106781, 0, 0, 0.707106781]
q3 = [0, 0, 0, 0, 0, 0, 1]
q4 = [0, 0, 0, -0.707106781, 0, 0, 0.707106781]
q5 = [0, 0, 0, -0.99, 0.14003571, 0.013, 0.011]
q6 = [0, 0, 0, -0.707106781, 0, 0, -0.707106781]
q7 = [0, 0, 0, 0, 0, 0, -1]
q8 = [0, 0, 0, 0.707106781, 0, 0, -0.707106781]
q9 = [0, 0, 0, 0.99, 0.14003571, 0.013, 0.011]


# Optimization succeeds:
q1 = [0, 0, 0, 1.0, 0.0, 0.0, 0.0]
q2=[0.0, 0.0, 0.0, 0.8573289792052967, 0.5110043077178016, 0.0474382998474562, 0.04014009987092448]
q3=[0.0, 0.0, 0.0, 0.47002595717039686, 0.8761976030104256, 0.08134045836690892, 0.06882654169507678]
q4=[0.0, 0.0, 0.0, -0.05139523108351973, 0.9913748854243123, 0.09203276443212992, 0.07787387759641762]
q5=[0.0, 0.0, 0.0, -0.5581511991721069, 0.8236712340507641, 0.07646425360117025, 0.06470052227791329]
q6=[0.0, 0.0, 0.0, -0.9056431645733478, 0.420939551154707, 0.03907727653904272, 0.033065387840728454]
q7=[0.0, 0.0, 0.0, -0.7409238777634497, 0.6666775704281703, 0.06188998802924064, 0.05236845140935746]
q8=[0.0, 0.0, 0.0, 0.2445263320841038, 0.9625514882482619, 0.08935698863687985, 0.07560975961582142]
q9=[0.0, 0.0, 0.0, 0.708781393086984, 0.7002692759274627, 0.0650084224020931, 0.05500712664792493]
q10=[0.0, 0.0, 0.0, 0.9707913243458437, 0.23817079875118724, 0.02211022019858673, 0.01870864786034262]
q11 = [0, 0, 0, 0.99, 0.14003571, 0.013, 0.011]
s:
          0          0          0   -1.04471 -0.0969841 -0.0820635 q2
          0          0          0   -2.08942  -0.193968  -0.164127 q3
          0          0          0   -3.13413  -0.290952   -0.24619 q4
          0          0          0   -4.17884  -0.387937  -0.328254 q5
          0          0          0   -5.22355  -0.484921  -0.410317 q6
          0          0          0    -4.6151  -0.428435  -0.362522 q7
          0          0          0   -2.41025  -0.223752  -0.189328 q8
          0          0          0   -1.30783   -0.12141  -0.102731 q9
          0          0          0  -0.205402 -0.0190682 -0.0161346 q10
init grad: 
            0            0            0  2.22045e-16            0  5.55112e-17
            0            0            0  4.44089e-16  8.32667e-17 -2.77556e-17
            0            0            0            0 -8.32667e-17  2.77556e-17
            0            0            0 -1.33227e-15            0 -8.32667e-17
            0            0            0       3.6716     0.340847     0.288409 q6
            0            0            0 -2.22045e-16 -2.77556e-17 -2.77556e-17
            0            0            0  2.22045e-16 -5.55112e-17  2.77556e-17
            0            0            0  4.44089e-16  5.55112e-17 -2.77556e-17
            0            0            0 -4.44089e-16 -2.77556e-17 -5.55112e-17


# Optimization fails:
q1 = [0, 0, 0, 1.0, 0.0, 0.0, 0.0]
q2=[0.0, 0.0, 0.0, 0.8573289792052967, 0.5110043077178016, 0.0474382998474562, 0.04014009987092448]
q3=[0.0, 0.0, 0.0, 0.47002595717039686, 0.8761976030104256, 0.08134045836690892, 0.06882654169507678]
q4=[0.0, 0.0, 0.0, -0.05139523108351973, 0.9913748854243123, 0.09203276443212992, 0.07787387759641762]
q5=[0.0, 0.0, 0.0, -0.5581511991721069, 0.8236712340507641, 0.07646425360117025, 0.06470052227791329]
q6=[0.0, 0.0, 0.0, -0.9056431645733478, 0.420939551154707, 0.03907727653904272, 0.033065387840728454]
q7=[0.0, 0.0, 0.0, -0.7409238777634497, -0.6666775704281703, 0.06188998802924064, 0.05236845140935746]
q8=[0.0, 0.0, 0.0, 0.2445263320841038, -0.9625514882482619, 0.08935698863687985, 0.07560975961582142]
q9=[0.0, 0.0, 0.0, 0.708781393086984, -0.7002692759274627, 0.0650084224020931, 0.05500712664792493]
q10=[0.0, 0.0, 0.0, 0.9707913243458437, -0.23817079875118724, 0.02211022019858673, 0.01870864786034262]
q11 = [0, 0, 0, 0.99, 0.14003571, 0.013, 0.011]
s:
            0            0            0   0.00116771    -0.107179   -0.0734056 q2
            0            0            0   0.00233542    -0.214357    -0.146811 q3
            0            0            0   0.00350313    -0.321536    -0.220217 q4
            0            0            0   0.00467084    -0.428714    -0.293622 q5                 
            0            0            0   0.00583854    -0.535893    -0.367028 q6
            0            0            0 -0.000506135    -0.371548    -0.417482 q7
            0            0            0   0.00182928    -0.187253    -0.226973 q8
            0            0            0   0.00299699   -0.0951061    -0.131718 q9
            0            0            0    0.0041647  -0.00295867   -0.0364632 q10

init grad:
            0            0            0  2.22045e-16            0  5.55112e-17
            0            0            0  4.44089e-16  8.32667e-17 -2.77556e-17
            0            0            0            0 -8.32667e-17  2.77556e-17
            0            0            0 -1.33227e-15            0 -8.32667e-17
            0            0            0  -0.00703153     0.313689    0.0854862 q6
            0            0            0   0.00703153    0.0271584     0.202923
            0            0            0 -2.22045e-16            0  5.55112e-17
            0            0            0 -4.44089e-16            0 -5.55112e-17
            0            0            0   -0.0120993    -0.150395   -0.0744349
init grad(lambda_i=1):
            0            0            0  5.66222e-09  5.25644e-10  4.44776e-10
            0            0            0  5.32278e-09  4.94132e-10  4.18112e-10
            0            0            0 -6.58531e-10 -6.11338e-11 -5.17285e-11
            0            0            0 -5.94183e-09 -5.51601e-10 -4.66739e-10
            0            0            0     -1.06766     0.239573   0.00883607 q6
            0            0            0     0.149762    0.0160623     0.207471
            0            0            0       0.9179   -0.0852118   -0.0721023
            0            0            0 -3.14234e-09  2.91715e-10  2.46835e-10
            0            0            0     0.266575   -0.0781524   -0.0475107


init grad(lambda_i=1) success:
            0            0            0  5.66222e-09  5.25644e-10  4.44776e-10
            0            0            0  5.32278e-09  4.94132e-10  4.18112e-10
            0            0            0 -6.58531e-10 -6.11338e-11 -5.17285e-11
            0            0            0 -5.94183e-09 -5.51601e-10 -4.66739e-10
            0            0            0      1.42452     0.132243     0.111898 q6
            0            0            0      1.32918     0.123393     0.104409
            0            0            0      -0.9179   -0.0852118   -0.0721023
            0            0            0  3.14234e-09  2.91715e-10  2.46835e-10
            0            0            0    -0.746878   -0.0693353   -0.0586683

#-> diff de signe qui change tout ?
#biblio: rien sur l'optimisation de longueur (de chemin) sur la sphere des quaternions
