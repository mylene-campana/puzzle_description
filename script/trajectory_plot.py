#/usr/bin/env python
# Author : Mylene Campana
# Script to plot some graphs after launching a corbaserver and solving 
# the problem.

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math

dt = 0.02 # global drawing step size


# Plot trajectory and concaves obstacles for potential scenario. nPath is the path number.
def planarConcObstaclesPlot (cl, nPath, path, nPlot):
    Tvec = np.arange(0., cl.problem.pathLength(nPath), dt)
    #plt.gcf().gca().add_artist(plt.Circle((0,0),.5,color='r'))
    plt.gcf().gca().add_artist(plt.Circle((0.9,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(0.9, 0.4, r'obst1', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.5,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(1.5, 0.3, r'obst1bis', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.2,-3.5),1.2,color='green'))
    plt.text(1.2, -3.5, r'obst2', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2.6,2.5),1.2,color='green'))
    plt.text(2.6, 2.5, r'obst6', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.5,1.2),.6,color='gray'))
    plt.text(-2.5, 1.2, r'obst4', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1,4),.6,color='gray'))
    plt.text(-1, 4, r'obst5', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.8,-2.2),1.2,color='b'))
    plt.text(-2.8, -2.2, r'obst3', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((3.8,-1),.6,color='b'))
    plt.text(3.8, -1, r'obst7', fontsize=11)
    plt.gcf().gca().add_artist(plt.Rectangle((-0.4,-0.9),0.8,1.8,color='r'))
    plt.text(0, 0, r'obst_base', fontsize=11)
    #plt.gcf().gca().add_artist(plt.Rectangle((0.6,0.1),1.4,0.6,color='r')) # (1.3,0.4)
    plt.gcf().gca().add_artist(plt.Circle((1.5,-1.8),0.6,color='b'))
    plt.text(1.5, -1.8, r'obst8', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2., 1.2),0.4,color=(0, 0.5, 1))) # light_blue
    plt.text(2., 1.2, r'obst9', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.6,1.4),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.6, 1.4, r'obst10', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.5,2.3),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.5, 2.3, r'obst11', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-0.9,3.),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-0.9, 3., r'obst12', fontsize=11)
    
    i = 0
    for n in cl.problem.nodes() :
        if i>1: # avoid 2 first nodes (init and goal)
            plt.plot(n[0], n[1], 'ro')
            plt.text(n[0]+.02, n[1], r'q_new %i' %(i), fontsize=11)
        i=i+1
    
    for t in Tvec:
        plt.plot([cl.problem.configAtParam(nPath, t)[0], \
                     cl.problem.configAtParam(nPath, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath, t)[1], \
                     cl.problem.configAtParam(nPath, t+dt)[1]], 'k', linewidth=2.0, label="optim"  if t == 0. else "")
    
    if (nPath == 1): # plot also initial path to compare
        for t in np.arange(0., cl.problem.pathLength(0), dt):
            plt.plot([cl.problem.configAtParam(0, t)[0], \
                     cl.problem.configAtParam(0, t+dt)[0]], \
                     [cl.problem.configAtParam(0, t)[1], \
                     cl.problem.configAtParam(0, t+dt)[1]], 'r', label="init." if t == 0. else "")
    plt.legend(loc=4)
    plt.axis([-5, 5, -5, 5])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory'); plt.grid()
    plt.plot(cl.problem.nodes()[0][0], cl.problem.nodes()[0][1], 'go')
    plt.plot(cl.problem.nodes()[1][0], cl.problem.nodes()[1][1], 'go')
    plt.text(cl.problem.nodes()[0][0]+.2, cl.problem.nodes()[0][1]+.1, r'q_init', fontsize=12)
    plt.text(cl.problem.nodes()[1][0]+.2, cl.problem.nodes()[1][1]+.1, r'q_end', fontsize=12)
    plt.show()
    #plt.savefig(path+'plannarPlotTrajObstS'+str(nPlot))

# --------------------------------------------------------------------#

# Plot trajectory and concaves obstacles for potential scenario. nPath is the path number.
# And special nodes in another color, for example connection nodes in visibility-prm
def planarConcObstaclesSpecNodesPlot (cl, nPath, specialNodes):
    Tvec = np.arange(0., cl.problem.pathLength(nPath), dt)
    
    plt.gcf().gca().add_artist(plt.Circle((0.9,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(0.9, 0.4, r'obst1', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.5,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(1.5, 0.3, r'obst1bis', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.2,-3.5),1.2,color='green'))
    plt.text(1.2, -3.5, r'obst2', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2.6,2.5),1.2,color='green'))
    plt.text(2.6, 2.5, r'obst6', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.5,1.2),.6,color='gray'))
    plt.text(-2.5, 1.2, r'obst4', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1,4),.6,color='gray'))
    plt.text(-1, 4, r'obst5', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.8,-2.2),1.2,color='b'))
    plt.text(-2.8, -2.2, r'obst3', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((3.8,-1),.6,color='b'))
    plt.text(3.8, -1, r'obst7', fontsize=11)
    plt.gcf().gca().add_artist(plt.Rectangle((-0.4,-0.9),0.8,1.8,color='r'))
    plt.text(0, 0, r'obst_base', fontsize=11)
    #plt.gcf().gca().add_artist(plt.Rectangle((0.6,0.1),1.4,0.6,color='r')) # (1.3,0.4)
    plt.gcf().gca().add_artist(plt.Circle((1.5,-1.8),0.6,color='b'))
    plt.text(1.5, -1.8, r'obst8', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2., 1.2),0.4,color=(0, 0.5, 1))) # light_blue
    plt.text(2., 1.2, r'obst9', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.6,1.4),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.6, 1.4, r'obst10', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.5,2.3),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.5, 2.3, r'obst11', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-0.9,3.),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-0.9, 3., r'obst12', fontsize=11)
    
    i = 0
    for n in cl.problem.nodes() :
        if i>1: # avoid 2 first nodes (init and goal)
            plt.plot(n[0], n[1], 'ro')
            plt.text(n[0]+.02, n[1], r'qNew%i' %(i), fontsize=8)
        i=i+1
    
    i = 0
    for n in specialNodes :
        plt.plot(n[0], n[1], 'bo')
        plt.text(n[0]+.02, n[1], r'qCol%i' %(i), fontsize=8)
        i = i+1
    
    for t in Tvec:
        plt.plot([cl.problem.configAtParam(nPath, t)[0], \
                     cl.problem.configAtParam(nPath, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath, t)[1], \
                     cl.problem.configAtParam(nPath, t+dt)[1]], 'k',  label="optim." if t == 0. else "")
    
    if (nPath == 1): # plot also initial path to compare
        for t in np.arange(0., cl.problem.pathLength(0), dt):
            plt.plot([cl.problem.configAtParam(0, t)[0], \
                     cl.problem.configAtParam(0, t+dt)[0]], \
                     [cl.problem.configAtParam(0, t)[1], \
                     cl.problem.configAtParam(0, t+dt)[1]], 'r', label="init." if t == 0. else "")
    
    plt.legend()
    plt.axis([-5, 5, -5, 5])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory'); plt.grid()
    plt.plot(cl.problem.nodes()[0][0], cl.problem.nodes()[0][1], 'go')
    plt.plot(cl.problem.nodes()[1][0], cl.problem.nodes()[1][1], 'go')
    plt.text(cl.problem.nodes()[0][0]+.1, cl.problem.nodes()[0][1]+.1, r'q_init', fontsize=10)
    plt.text(cl.problem.nodes()[1][0]+.1, cl.problem.nodes()[1][1]+.1, r'q_end', fontsize=10)
    plt.show()

# --------------------------------------------------------------------#

# Draw only minDistance
def minDistancePlot (cl):
    Tvec = np.arange(0., cl.problem.pathLength(0), dt)
    argminFound=0; minDistance=0; resDistance=[]
    for t in Tvec:
        cl.robot.setCurrentConfig(cl.problem.configAtParam(0,t))
        resDistance=cl.robot.distancesToCollision()
        minDistance=min(resDistance[0])
        plt.plot(t, minDistance, '.')
        argminFound=np.argmin(resDistance[0])
        print "minDistancepair for distance: "+str(minDistance)+" (time: "+str(t)+") are:"
        print cl.robot.distancesToCollision()[1][argminFound]
        print cl.robot.distancesToCollision()[2][argminFound]
    
    plt.axis([min(Tvec),max(Tvec),-0.05,0.8])
    plt.xlabel('t')
    plt.ylabel('distance_obst')
    plt.grid()
    plt.show()

# --------------------------------------------------------------------#

# Draw specified DoF(t) of the nPath (0 or 1)
def dofPlot (cl, nPath, n1, n2, n3, n4):
    t_vec = np.arange(0., cl.problem.pathLength(nPath), dt) # all
    t_ssvec = np.arange(4.2, 6., dt) # subvector
    Tvec = t_vec [::]
    plt.subplot(221)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtParam(nPath, t)[n1], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtParam(nPath, t)[n1], \
                                 cl.problem.configAtParam(nPath, t+dt)[n1]], 'k-')
    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n1))
    plt.grid()
    plt.subplot(222)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtParam(0, t)[42], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtParam(nPath, t)[n2], \
                                 cl.problem.configAtParam(nPath, t+dt)[n2]], 'k-')
    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n2))
    plt.grid()
    plt.subplot(223)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtParam(0, t)[43], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtParam(nPath, t)[n3], \
                                 cl.problem.configAtParam(nPath, t+dt)[n3]], 'k-')
    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n3))
    plt.grid()
    plt.subplot(224)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtParam(0, t)[45], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtParam(nPath, t)[n4], \
                                 cl.problem.configAtParam(nPath, t+dt)[n4]], 'k-')
    plt.axis([min(Tvec),max(Tvec),-2,2])
    plt.xlabel('t')
    plt.ylabel('theta' + str(n4))
    plt.grid(); plt.show()
    #fig.savefig('dofPlot.png')


# --------------------------------------------------------------------#

# Draw only one graph with trajectory, RM nodes and gradient arrows
def gradArrowsPlot(cl, q_list, grad_list, q_rand_list):
    
    # Trajectory, obstacle, RM nodes stuff :
    t_vec = np.arange(0., cl.problem.pathLength(0), dt)
    Tvec = t_vec [::]
    circle_obst=plt.Circle((0,0),.5,color='r')
    plt.gcf().gca().add_artist(circle_obst)
    i = 0
    for n in cl.problem.nodes() :
        plt.plot(n[1], n[0], 'ro')
        if i>1: # avoid 2 first nodes (init and goal)
            plt.text(n[1]+.01, n[0], r'q_new %i' %(i), fontsize=12)
        i+=1

    for t in Tvec:
        plt.plot([cl.problem.configAtParam(0, t)[1], \
                     cl.problem.configAtParam(0, t+dt)[1]], \
                     [cl.problem.configAtParam(0, t)[0], \
                     cl.problem.configAtParam(0, t+dt)[0]], 'b', \
                     label="trajectory" if t==0. else "")

    plt.axis([-3, 3, -3, 3])
    plt.xlabel('y'); plt.ylabel('x')
    plt.title('Trajectory and gradients'); plt.text(0, 0, r'obstacle')
    plt.grid(); plt.legend(); plt.gca().invert_xaxis()
    plt.plot(cl.problem.nodes()[0][1], cl.problem.nodes()[0][0], 'go')
    plt.plot(cl.problem.nodes()[1][1], cl.problem.nodes()[1][0], 'go')
    plt.text(cl.problem.nodes()[0][1]+.2, cl.problem.nodes()[0][0]+.15, r'q_init', fontsize=12)
    plt.text(cl.problem.nodes()[1][1]+.2, cl.problem.nodes()[1][0]+.15, r'q_end', fontsize=12)
      
    # Gradients Arrows stuff :
    # formats : q_list[x, y, theta][t] and grad_list[x, y, theta][t]
    #i=0; thres1=0.8; thres2=1.; thres3=1.3 # v1 for potential-method
    i=0; thres1=0.9; thres2=1.1; thres3=1.4 # v2 for superposed-potential-method
    #coef1=0.01; coef2=0.07; coef3=0.8; coef4=1.5 # v1 for potential-method
    coef1=0.3; coef2=0.9; coef3=1.9; coef4=2.4 # v2 for superposed-potential-method
    boolPlot1=True; boolPlot2=True; boolPlot3=True; boolPlot4=True
    for q in zip(*q_list) :
        #print q
        x = q[0]; y = q[1]; dist = math.sqrt(x**2+y**2)
        
        """if (dist <= thres1):
            dx = grad_list[0][i]*coef1; dy = grad_list[1][i]*coef1
            arrow1=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='y', ec='y', label="arrow1" if boolPlot1 else "")
            boolPlot1=False"""
        
        #elif (dist > thres1 and dist <= thres2):
        if (dist < thres2):
            dx = grad_list[0][i]*coef2; dy = grad_list[1][i]*coef2
            arrow2=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='y', ec='y', label="arrow2" if boolPlot2 else "")
            boolPlot2=False
        
        elif (dist > thres2 and dist <= thres3):
            dx = grad_list[0][i]*coef3; dy = grad_list[1][i]*coef3
            arrow3=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='0.7', ec='0.7', label="arrow3" if boolPlot3 else "")
            boolPlot3=False
        
        #elif (dist > thres3):
        elif (dist > thres2):
            dx = grad_list[0][i]*coef4; dy = grad_list[1][i]*coef4       
            arrow4=plt.arrow(y, x, dy, dx, lw=0.8,head_width=.015, head_length=0.025, fc='0.5', ec='0.5', label="arrow4" if boolPlot4 else "")
            boolPlot4=False
        i=i+1
    
    #plt.legend([arrow1, arrow2, arrow3, arrow4], ['coef=0.01, thres=0.8', 'coef 0.07, thres=1', 'coef 0.8, thres=1.3', 'coef 1.5'])
    plt.legend([arrow2, arrow3, arrow4], ['coef=0.9, thres=0.9', 'coef 1.9, thres=1.1', 'coef 2.4'])
    plt.show()
    #fig.savefig('gradientsArrowsPlot.png')

# --------------------------------------------------------------------#

# Draw only one graph with trajectory, RM nodes and gradient arrows
def gradArrowsConcPlot(cl, q_list, grad_list, q_rand_list, nPath):
    Tvec = np.arange(0., cl.problem.pathLength(nPath), dt)
    
    plt.gcf().gca().add_artist(plt.Circle((0.9,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(0.9, 0.4, r'obst1', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.5,0.4),.6,color=(1, 0.4, 0.3))) # orange
    plt.text(1.5, 0.3, r'obst1bis', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.2,-3.5),1.2,color='green'))
    plt.text(1.2, -3.5, r'obst2', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2.6,2.5),1.2,color='green'))
    plt.text(2.6, 2.5, r'obst6', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.5,1.2),.6,color='gray'))
    plt.text(-2.5, 1.2, r'obst4', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1,4),.6,color='gray'))
    plt.text(-1, 4, r'obst5', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-2.8,-2.2),1.2,color='b'))
    plt.text(-2.8, -2.2, r'obst3', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((3.8,-1),.6,color='b'))
    plt.text(3.8, -1, r'obst7', fontsize=11)
    plt.gcf().gca().add_artist(plt.Rectangle((-0.4,-0.9),0.8,1.8,color='r'))
    plt.text(0, 0, r'obst_base', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((1.5,-1.8),0.6,color='b'))
    plt.text(1.5, -1.8, r'obst8', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((2., 1.2),0.4,color=(0, 0.5, 1))) # light_blue
    plt.text(2., 1.2, r'obst9', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.6,1.4),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.6, 1.4, r'obst10', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-1.5,2.3),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-1.5, 2.3, r'obst11', fontsize=11)
    plt.gcf().gca().add_artist(plt.Circle((-0.9,3.),0.5,color=(1, 0.5, 0))) # yellow
    plt.text(-0.9, 3., r'obst12', fontsize=11)
    
    i = 0
    for n in cl.problem.nodes() :
        if i>1: # avoid 2 first nodes (init and goal)
            plt.plot(n[0], n[1], 'ro')
            plt.text(n[0]+.02, n[1], r'q_new %i' %(i), fontsize=11) # rand or new
        i=i+1
    
    for t in Tvec:
        plt.plot([cl.problem.configAtParam(nPath, t)[0], \
                     cl.problem.configAtParam(nPath, t+dt)[0]], \
                     [cl.problem.configAtParam(nPath, t)[1], \
                     cl.problem.configAtParam(nPath, t+dt)[1]], 'k')
    
    plt.axis([-5, 5, -5, 5])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory'); plt.grid()
    plt.plot(cl.problem.nodes()[0][0], cl.problem.nodes()[0][1], 'go')
    plt.plot(cl.problem.nodes()[1][0], cl.problem.nodes()[1][1], 'go')
    plt.text(cl.problem.nodes()[0][0]+.2, cl.problem.nodes()[0][1]+.1, r'q_init', fontsize=12)
    plt.text(cl.problem.nodes()[1][0]+.2, cl.problem.nodes()[1][1]+.1, r'q_end', fontsize=12)
    
    # Gradients Arrows stuff :
    # formats : q_list[x, y, theta][t] and grad_list[x, y, theta][t]
    i=0; boolPlot1=True; coef1=3
    for q in zip(*q_list) :
        #print q
        x = q[0]; y = q[1];
        dx = grad_list[0][i]*coef1; dy = grad_list[1][i]*coef1
        arrow1=plt.arrow(x, y, dx, dy, lw=0.8,head_width=.015, head_length=0.025, fc='0.4', ec='0.4', label="fleches" if boolPlot1 else "")
        boolPlot1=False; i=i+1
    plt.legend([arrow1], ['gain de 3'])
    plt.show()
    #fig.savefig('gradientsArrowsPlot.png')

# --------------------------------------------------------------------#

# 3D plot of translations :
def xyzPlot (cl, pathNum):
    Tvec = np.arange(0., cl.problem.pathLength(pathNum), dt)
    ax = plt.figure().gca(projection='3d')
    
    i = 0
    """for n in cl.problem.nodes() :
        ax.scatter(n[0], n[1], zs=n[2], marker='o')
        if i>1: # avoid 2 first nodes (init and goal)
            ax.text(n[0], n[1], n[2], "q_new"+str(i), zdir=None)
        i=i+1
    """
    
    for t in Tvec:
        ax.plot([cl.problem.configAtParam(pathNum, t)[0], \
                     cl.problem.configAtParam(pathNum, t+dt)[0]], \
                     [cl.problem.configAtParam(pathNum, t)[1], \
                     cl.problem.configAtParam(pathNum, t+dt)[1]], \
                     zs = [cl.problem.configAtParam(pathNum, t)[2], \
                     cl.problem.configAtParam(pathNum, t+dt)[2]], color='black')
                     
    if (pathNum == 1): # plot also initial path to compare
        for t in np.arange(0., cl.problem.pathLength(0), dt):
            ax.plot([cl.problem.configAtParam(0, t)[0], \
                         cl.problem.configAtParam(0, t+dt)[0]], \
                         [cl.problem.configAtParam(0, t)[1], \
                         cl.problem.configAtParam(0, t+dt)[1]], \
                         zs = [cl.problem.configAtParam(0, t)[2], \
                         cl.problem.configAtParam(0, t+dt)[2]], color='red')
    
    axisRange = 1; ax.axis([-axisRange, axisRange, -axisRange, axisRange])
    ax.set_xlabel('x'); ax.set_ylabel('y'); ax.set_zlabel('z')
    ax.text(cl.problem.nodes()[0][0], cl.problem.nodes()[0][1], cl.problem.nodes()[0][2], "q_init", zdir=None)
    ax.text(cl.problem.nodes()[1][0], cl.problem.nodes()[1][1], cl.problem.nodes()[1][2], "q_end", zdir=None)
    ax.legend()
    plt.show()

# --------------------------------------------------------------------#

# Plot on z axis gradient values (instead of arrows)
def grad3DPlot(cl, q_list, grad_list, q_rand_list):
    fig = plt.figure()
    ax = plt.figure().gca(projection='3d')
    
    # Trajectory, obstacle, RM nodes stuff :
    t_vec = np.arange(0., cl.problem.pathLength(0), dt)
    Tvec = t_vec [::]
    circle_obst=plt.Circle((0,0),.5,color='r')
    plt.gcf().gca().add_artist(circle_obst)
    i = 0
    for n in cl.problem.nodes() :
        plt.plot(n[1], n[0], 'ro')
        if i>1: # avoid 2 first nodes (init and goal)
            plt.text(n[1]+.01, n[0], r'q_new %i' %(i), fontsize=7)
        i+=1

    for t in Tvec:
        plt.plot([cl.problem.configAtDistance(0, t)[1], \
                     cl.problem.configAtDistance(0, t+dt)[1]], \
                     [cl.problem.configAtDistance(0, t)[0], \
                     cl.problem.configAtDistance(0, t+dt)[0]], 'b', \
                     label="trajectory" if t==0. else "")

    plt.axis([-3, 3, -3, 3])
    plt.xlabel('y'); plt.ylabel('x')
    plt.title('Trajectory and gradients'); plt.text(0, 0, r'obstacle')
    plt.grid(); plt.legend(); plt.gca().invert_xaxis()
    plt.text(cl.problem.nodes()[0][1]+.2, cl.problem.nodes()[0][0]+.15, r'q_init')
    plt.text(cl.problem.nodes()[1][1]+.2, cl.problem.nodes()[1][0]+.15, r'q_end')
      
    # Gradients :
    # formats : q_list[x, y, theta][t] and grad_list[x, y, theta][t]
    #i=0; thres1=0.8; thres2=1.; thres3=1.3 # v1 for potential-method
    i=0; thres1=0.9; thres2=1.1; thres3=1.4 # v2 for superposed-potential-method
    #coef1=0.01; coef2=0.07; coef3=0.8; coef4=1.5 # v1 for potential-method
    coef1=0.03; coef2=0.11; coef3=1.; coef4=1.7 # v2 for superposed-potential-method
    boolPlot1=True; boolPlot2=True; boolPlot3=True; boolPlot4=True
    for q in zip(*q_list) :
        #print q
        x = q[0]; y = q[1]; dist = math.sqrt(x**2+y**2)
        normGrad = math.sqrt(grad_list[0][i]**2 + grad_list[1][i]**2)
        if (normGrad < 2):
            #dx = grad_list[0][i]*coef1; dy = grad_list[1][i]*coef1
            plot_surface(x, y, normGrad)
        i=i+1
    
    """
    # q_rand_list plot#
    i = 1
    for qrand in zip(*q_rand_list) :
        plt.plot(qrand[1], qrand[0], 'go')
        plt.text(qrand[1]+.01, qrand[0], r'q_rand %i' %(i), fontsize=7)
        i+=1
    """
    #ax.plot(x, y, z, label='parametric curve') # 3D
    # ax.legend() # 3D
    plt.show()
    #fig.savefig('gradientsArrowsPlot.png')

