#/usr/bin/env python
# Author : Mylene Campana

# Plot trajectory and red cylinder obstacle for potential scenario.
def plannarPlot (cl):
    t_vec = np.arange(0., cl.problem.pathLength(0), dt)

    Tvec = t_vec [::]
    plt.subplot(221)
    plt.gcf().gca().add_artist(plt.Circle((0,0),.5,color='r'))
    i = 0
    for n in cl.problem.nodes() :
        plt.plot(n[1], n[0], 'ro')
        if i>1: # avoid 2 first nodes (init and goal)
            plt.text(n[1]+.01, n[0], r'q_new %i' %(i), fontsize=12)
        i=i+1

    for t in Tvec:
        plt.plot([cl.problem.configAtDistance(0, t)[1], \
                     cl.problem.configAtDistance(0, t+dt)[1]], \
                     [cl.problem.configAtDistance(0, t)[0], \
                     cl.problem.configAtDistance(0, t+dt)[0]], 'b')

    plt.axis([-3, 3, -3, 3]) #Ymin Ymax Xmin Xmax
    plt.xlabel('y'); plt.ylabel('x')
    plt.title('trajectory'); plt.grid()
    plt.gca().invert_xaxis()
    plt.text(.5, -.5, r'obstacle')
    plt.plot(cl.problem.nodes()[0][1], cl.problem.nodes()[0][0], 'go')
    plt.plot(cl.problem.nodes()[1][1], cl.problem.nodes()[1][0], 'go')
    plt.text(cl.problem.nodes()[0][1]+.2, cl.problem.nodes()[0][0]+.1, r'q_init', fontsize=12)
    plt.text(cl.problem.nodes()[1][1]+.2, cl.problem.nodes()[1][0]+.1, r'q_end', fontsize=12)

    plt.subplot(222)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[0], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[0], \
                                 cl.problem.configAtDistance(0, t+dt)[0]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-3,3])
    plt.xlabel('t'); plt.ylabel('x'); plt.grid()

    plt.subplot(223)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[1], 'ro')
        plt.plot([t, t+dt], [cl.problem.configAtDistance(0, t)[1], \
                                 cl.problem.configAtDistance(0, t+dt)[1]], 'k-')

    plt.axis([min(Tvec),max(Tvec),-3,3])
    plt.xlabel('t'); plt.ylabel('y'); plt.grid()

    plt.subplot(224)
    for t in Tvec:
        #plt.plot(t, cl.problem.configAtDistance(0, t)[2], 'ro') #theta
        cl.robot.setCurrentConfig(cl.problem.configAtDistance(0,t))
        plt.plot(t, cl.robot.distancesToCollision()[0], '.')

    plt.axis([min(Tvec),max(Tvec),-0.1,3.2])
    plt.xlabel('t')
    plt.ylabel('distance_obst')
    plt.grid(); plt.show()

# --------------------------------------------------------------------#

# Plot the x-y trajectory alone (no subplot or arrows) for multiple runs,
# and save it to a png file. i is the figure number.
def xyTrajectoryPlot (cl, path, nPlot):
    Tvec = np.arange(0., cl.problem.pathLength(0), dt)
    
    plt.gcf().gca().add_artist(plt.Circle((0,0),.5,color='r'))
    i = 0
    for n in cl.problem.nodes() :
        plt.plot(n[1], n[0], 'ro')
        if i>1: # avoid 2 first nodes (init and goal)
            plt.text(n[1]+.01, n[0], r'q_new %i' %(i), fontsize=12)
        i=i+1

    for t in Tvec:
        plt.plot([cl.problem.configAtDistance(0, t)[1], \
                     cl.problem.configAtDistance(0, t+dt)[1]], \
                     [cl.problem.configAtDistance(0, t)[0], \
                     cl.problem.configAtDistance(0, t+dt)[0]], 'b')

    plt.axis([-3, 3, -3, 3]) #Ymin Ymax Xmin Xmax
    plt.xlabel('y'); plt.ylabel('x')
    plt.title('trajectory'); plt.grid()
    plt.gca().invert_xaxis()
    plt.text(.5, -.5, r'obstacle')
    plt.plot(cl.problem.nodes()[0][1], cl.problem.nodes()[0][0], 'go')
    plt.plot(cl.problem.nodes()[1][1], cl.problem.nodes()[1][0], 'go')
    plt.text(cl.problem.nodes()[0][1]+.2, cl.problem.nodes()[0][0]+.1, r'q_init', fontsize=12)
    plt.text(cl.problem.nodes()[1][1]+.2, cl.problem.nodes()[1][0]+.1, r'q_end', fontsize=12)
    plt.show()
    #plt.savefig(path+'plannarPlotTraj'+str(nPlot))

# --------------------------------------------------------------------#


# Draw gradAtt(t) and gradRep(t) for each x y dimension
def gradientPlot (gradAtt, gradRep):
    t_grad = np.arange(0., len(zip(*gradAtt))-1)

    plt.subplot(211)
    for t in t_grad:
        plt.plot([t, t+1], [gradAtt[0][t], gradAtt[0][t+1]],'g', label='gradAtt')
        plt.plot([t, t+1], [gradRep[0][t], gradRep[0][t+1]],'r', label='gradRep')

    plt.axis([min(t_grad),max(t_grad),min(min(gradRep[0]),min(gradAtt[0]))-5, max(max(gradRep[0]),max(gradAtt[0]))+5])
    plt.xlabel('t')
    plt.ylabel('Gradients (x) in first loop')
    plt.grid()

    plt.subplot(212)
    for t in t_grad:
        plt.plot([t, t+1], [gradAtt[1][t], gradAtt[1][t+1]],'g', label='gradAtt')
        plt.plot([t, t+1], [gradRep[1][t], gradRep[1][t+1]],'r', label='gradRep')

    plt.axis([min(t_grad),max(t_grad),min(min(gradRep[1]),min(gradAtt[1]))-5, max(max(gradRep[1]),max(gradAtt[1]))+5])
    plt.xlabel('t')
    plt.ylabel('Gradients (y) in first loop')
    plt.grid()

    plt.show()
    #fig.savefig('gradientPlot.png')

# --------------------------------------------------------------------#

# Plot trajectory and obstacles for potential scenario. nPath is the path number.
def planarObstaclesPlot (cl, nPath, path, nPlot):
    Tvec = np.arange(0., cl.problem.pathLength(nPath), dt)
    
    plt.gcf().gca().add_artist(plt.Circle((1.2,-3.5),1.2,color='green'))
    plt.text(1.2, -3.5, r'obst2', fontsize=10)
    plt.gcf().gca().add_artist(plt.Circle((2.6,2.5),1.2,color='green'))
    plt.text(2.6, 2.5, r'obst6', fontsize=10)
    plt.gcf().gca().add_artist(plt.Circle((-2.5,1.2),.6,color='gray'))
    plt.text(-2.5, 1.2, r'obst4', fontsize=10)
    plt.gcf().gca().add_artist(plt.Circle((-1,4),.6,color='gray'))
    plt.text(-1, 4, r'obst5', fontsize=10)
    plt.gcf().gca().add_artist(plt.Circle((-2.8,-2.2),1.2,color='b'))
    plt.text(-2.8, -2.2, r'obst3', fontsize=10)
    plt.gcf().gca().add_artist(plt.Circle((3.8,-1),.6,color='b'))
    plt.text(3.8, -1, r'obst7', fontsize=10)
    plt.gcf().gca().add_artist(plt.Rectangle((-0.4,-0.9),0.8,1.8,color='r'))
    plt.text(0, 0, r'obst_base', fontsize=10)
    plt.gcf().gca().add_artist(plt.Rectangle((0.6,0.1),1.4,0.6,color='r')) # (1.3,0.4)
    
    i = 0
    for n in cl.problem.nodes() :
        plt.plot(n[0], n[1], 'ro')
        if i>1: # avoid 2 first nodes (init and goal)
            plt.text(n[0]+.02, n[1], r'q_rand %i' %(i), fontsize=7)
        i=i+1

    for t in Tvec:
        plt.plot([cl.problem.configAtDistance(nPath, t)[0], \
                     cl.problem.configAtDistance(nPath, t+dt)[0]], \
                     [cl.problem.configAtDistance(nPath, t)[1], \
                     cl.problem.configAtDistance(nPath, t+dt)[1]], 'k')

    plt.axis([-5, 5, -5, 5])
    plt.xlabel('x'); plt.ylabel('y')
    plt.title('trajectory'); plt.grid()
    plt.text(cl.problem.nodes()[0][0]+.2, cl.problem.nodes()[0][1]+.1, r'q_init')
    plt.text(cl.problem.nodes()[1][0]+.2, cl.problem.nodes()[1][1]+.1, r'q_end')
    plt.show()
    #plt.savefig(path+'plannarPlotTrajObstS'+str(nPlot))

# --------------------------------------------------------------------#

# Plot on z axis gradient values (instead of arrows)
def grad3DPlot(cl, q_list, grad_list, q_rand_list):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
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
