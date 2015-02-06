#/usr/bin/env python
# Parser script to gather float vector from a (Log) file
import numpy as np

logFile = "/local/mcampana/devel/hpp/install/var/log/hpp/"
endWhile = "INFO:/local/mcampana/devel/hpp/src/hpp-core/src/potential-method.cc:37: finalPath:"

# Parse configurations of given path, until the endWhile line is reached.
def parseConfig (pid, prefix):
    l = len (prefix)
    lend = len(endWhile)
    isNotFinished = True
    with open (logFile + "journal." + str(pid) + ".log") as f:
        configs = []
        for line in f:
            if line [:l] == prefix and isNotFinished :
                suffix = line [l:]
                st = suffix.strip (',\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    config = map (float, sp) # convert into float
                    configs.append (config)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
            if line[:lend] == endWhile :
                isNotFinished = False
    return np.array (zip (*configs)) # transpose and make array

"""
gradAtt :
_,    _,
_,  , _,  , ....
_,    _,
"""

# --------------------------------------------------------------------#

# Depend on how is written the vector in Log file.
# Easier if separated by comas than irregular spaces.
def parseGrad (pid, prefix):
    l = len (prefix)
    lend = len(endWhile)
    isNotFinished = True
    with open (logFile + "journal." + str(pid) + ".log") as f:
        grads = []
        for line in f:
            if line [:l] == prefix and isNotFinished :
                suffix = line [l:]
                st = suffix.strip ('\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    grad = map (float, sp) # convert into float
                    grads.append (grad)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
            if line[:lend] == endWhile :
                isNotFinished = False
    return np.array (zip (*grads)) # transpose and make array


# --------------------------------------------------------------------#

# Parse configurations without end condition. 
# Used to parse guard nodes of visibility-prm algorithm.
def parseNodes (pid, prefix):
    l = len (prefix)
    lend = len(endWhile)
    with open (logFile + "journal." + str(pid) + ".log") as f:
        nodes = []
        for line in f:
            if line [:l] == prefix:
                suffix = line [l:]
                st = suffix.strip (',\n') # remove end characters
                sp = st.split (',') # separate numbers with coma
                try:
                    node = map (float, sp) # convert into float
                    nodes.append (node)
                    
                except:
                    print ("st=%s"%st)
                    print ("sp=%s"%sp)
    #return np.array (zip (*nodes)) # transpose and make array
    return np.array (nodes) # make array

# --------------------------------------------------------------------#

# Parse a pathVector returned in a hpp log file, until the endLine is reached.
# iterNB allows to select the optim iteration which we want to plot.
def parsePathVector (pid, beginLine, endLine, iterNB):
    prefixConfig = 'INFO:/local/mcampana/devel/hpp/src/hpp-core/src/path-optimization/gradient-based.cc:239: '
    l = len(prefixConfig)
    lineNB = 0
    itNB = 1 # current iteration number (of optim algo)
    confVect = [0, 0] # 2D row
    with open (logFile + "journal." + str(pid) + ".log") as f:
        lines=f.readlines()
        configs = []
        for line in lines:
            lineNB = lineNB+1
            if (line.strip('\n') == beginLine) : # begin line found
                if (itNB == iterNB): # wanted iteration
                    #lineNB = lineNB + 1 # Start parsing path until the final line is reached
                    while (1):
                        actualLine = lines[lineNB]
                        actualLine1 = lines[lineNB+1]
                        suffix = actualLine[len (prefixConfig):]
                        suffix1 = actualLine1[len (prefixConfig):]
                        st = suffix.strip ('\n') # remove end characters
                        st1 = suffix1.strip ('\n')
                        #print float(st)
                        confVect[0] = float(suffix)
                        confVect[1] = float(suffix1)
                        configs.append(confVect[:])
                        print confVect
                        # skip 2 useless lines (y parsed and 2 quaternions)
                        lineNB = lineNB + 4 
                        if (lines [lineNB].strip ('\n') == endLine):
                            #print "END OF PARSING"
                            return np.array (configs) # make array and finish
                itNB = itNB+1


