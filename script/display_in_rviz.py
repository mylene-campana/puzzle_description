#!/usr/bin/env python

#rostopic pub -1 /tf geometry_msgs/TransformStamped[] -- '[2.0, 0.0, 0.0]' '[0, 0.0, 0.0, 0]'
#/tf: tf2_msgs/TFMessage  geometry_msgs/TransformStamped[]
from math import sqrt, atan2
import numpy as np
import rospy
import time
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
from tf import TransformBroadcaster

class Transform (object):
    def __init__ (self, quat, trans):
        self.quat = quat
        self.trans = trans

    def __mul__ (self, other):
        if not isinstance (other, Transform):
            raise TypeError ("expecting Transform type object")
        trans = self.trans + (self.quat * Quaternion (0, other.trans) *
                              self.quat.conjugate ()).array [1:]
        quat = self.quat * other.quat
        return Transform (quat, trans)
    def __str__ (self):
        return \
            """
Transform
  Quaternion:  %s
  Translation: %s """%(self.quat, self.trans)


I4 = np.matrix ([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
def getRootJointPosition (robot):
    pos = robot.getRootJointPosition ()
    return Transform (Quaternion (pos [3:7]), np.array (pos [0:3]))

def computeRobotPositionAnchor (self, config):
    pos = self.rootJointPosition
    self.transform.transform.rotation = (pos.quat.array [1],
                                          pos.quat.array [2],
                                          pos.quat.array [3],
                                          pos.quat.array [0])
    self.transform.transform.translation = (pos.trans [0],
                                             pos.trans [1],
                                             pos.trans [2])
    self.js.position = []
    for (rank, convert) in self.jointConversion:
        self.js.position.append (convert (config [rank:]))

def computeRobotPositionFreeflyer (self, config):
    ff_rot = config [self.cfgBegin+3:self.cfgBegin+7]
    ff_pos = config [self.cfgBegin+0:self.cfgBegin+3]
    jointMotion = Transform (Quaternion (ff_rot), ff_pos)
    pos = self.rootJointPosition * jointMotion
    self.transform.transform.rotation = (pos.quat.array [1],
                                          pos.quat.array [2],
                                          pos.quat.array [3],
                                          pos.quat.array [0])
    self.transform.transform.translation = (pos.trans [0],
                                             pos.trans [1],
                                             pos.trans [2])
    self.js.position = []
    for (rank, convert) in self.jointConversion:
        self.js.position.append (convert (config [rank:]))

def publishRobots (self):
    if not rospy.is_shutdown ():
        config = self.robotConfig
        now = rospy.Time.now ()
        self.computeRobotPosition (self, config)
        self.transform.header.stamp.secs = now.secs
        self.transform.header.stamp.nsecs = now.nsecs
        self.transform.header.seq = self.js.header.seq
        if self.pubRobots ['robot'] is not None:
            self.js.header.stamp.secs = now.secs
            self.js.header.stamp.nsecs = now.nsecs
            self.js.header.seq += 1
            self.js.velocity = len (self.js.position)*[0.,]
            self.js.effort = len (self.js.position)*[0.,]
            self.broadcaster.sendTransform \
                (self.transform.transform.translation,
                 self.transform.transform.rotation,
                 now, self.tf_root, self.referenceFrame)
            if self.pubRobots ['robot'] is not None:
                self.pubRobots ['robot'].publish (self.js)



