#!/usr/bin/env python
from scipy.optimize import least_squares
import rospy
import tf
import numpy
from numpy import *
from math import sqrt



def rebroadcast():
    rospy.init_node('rebroadcast', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    haveWorld = 0

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        try:
            #get the world transform
            (worldTrans,worldRot) = listener.lookupTransform('/mocha_world', '/hlInMC', rospy.Time(0))
            (offsetTrans,offsetRot) = listener.lookupTransform('/holoMC', '/OCmark', rospy.Time(0))
            haveWorld = 1
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if haveWorld == 1:
            try:
                #this is the collection of one sample of the marker position as reported by the motion capture.
                time = listener.getLatestCommonTime('/mocha_world', '/holoLens')
                (holoTrans,holoRot) = listener.lookupTransform('/mocha_world', '/holoLens', rospy.Time(0))
                br.sendTransform(worldTrans,
                                     worldRot,
                                     rospy.Time.now(),
                                     'hlInMC',
                                     'mocha_world')

                br.sendTransform(holoTrans,
                                      holoRot,
                                      rospy.Time.now(),
                                      'HLOC',
                                      'hlInMC')
                br.sendTransform(offsetTrans,
                                    offsetRot,
                                    rospy.Time.now(),
                                    'MCOC',
                                    'holoMC')
                rate.sleep()
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue




if __name__ == '__main__':
    try:
        rebroadcast()
    except rospy.ROSInterruptException:
        pass
