#!/usr/bin/env python
from scipy.optimize import least_squares
import rospy
import tf
import numpy as np
import math
from numpy import linalg as LA
from numpy import *
from tf import transformations as TR
targetLength =100

holoTransformList = np.zeros((targetLength,4,4))
inverseholoTransformList = np.zeros((targetLength,4,4))
markerTransformList = np.zeros((targetLength,4,4))
worldTransformStack = np.zeros((targetLength,4,4))
offsetTransformStack = np.zeros((targetLength,4,4))
markerTranslations = np.zeros( (targetLength,3) )
holoTranslations = np.zeros( (targetLength,3) )

worldTransform = np.zeros((4,4))
offsetTransform = np.zeros((4,4))

# returns a translation and rotation that best aligns constallations A and B
######################## This code was shamelessly borrowed from http://nghiaho.com/uploads/code/rigid_transform_3D.py_
######################## Credit to Nghia Ho
# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = mean(A, axis=0)
    centroid_B = mean(B, axis=0)

    # centre the points
    AA = A - tile(centroid_A, (N, 1))
    BB = B - tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = np.dot(transpose(AA), BB)

    U, S, Vt = linalg.svd(H)

    R = np.dot(Vt.T,U.T)

    # special reflection case
    if linalg.det(R) < 0:
       print "Reflection detected"
       Vt[2,:] *= -1
       R = np.dot(Vt.T , U.T)
    t = np.dot(-R,centroid_A.T) + centroid_B.T

    print t

    return R, t
## removed Printing from code.

########################

# returns a translation and rotation that best aligns Vectors post rotation.
def findOffset():
    global holoTransformList
    global markerTransformList
    global worldTransform
    global offsetTransform
    #transform holoLens to output using world transfrom
    thisHoloMat = holoTransformList[0,:,:]
    holoAnswer = np.dot(worldTransform,thisHoloMat)

    #find inverse of the marker transform
    markerTransform = markerTransformList[0,:,:]
    markerInverse = TR.inverse_matrix(markerTransform)

    #applying the inverse of the marker transform to the optical centre predicted by the transformed hololens tranfrom should iscolate the offset transform
    offsetTransform = np.dot(markerInverse,holoAnswer)


# cost function for optimisation. Calculated discrepancy between the optical frame calculated via the two methods.
# 1) applying the world transform to the hololens measurements 2) applying the offset transform to the marker measurements.
def costFun(x):

    global holoTransformList
    global markerTransformList
    global worldTransform
    global offsetTransform
    global inverseholoTransformList

    arrSize = holoTransformList.shape[0]
    residuals = np.zeros(arrSize)

    #parse input vector into meaningful quantities
    xworldT = x[0:3]
    xworldR = x[3:6]
    xworldRQ = TR.quaternion_about_axis(np.sqrt(xworldR.dot(xworldR)),xworldR)
    xoffsetT = x[6:9]
    xoffsetR = x[9:12]
    xoffsetRQ = TR.quaternion_about_axis(np.sqrt(xoffsetR.dot(xoffsetR)) ,xoffsetR)

    #calculate world transform to use.
    worldIncrement = TR.compose_matrix(translate = xworldT, angles = TR.euler_from_quaternion(xworldRQ))
    localworldTransform = np.dot(worldIncrement,worldTransform)
    localworldTransformInverse = TR.inverse_matrix(localworldTransform)

    #calculat the offsetTransform to use.
    offsetIncrement =  TR.compose_matrix(translate = xoffsetT, angles = TR.euler_from_quaternion(xoffsetRQ))
    localoffsetTransform = np.dot(offsetIncrement,offsetTransform)

    # tile these into a repeating stack.
    InvWorldTransformStack = tile(localworldTransformInverse,(arrSize,1)).reshape((arrSize,4,4))
    offsetTransformStack = tile(localoffsetTransform,(arrSize,1)).reshape((arrSize,4,4))

    #find stack of errors.
    left = np.matmul(markerTransformList,offsetTransformStack)
    right =  np.matmul(inverseholoTransformList,InvWorldTransformStack)
    difference = np.matmul( left,right)

    #if the diagonal is close to an Identity then we know that the rotations is good.
    myIdentity = np.identity(4)
    stackI  = tile(myIdentity,(arrSize,1)).reshape((arrSize,4,4))
    justDiagonals =  stackI * difference

    #extract just the distance cost
    difference = np.absolute(difference)
    edge = [[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,0]]
    stackE  = tile(edge,(arrSize,1)).reshape((arrSize,4,4))
    justEdge = stackE * difference

    #calculate Costs, will be 0 if everything lines up.
    angleCost = 4 - np.sum(justDiagonals, axis=(1,2))
    edgeCost = np.sum(justEdge,axis=(1,2))

    return edgeCost + angleCost


def updateStoredTransforms(update):
    global holoTransformList
    global markerTransformList
    global worldTransform
    global offsetTransform

    arrSize = holoTransformList.shape[0]

    #parse input into meaningful values
    xworldT =update[0:3]
    xworldR = update[3:6]
    xworldRQ = TR.quaternion_about_axis(np.sqrt(xworldR.dot(xworldR)),xworldR)
    xoffsetT = update[6:9]
    xoffsetR = update[9:12]
    xoffsetRQ = TR.quaternion_about_axis(np.sqrt(xoffsetR.dot(xoffsetR)),xoffsetR)

    #update world transform
    worldIncrement = TR.compose_matrix(translate = xworldT, angles = TR.euler_from_quaternion(xworldRQ))
    worldTransform = np.dot(worldIncrement,worldTransform)

    #update offset transform
    offsetIncrement =  TR.compose_matrix(translate = xoffsetT, angles = TR.euler_from_quaternion(xoffsetRQ))
    offsetTransform = np.dot(offsetIncrement,offsetTransform)


def holoSolve():

    global holoTransformList
    global inverseholoTransformList
    global markerTransformList
    global holoTransList
    global holoRotList
    global markerTransList
    global markerRotList
    global worldTransform
    global offsetTransform

    #ros related variables
    rospy.init_node('optimiseHolo', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)

    #State holding variables
    preCalibration = True
    listFull = False
    holoSaved = np.zeros((3))
    i = 0;

    while not rospy.is_shutdown():
        try:

            #get holoLens reported position
            timeMC = listener.getLatestCommonTime('/holoLens', '/holoMC')
            (holoTrans,holoRot) = listener.lookupTransform('/mocha_world', '/holoLens',timeMC)
            if LA.norm(np.asarray(holoSaved)-np.asarray(holoTrans)) < 0.05:
                raise NameError('Too close')
            holoSaved = holoTrans

            #this is the collection of one sample of the marker position as reported by the motion capture.
            (markerTrans,markerRot) = listener.lookupTransform('/mocha_world', '/holoMC',timeMC)

            #Produce stack of matricies for use in the optimisation
            holoTranslations[i,:] = holoTrans
            markerTranslations[i,:] = markerTrans
            holoTransformList[i,:,:] = TR.compose_matrix(translate=holoTrans ,angles = TR.euler_from_quaternion(holoRot))
            inverseholoTransformList[i,:,:] = TR.inverse_matrix(TR.compose_matrix(translate=holoTrans ,angles = TR.euler_from_quaternion(holoRot)))
            markerTransformList[i,:,:] = TR.compose_matrix(translate=markerTrans ,angles = TR.euler_from_quaternion(markerRot))

            #Track where we are in the circular buffers, indicate when the lists are first full.
            i = i +1
            i = i%targetLength
            if i == 0:
                listFull = True


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception, NameError):
            continue

        if preCalibration:
            print i
            # are we ready to perform the preCal?
            if listFull:

                #make an initial guess at world trans and Rot using SVD matching using the position vectors only.
                (worldRot,worldTrans) =  rigid_transform_3D(holoTranslations,markerTranslations)
                worldRot4 = np.identity(4)
                for i in range(3):
                    worldRot4[(0,1,2), i] = worldRot[:, i]
                worldTransform = TR.compose_matrix(translate=worldTrans ,angles = TR.euler_from_matrix(worldRot4))

                #make an initial guess at the marker offset, uses only first value in the buffers.
                findOffset()

                preCalibration = False

        #Here we have calculated the precalibration and will now calculate the error between the transforms using a solver.
        else:

            # initial value for solver. 12 values representing the worldTrans, worldRot, markerOffset, markerRot
            Xinit = np.array([0,0,0,0,0,0,0,0,0,0,0,0])

            # The main solving function. # should return near 0 in functional steady state.
            result  = least_squares(costFun,Xinit, method = 'lm', jac = '3-point',verbose = 0)

            #update our guesses with the Delta that we optimisted.
            updateStoredTransforms(result.x)
            print('updated')

            #send transforms for visualisation.
            br.sendTransform(   TR.translation_from_matrix(worldTransform),
                                TR.quaternion_from_matrix(worldTransform),
                                rospy.Time.now(),
                                'hlInMC',
                                'mocha_world')
            br.sendTransform(   TR.translation_from_matrix(offsetTransform),
                                TR.quaternion_from_matrix(offsetTransform),
                                rospy.Time.now(),
                                'OCmark',
                                'holoMC')
            br.sendTransform(   holoTrans,
                                holoRot,
                                #TR.quaternion_inverse(holoRot2),
                                rospy.Time.now(),
                                'OCholo',
                                'hlInMC')
        try:
            rate.sleep()
        except:
            continue

if __name__ == '__main__':
    try:
        holoSolve()
    except rospy.ROSInterruptException:
        pass
