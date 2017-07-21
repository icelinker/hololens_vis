#!/usr/bin/env python
from scipy.optimize import least_squares
import rospy
import rospkg
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
import tf
import numpy as np
import math
from numpy import linalg as LA
from numpy import *
from tf import transformations as TR
targetLength =200

buffersFilled = 0
preCalibration = True
roughCalibrate = True
i = 0
listFull = False

holoTransformList = np.zeros((targetLength,4,4))
inverseholoTransformList = np.zeros((targetLength,4,4))
markerTransformList = np.zeros((targetLength,4,4))
worldTransformStack = np.zeros((targetLength,4,4))
offsetTransformStack = np.zeros((targetLength,4,4))
markerTranslations = np.zeros( (targetLength,3) )
holoTranslations = np.zeros( (targetLength,3) )
roughCalibrate = True
worldTransform = np.zeros((4,4))
offsetTransform = np.zeros((4,4))

askedMarkers = np.zeros( (8,3) )
measuredMarkers = np.zeros( (8,3) )

speechPub = rospy.Publisher('speech', String, queue_size=10)
wTPosOut = np.zeros(3)
wTQuatOut= np.zeros(4)

listener = 0
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

    ##if the diagonal is close to an Identity then we know that the rotations is good.
    #myIdentity = np.identity(4)
    #stackI  = tile(myIdentity,(arrSize,1)).reshape((arrSize,4,4))
    #justDiagonals =  stackI * difference

    ##extract just the distance cost
    #difference = np.absolute(difference)
    #edge = [[0,0,0,1],[0,0,0,1],[0,0,0,1],[0,0,0,0]]
    #stackE  = tile(edge,(arrSize,1)).reshape((arrSize,4,4))
    #justEdge = stackE * difference

    ##calculate Costs, will be 0 if everything lines up.
    #angleCost = 4 - np.sum(justDiagonals, axis=(1,2))
    #edgeCost = np.sum(justEdge,axis=(1,2))

    #return edgeCost + angleCost
    myIdentity = np.identity(4)
    stackI  = tile(myIdentity,(arrSize,1)).reshape((arrSize,4,4))
    difference = difference - stackI
    difference = np.absolute(difference) #this whole vector should now be minimised to 0

    return difference.flatten()

def costFun6(x):

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


    #calculate world transform to use.
    worldIncrement = TR.compose_matrix(translate = xworldT, angles = TR.euler_from_quaternion(xworldRQ))
    localworldTransform = np.dot(worldIncrement,worldTransform)
    localworldTransformInverse = TR.inverse_matrix(localworldTransform)

    #calculat the offsetTransform to use.
    localoffsetTransform = offsetTransform

    # tile these into a repeating stack.
    InvWorldTransformStack = tile(localworldTransformInverse,(arrSize,1)).reshape((arrSize,4,4))
    offsetTransformStack = tile(localoffsetTransform,(arrSize,1)).reshape((arrSize,4,4))

    #find stack of errors.
    left = np.matmul(markerTransformList,offsetTransformStack)
    right =  np.matmul(inverseholoTransformList,InvWorldTransformStack)
    difference = np.matmul( left,right)

    myIdentity = np.identity(4)
    stackI  = tile(myIdentity,(arrSize,1)).reshape((arrSize,4,4))
    difference = difference - stackI
    difference = np.absolute(difference) #this whole vector should now be minimised to 0

    return difference.flatten()

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

def updateStoredTransforms6(update):
    global holoTransformList
    global markerTransformList
    global worldTransform
    global offsetTransform

    arrSize = holoTransformList.shape[0]

    #parse input into meaningful values
    xworldT =update[0:3]
    xworldR = update[3:6]
    xworldRQ = TR.quaternion_about_axis(np.sqrt(xworldR.dot(xworldR)),xworldR)

    #update world transform
    worldIncrement = TR.compose_matrix(translate = xworldT, angles = TR.euler_from_quaternion(xworldRQ))
    worldTransform = np.dot(worldIncrement,worldTransform)


def reCalCB(data):
    global preCalibration
    global i
    global buffersFilled
    global roughCalibrate
    global listFull
    global speechPub
    print 'calibrate'
    i = 0
    buffersFilled = 0
    preCalibration = True
    roughCalibrate = True
    listFull = False
    speechPub.publish('started Calibration')

def gripCB(data):
    print 'Grip!'
    transIn = np.zeros(3)
    quatIn = np.zeros(4)
    transIn[0] = data.position.x
    transIn[1] = -data.position.z
    transIn[2] = data.position.y
    quatIn[0] = data.rotation.x
    quatIn[1] = -data.rotation.z
    quatIn[2] = data.rotation.y
    quatIn[3] = -data.rotation.w
    print TR.euler_from_quaternion(quatIn)
    print transIn
    mat = TR.compose_matrix(translate = transIn +trans, angles = anglesWorld);
    worldTransform = mat

clickNum = 0
def clickedCB(data):
    global askedMarkers
    global listener
    global worldTransform
    global measuredMarkers
    global clickNum
    print 'clicked'
    try:
        (tipTrans,tipRot) = listener.lookupTransform('/mocha_world', '/tip', rospy.Time(0))
        measuredMarkers[clickNum,:] = tipTrans

        clickNum = clickNum + 1
        if clickNum == 8:
            clickNum = 0
            print 'finished clicking'
            print 'AskedMarkers:'
            print askedMarkers
            print 'measured markers:'
            print measuredMarkers
            (worldRotupdate,worldTransUpdate) = rigid_transform_3D(measuredMarkers,askedMarkers)
            worldRot4 = np.identity(4)
            for i in range(3):
                worldRot4[(0,1,2), i] = worldRotupdate[:, i]
            worldTransformUpdate = TR.compose_matrix(translate=worldTransUpdate ,angles = TR.euler_from_matrix(worldRot4))
            worldTransformationUpdateInv = TR.inverse_matrix(worldTransformUpdate)
            worldTransform = np.dot(worldTransformationUpdateInv,worldTransform)
    except():
        nothing =0

def poseArrCB(data):
    global askedMarkers
    for i in range(8):
        coord = np.zeros(3)
        coord[0] = data.poses[i].position.x
        coord[1] = data.poses[i].position.z
        coord[2] = data.poses[i].position.y #switched spaces to ros space
        askedMarkers[i,:] = coord


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
    global buffersFilled
    global roughCalibrate
    global preCalibration
    global i
    global listFull
    global speechPub
    global wTQuatOut
    global wTPosOut
    global listener

    #ros related variables
    rospy.init_node('optimiseHolo', anonymous=True)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(100)
    rospy.Subscriber("reCalibrate", String, reCalCB)
    rospy.Subscriber("cameraPosArr", PoseArray, poseArrCB)
    rospy.Subscriber("grip", Pose, gripCB)
    rospy.Subscriber("pointClicked", Int32, clickedCB)
    speechPub = rospy.Publisher('speech', String, queue_size=10)


    #State holding variables
    preCalibration = True
    roughCalibrate = True
    listFull = False
    holoSaved = np.zeros((3))
    i = 0;
    lastUp = 0
    sinceLast = 0

    #check if we have a saved callibration
    rospack = rospkg.RosPack()
    packPath = rospack.get_path('hololens_vis')
    try:
        offsetTransform = np.load(packPath + '/calibrate/saved.npy')
        print 'using saved offsetTransform'
        wTQuatOut = np.array([0,0,0,1])
        wTPosOut = np.array([0,0,0])
        worldTransform = TR.compose_matrix(translate = wTPosOut, angles = TR.euler_from_quaternion(wTQuatOut))
        preCalibration = False
        roughCalibrate = False
        updateOnFirst = True
    except(IOError):
        preCalibration = True
        updateOnFirst = False
        roughCalibrate = True


    while not rospy.is_shutdown():
        try:
            #if preCalibration:
                #print 'precal'
            #else:
                #print 'notCal'
            #get holoLens reported position
            timeMC = listener.getLatestCommonTime('/holoLens', '/holoMC')
            (holoTrans,holoRot) = listener.lookupTransform('/mocha_world', '/holoLens',timeMC - rospy.Duration.from_sec(0.01))

            # only when we are doing the precal and rough cal do we need the transforms to be adequately separated.
            if buffersFilled <1 and preCalibration:
                if LA.norm(np.asarray(holoSaved)-np.asarray(holoTrans)) < 0.01:
                    raise NameError('Too close')
            else:
                if LA.norm(np.asarray(holoSaved)-np.asarray(holoTrans)) < 0.01:
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
            mostRecentholoInv = inverseholoTransformList[i,:,:]
            mostRecentMarker = markerTransformList[i,:,:]

            #Track where we are in the circular buffers, indicate when the lists are first full.
            i = i +1
            sinceLast = sinceLast +1
            i = i%targetLength
            if i == 0:
                listFull = True
                buffersFilled = buffersFilled + 1
                #print 'listfull'

            if updateOnFirst:
                print 'setting agressivly'
                worldTransform = np.dot(mostRecentMarker,offsetTransform)
                worldTransform = np.dot(worldTransform,mostRecentholoInv)
                wTQuatOut =  TR.quaternion_from_matrix(worldTransform)
                wTPosOut =TR.translation_from_matrix(worldTransform)

                worldTransform = TR.compose_matrix(translate = wTPosOut, angles = TR.euler_from_quaternion(wTQuatOut))
                updateOnFirst = False


        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception, NameError):
            continue

        if preCalibration:
            print i
            # are we ready to perform the preCal?
            if listFull:
                print 'svd calibration'
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
            #full 12 degrees of freedom as marker offset is still not known
            if buffersFilled == 1 and roughCalibrate:
                print '12DoF calibration'
                roughCalibrate = False
                # initial value for solver. 12 values representing the worldTrans, worldRot, markerOffset, markerRot
                Xinit = np.array([0,0,0,0,0,0,0,0,0,0,0,0])

                # The main solving function. # should return near 0 in functional steady state.
                result  = least_squares(costFun,Xinit, method = 'lm', jac = '3-point',verbose = 0)
                print result.x
                #update our guesses with the Delta that we optimisted.
                updateStoredTransforms(result.x)
                print 'offsetTransform'
                print offsetTransform
                speechPub.publish('Finished Calibration')
                np.save(packPath + '/calibrate/newest.npy',offsetTransform)
                wTQuatOut =  TR.quaternion_from_matrix(worldTransform)
                wTPosOut =TR.translation_from_matrix(worldTransform)
            #Only 6 degrees of freedom as only the world offset might drift.
            else:
                # transmit the one shot calculated world transform assuming offset Transform remains correct.
                if lastUp != i and sinceLast > 1:
                    sinceLast = 0
                    lastUp = i
                    #Xinit = np.array([0,0,0,0,0,0])

                    # The main solving function. # should return near 0 in functional steady state.
                    #result  = least_squares(costFun6,Xinit, method = 'lm', jac = '3-point',verbose = 0)
                    #print result.x
                    #update our guesses with the Delta that we optimisted.
                    #updateStoredTransforms6(result.x)
                    worldTransform = np.dot(mostRecentMarker,offsetTransform)
                    worldTransform = np.dot(worldTransform,mostRecentholoInv)
                     ##make an IIR filter for the transform.
                    wTQuatIn = TR.quaternion_from_matrix(worldTransform)
                    wTQuatOut = TR.quaternion_slerp(wTQuatOut,wTQuatIn,0.01)
                    wTPosIn = TR.translation_from_matrix(worldTransform)
                    wTPosOut = 0.99*wTPosOut + 0.01* wTPosIn

                    #wTQuatOut = wTQuatOut / LA.norm(wTQuatOut)
                    worldTransform = TR.compose_matrix(translate = wTPosOut, angles = TR.euler_from_quaternion(wTQuatOut))


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
        print 'starting solver'
        holoSolve()
    except rospy.ROSInterruptException:
        pass
