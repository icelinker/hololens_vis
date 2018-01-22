#!/usr/bin/env python

import rospy
import sys
import tf
from tf import TransformListener, TransformBroadcaster
from tf2_msgs.msg import TFMessage


class CameraTFRepublisher(object):
    def __init__(self,namespace1,namespace2):

        queue_size = None
        buff_size = 65536
        tcp_nodelay = False
        self.tf_broadcaster = TransformBroadcaster()

        self.tf_head_camera_listener = TransformListener()
        self.tf_scene_camera_listener = TransformListener()

        #self.tf_head_camera_listener._listener.tf_sub.unregister()
        #self.tf_head_camera_listener._listener.tf_static_sub.unregister()

        self.tf_head_camera_listener._listener.tf_sub = rospy.Subscriber("/tf_head_camera", TFMessage,
                                                                         self.tf_head_camera_listener._listener.callback,
                                                                         queue_size=queue_size, buff_size=buff_size,
                                                                         tcp_nodelay=tcp_nodelay)
        self.tf_head_camera_listener._listener.tf_static_sub = rospy.Subscriber("/tf_static_head_camera", TFMessage,
                                                                                self.tf_head_camera_listener._listener.static_callback,
                                                                                queue_size=queue_size, buff_size=buff_size,
                                                                                tcp_nodelay=tcp_nodelay)

        self.tf_scene_camera_listener._listener.tf_sub.unregister()
        self.tf_scene_camera_listener._listener.tf_static_sub.unregister()
        self.tf_scene_camera_listener._listener.tf_sub = rospy.Subscriber("/tf_scene_camera", TFMessage,
                                                                          self.tf_scene_camera_listener._listener.callback,
                                                                          queue_size=queue_size, buff_size=buff_size,
                                                                          tcp_nodelay=tcp_nodelay)
        self.tf_scene_camera_listener._listener.tf_static_sub = rospy.Subscriber("/tf_static_scene_camera", TFMessage,
                                                                                 self.tf_scene_camera_listener._listener.static_callback,
                                                                                 queue_size=queue_size, buff_size=buff_size,
                                                                                 tcp_nodelay=tcp_nodelay)

    def publish_inverse_transforms(self):
        try:
            # publish the inverse of the ar_maker_0 and head_camera
            lct = self.tf_head_camera_listener.getLatestCommonTime("/head_camera", "/ar_marker_0")
            if lct is not None:
                (trans, rot) = self.tf_head_camera_listener.lookupTransform("/ar_marker_0", "/head_camera", lct)
                self.tf_broadcaster.sendTransform(translation=trans, rotation=rot, parent="/ar_marker_0", child="/head_camera", time=lct)

            # publish the inverse of the ar_maker_0 and scene_camera
            lct = self.tf_scene_camera_listener.getLatestCommonTime("/scene_camera", "/ar_marker_0")
            if lct is not None:
                (trans, rot) = self.tf_scene_camera_listener.lookupTransform("/ar_marker_0", "/scene_camera", lct)
                self.tf_broadcaster.sendTransform(translation=trans, rotation=rot, parent="/ar_marker_0",
                                                  child="/scene_camera", time=lct)
        except tf.LookupException:
            pass


# Main loop
def loop():
    # Parse input args for topic names
    # progName, cam1,namespace1,cam2,namespace2,marker
    if len(sys.argv) != 6:
        print('not correct number of arguments (cam1,namespace1,cam2,namespace2,marker)')
        sys.exit()
    cam1 = sys.argv[1]
    namespace1 = sys.argv[2]
    cam2 = sys.argv[3]
    namespace2 = sys.argv[4]
    markerName = sys.argv[5]

    # Initialize the node
    rospy.init_node('camera_tf_republisher')

    # Create the camera publisher
    ctfr = CameraTFRepublisher(namespace1,namespace2)
    # maximum rate of camera
    rate = rospy.Rate(90)
    try:
        while not rospy.is_shutdown():
            ctfr.publish_inverse_transforms(cam1,cam2,markerName)
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rospy.spin()


# Main function
if __name__ == '__main__':
    try:
        loop()
    except rospy.ROSInterruptException:
        pass
