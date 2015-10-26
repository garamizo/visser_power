#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import scipy.optimize
from std_msgs.msg import Float64MultiArray
import robot
from geometry_msgs.msg import TransformStamped
import tf2_ros


def pack_pose(time, child, parent, matrix=None, trans=None, quat=None):

    if matrix is not None and (trans is None and quat is None):
        trans = tfs.translation_from_matrix(matrix)
        quat = tfs.quaternion_from_matrix(matrix)
    elif matrix is None and trans is not None and quat is not None:
        matrix = None  # nothing
    else:
        print 'invalid use'

    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = time
    t.child_frame_id = child
    t.transform.translation.x = trans[0]
    t.transform.translation.y = trans[1]
    t.transform.translation.z = trans[2]

    quat = numpy.array(quat)
    quat = quat / numpy.linalg.norm(quat, ord=2)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]

    return t


if __name__ == '__main__':

    rospy.init_node("simulator")
    bc = tf2_ros.TransformBroadcaster()
    r = robot.Robot()

    def update(msg):
        r.joints = r.ideal_joints(numpy.array(msg.data))
    joint_subscriber = rospy.Subscriber('angles', Float64MultiArray, update)

    rate = rospy.Rate(1 / r.dt)
    while not rospy.is_shutdown():

        t1 = pack_pose(rospy.Time.now(), "link1", "world",
                       matrix=r.transformations[0](r.joints[0]))
        t2 = pack_pose(rospy.Time.now(), "link2", "link1",
                       matrix=r.transformations[1](r.joints[1]))
        t3 = pack_pose(rospy.Time.now(), "link3", "link2",
                       matrix=r.transformations[2](r.joints[2]))
        t4 = pack_pose(rospy.Time.now(), "gripper", "link3",
                       matrix=r.transformations[3](r.joints[3]))

        bc.sendTransform([t1, t2, t3, t4])
        rate.sleep()

    rospy.spin()
