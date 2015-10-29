#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import scipy.optimize
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Header
import tf2_ros
from functools import partial
import threading

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

class Robot(object):
    def __init__(self):

        def transformation_matrix(rot_angle, rot_dir, trans):
            return tfs.concatenate_matrices(
                tfs.rotation_matrix(rot_angle, rot_dir),
                tfs.translation_matrix(trans))

        rospy.init_node("robot")
        self.joint_publisher = rospy.Publisher("joint_states", JointState, queue_size=5)
        self.joint_publisher2 = rospy.Publisher('joint_states2', Float64MultiArray, queue_size=5)
        self.tf_publisher = tf2_ros.TransformBroadcaster()

        # define kinematic chain for Runner Robot
        # links = ['world', 'link1', 'link2', 'link3', 'gripper']
        # joints = ['joint1', 'joint2', 'joint3', 'joint4']
        # offset = [[17.5e-3, 0, 70e-3], [0, 0, 146.5e-3], [186e-3, 0, 0], [75e-3, -20e-3, 0]]
        # axis = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 0, 1]]
        # space_from_matrix = lambda T: numpy.array([T[0,3], T[1,3], T[2,3], math.atan2(T[1,0], T[0,0])])

        # define kinematic chain for Vision Robot
        links = ['map', 'link1', 'link2', 'link3', 'link4', 'gripper']
        jnames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        offset = [[0, 0, 70e-3], [0, 0, 145e-3], [150e-3, 0, 0], [50e-3, 0, 0], [75e-3, 0, 0]]
        axis = [[0, 0, 1], [0, 1, 0], [0, 1, 0], [1, 0, 0], [0, 1, 0]]
        space_from_matrix = lambda T: numpy.array([T[0,3], T[1,3], T[2,3], math.atan2(T[1,0], T[0,0]), math.asin(-T[2,0])])

        (self.links, self.jnames, self.offset, self.axis, self.space_from_matrix) = (links, jnames, offset, axis, space_from_matrix)

        self.transformations = [partial(transformation_matrix, rot_dir=axis[k], trans=offset[k]) for k in range(len(self.jnames))]

        self.dt = 0.05
        self.dof = len(self.transformations)
        self.joints = numpy.zeros(self.dof)

        t = threading.Thread(target=self.state_publisher)
        t.start()

    def space_coordinates(self, joints):
        T = tfs.identity_matrix()
        for idx in range(0, self.dof):
            T = tfs.concatenate_matrices(T, self.transformations[idx](joints[idx]))
        return self.space_from_matrix(T)

    def plan(self, goal_space, speed):
        # return tuples of joint coordinates to be moved at every self.dt
        # first calculate goal_joint
        def error_function(joint): return (self.space_coordinates(joint) - numpy.array(goal_space))
        x, _, ler, message = scipy.optimize.fsolve(error_function, 0*self.joints, full_output=True)
        print x, ler, message
        x = (x+math.pi) % (2*math.pi) - math.pi

        # break trajectory to intermediate points
        time_required = numpy.max(numpy.abs(x-self.joints)) / speed
        N = numpy.int(numpy.ceil(time_required / self.dt))
        mask = numpy.array(range(1, N+1)).reshape([N, 1]).repeat(self.dof, 1) / numpy.float64(N)
        b = numpy.array(self.joints).reshape(1, self.dof).repeat(N, 0)
        a = numpy.array(x-self.joints).reshape(1, self.dof).repeat(N, 0)
        traject = a * mask + b

        return map(numpy.array, traject), ler==1

    def move(self, newPlan):

        rate = rospy.Rate(1.0/self.dt)
        for joints in newPlan:
            self.joints = joints
            rate.sleep()

        print 'Plan completed'

    def recover(self):
        self.joints = numpy.zeros(self.dof)
        self.move([self.joints])

    def state_publisher(self):

        rate = rospy.Rate(1.0/self.dt)
        while not rospy.is_shutdown():
            time = rospy.Time.now()

            # write to JointState
            header = Header(stamp=time)
            msg = JointState(header=header, name=self.jnames, position=self.joints,
                             velocity=numpy.zeros(self.dof), effort=numpy.zeros(self.dof))
            self.joint_publisher.publish(msg)

            msg = Float64MultiArray()
            msg.data = self.joints
            self.joint_publisher2.publish( msg )

            # write to tf
            t = []
            for k in range(0, self.dof):
                t = t + [pack_pose(time, self.links[k+1], self.links[k], matrix=self.transformations[k](self.joints[k]))]
            self.tf_publisher.sendTransform(t)

            rate.sleep()
