#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import scipy.optimize
from std_msgs.msg import Float64MultiArray
from threading import Timer


class Robot(object):
    def __init__(self, interface_function=lambda x: False):

        def transformation_matrix(rot_angle, rot_dir, trans):
            return tfs.concatenate_matrices(
                tfs.rotation_matrix(rot_angle, rot_dir),
                tfs.translation_matrix(trans))

        def t01(q): return transformation_matrix(q, [0, 0, 1], [17.5e-3, 0, 70e-3])
        def t12(q): return transformation_matrix(q, [0, 1, 0], [0, 0, 146.5e-3])
        def t23(q): return transformation_matrix(q, [0, 1, 0], [186e-3, 0, 0])
        def t34(q): return transformation_matrix(q, [0, 0, 1], [75e-3, -20e-3, 0])
        self.transformations = (t01, t12, t23, t34)

        self.space_from_matrix = lambda T: numpy.array([T[0,3], T[1,3], T[2,3], math.atan2(T[1,0], T[0,0])])
        self.real_joints = lambda q: q*numpy.array([-1, -1, -1, 1])*180.0/math.pi + numpy.array([87, 95, 110, 87])
        self.ideal_joints = lambda q: (q - numpy.array([87, 95, 110, 87]))*(math.pi/180.0)/numpy.array([-1, -1, -1, 1])
        self.write = interface_function
        self.dt = 0.1

        self.dof = len(self.transformations)
        self.joints = numpy.zeros(self.dof)
        self.plans = []

    def space_coordinates(self, joints):
        T = tfs.identity_matrix()
        for idx in range(0, self.dof):
            T = tfs.concatenate_matrices(T, self.transformations[idx](joints[idx]))
        return self.space_from_matrix(T)

    def plan(self, goal_space, speed):
        # return tuples of joint coordinates to be moved at every self.dt
        # first calculate goal_joint
        def error_function(joint): return (self.space_coordinates(joint) - numpy.array(goal_space))
        x, _, ler, message = scipy.optimize.fsolve(error_function, self.joints, full_output=True)
        if not ler == 1:
            print 'First failure. Trying again'
            x, _, ler, message = scipy.optimize.fsolve(error_function, numpy.zeros(self.dof), full_output=True)
        print x, ler, message

        # break trajectory to intermediate points
        time_required = numpy.max(numpy.abs(x-self.joints)) / speed
        N = numpy.int(numpy.ceil(time_required / self.dt))
        mask = numpy.array(range(1, N+1)).reshape([N, 1]).repeat(self.dof, 1) / numpy.float64(N)
        b = numpy.array(self.joints).reshape(1, self.dof).repeat(N, 0)
        a = numpy.array(x-self.joints).reshape(1, self.dof).repeat(N, 0)
        traject = a * mask + b

        return map(numpy.array, traject)

    def move(self, newPlan):

        rate = rospy.Rate(1.0/self.dt)
        for joints in newPlan:
            self.joints = joints
            self.write(self.real_joints(self.joints))
            rate.sleep()

        print 'Plan completed'

    def recover(self):
        self.joints = numpy.zeros(self.dof)
        self.write(self.real_joints(self.joints))

