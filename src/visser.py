#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import scipy.optimize
from std_msgs.msg import Float64MultiArray


class Robot:
    def __init__(self):
        self.dof = 4
        self.joints = numpy.zeros(self.dof)
        self.dt = 0.1
        self.joint_speed = 2*math.pi/1

        self.joint_direction = numpy.array([-1, -1, -1, 1])
        self.joint_offset = numpy.array([87, 95, 110, 87]) * math.pi/180

        def transformation_matrix(rot_angle, rot_dir, trans):
            return tfs.concatenate_matrices(tfs.rotation_matrix(rot_angle, rot_dir), tfs.translation_matrix(trans))

        def t01(q): return transformation_matrix(q, [0, 0, 1], [17.5e-3, 0, 70e-3])
        def t12(q): return transformation_matrix(q, [0, 1, 0], [0, 0, 146.5e-3])
        def t23(q): return transformation_matrix(q, [0, 1, 0], [186e-3, 0, 0])
        def t34(q): return transformation_matrix(q, [0, 0, 1], [75e-3, -20e-3, 0])
        self.transformations = (t01, t12, t23, t34)

        self.space_from_matrix = lambda T: numpy.array([T[0,3], T[1,3], T[2,3], math.atan2(T[1,0], T[0,0])])

        self.joint_publisher = rospy.Publisher('angles', Float64MultiArray, queue_size=5)

    def space_coordinates(self, joints):
        T = tfs.identity_matrix()
        for idx in range(0, self.dof):
            T = tfs.concatenate_matrices(T, self.transformations[idx](joints[idx]))
        return self.space_from_matrix(T)

    def plan(self, goal_space):
        # return tuples of joint coordinates to be moved at every self.dt
        # first calculate goal_joint
        def error_function(joint): return (self.space_coordinates(joint) - goal_space)
        x, _, success, message = scipy.optimize.fsolve(error_function, self.joints, full_output=True)
        print x, success, message

        # break trajectory to intermediate points
        time_required = numpy.max(numpy.abs(x-self.joints)) / self.joint_speed
        N = numpy.int(numpy.ceil(time_required / self.dt))
        mask = numpy.array(range(1, N+1)).reshape([N, 1]).repeat(self.dof, 1) / numpy.float64(N)
        b = numpy.array(self.joints).reshape(1, self.dof).repeat(N, 0)
        a = numpy.array(x-self.joints).reshape(1, self.dof).repeat(N, 0)
        traject = a * mask + b

        return tuple(map(numpy.array, traject))

    def move(self, plan):
        rate = rospy.Rate(1.0/self.dt)
        msg = Float64MultiArray()
        for k in range(0, len(plan)):
            msg.data = (plan[k]*self.joint_direction + self.joint_offset) * 180.0/math.pi
            self.joint_publisher.publish( msg )
            rate.sleep()



if __name__ == '__main__':
    rospy.init_node("visser")

    robot = Robot()
    pose = numpy.array([0.4, 0, 0.2, 0])
    motion_plan = robot.plan(pose)
    print motion_plan
    robot.move(motion_plan)

    rospy.spin()