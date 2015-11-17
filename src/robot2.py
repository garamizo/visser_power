#!/usr/bin/env python
import rospy
import numpy
import math
import scipy.optimize
from std_msgs.msg import Float64
import kdl_parser_py.urdf
import PyKDL
import threading

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import tf_conversions.posemath


class Robot(object):
    def __init__(self):

        print("Hi!")
        rospy.init_node("robot")
        
        pub1 = rospy.Publisher("joint_1/command", Float64, queue_size=5)
        pub2 = rospy.Publisher("joint_2/command", Float64, queue_size=5)
        pub3 = rospy.Publisher("joint_3/command", Float64, queue_size=5)
        pub4 = rospy.Publisher("joint_4/command", Float64, queue_size=5)
        pub5 = rospy.Publisher("joint_5/command", Float64, queue_size=5)
        pub6 = rospy.Publisher("joint_7/command", Float64, queue_size=5)
        self.pubs = [pub1, pub2, pub3, pub4, pub5, pub6]
        

        # define kinematic chain for WidowX
        self.space_from_matrix = lambda T: numpy.array([T[0,3], T[1,3], T[2,3], math.atan2(T[1,0], T[0,0]), math.asin(-T[2,0])]) # x y z yaw pitch        
        filePath = '/home/hirolab/catkin_ws/src/visual_servoing_powergrid/urdf/widowx2.urdf'
        
        
        (ok, tree) = kdl_parser_py.urdf.treeFromFile(filePath)
        chain = tree.getChain("base", "male")
        self.dk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
        
        self.dt = 0.05
        self.dof = int(chain.getNrOfJoints())
        self.joints = numpy.zeros(self.dof)
        
        
    def matrix_from_joint(self, Q):
        # unwrap PyKDL frame
        frame = PyKDL.Frame()
        jarray = PyKDL.JntArray(len(Q))
        for k in range(0, len(Q)):
            jarray[k] = Q[k]
        self.dk_solver.JntToCart(jarray, frame)
        return tf_conversions.posemath.toMatrix(frame)


    def space_coordinates(self, joints):
        return self.space_from_matrix(self.matrix_from_joint(joints))

    def plan(self, goal_space, speed):
        # return tuples of joint coordinates to be moved at every self.dt
        # first calculate goal_joint
        def error_function(joint): return (self.space_coordinates(joint) - numpy.array(goal_space))
        x, _, ler, message = scipy.optimize.fsolve(error_function, self.joints, full_output=True)
        if not ler == 1:
            # print "No solution found. Target: ", goal_space
            return None, False

        x = (x+math.pi) % (2*math.pi) - math.pi

        # break trajectory to intermediate points
        current_space = self.space_coordinates(self.joints)
        time_required = numpy.max(numpy.abs(goal_space-current_space)) / speed
        N = numpy.int(numpy.ceil(time_required / self.dt))
        mask = numpy.array(range(1, N+1)).reshape([N, 1]).repeat(self.dof, 1) / numpy.float64(N)

        b_joint = numpy.array(self.joints).reshape(1, self.dof).repeat(N, 0)
        a_joint = numpy.array(x-self.joints).reshape(1, self.dof).repeat(N, 0)
        traject_joint = a_joint * mask + b_joint

        # print traject_joint

        # b_space = current_space.reshape(1, self.dof).repeat(N, 0)
        # a_space = numpy.array(goal_space-current_space).reshape(1, self.dof).repeat(N, 0)
        # traject_space = a_space * mask + b_space

        # # solve for each point
        # for k in range(0, N):
        #     def error_func (joint): return(self.space_coordinates(joint) - traject_space[k])
        #     x, _, ler, message = scipy.optimize.fsolve(error_func, traject_joint[k], full_output=True)
        #     if not ler == 1:
        #         print "Error!"
        #     else:
        #         traject_joint[k] = x

        return traject_joint, True

    def move(self, newPlan):

        rate = rospy.Rate(1.0/self.dt)
        for joints in newPlan:
            for k in range(0, len(joints)):
                self.pubs[k].publish(joints[k])
            self.joints = joints
            rate.sleep()

    def recover(self):
        self.joints = numpy.zeros(self.dof)
        self.move([self.joints])


