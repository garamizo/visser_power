#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import scipy.optimize
from std_msgs.msg import Float64MultiArray
import robot
from transitions import Machine
from tf2_msgs.msg import TFMessage
import tf2_ros
from geometry_msgs.msg import TransformStamped

class SeeingRobot(robot.Robot):

    states = ['resting', 'searching', 'gazing', 'loading', 'connecting', 'unloading']

    def __init__(self):
        rospy.init_node("visser")
        self.joint_publisher = rospy.Publisher('angles', Float64MultiArray, queue_size=5)

        tf_buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buff)

        # Publish joint transformations
        bc = tf2_ros.TransformBroadcaster()
        
        # Publish static transformations
        pub_tf_static = rospy.Publisher("/tf_static", TFMessage, queue_size=5)
        t1 = pack_transrot([-30e-3, 0, 115e-3], [-1, 1, -1, 1], rospy.Time.now(), "usb_cam", "gripper")
        t2 = pack_transrot([75e-3, 0, 0], [0, 0, 0, 1], rospy.Time.now(), "male", "gripper")
        t3 = pack_transrot([125e-3, -10e-3, 75e-3], [-1, 0, 1, 0], rospy.Time.now(), "female", "ar_marker_6")
        t4 = pack_transrot([75e-3, 0, 0], [0, 0, 0, 1], rospy.Time.now(), "gripper", "link5")
        t5 = pack_transrot([-150e-3, 0,0], [0, 0, 0, 1], rospy.Time.now(), "approx", "female")
        t6 = pack_transrot([100e-3, 0, 300e-3], [0, 0, 0, 1], rospy.Time.now(), "rest", "world")
        static_tfs = TFMessage([t1, t2, t3, t4, t5, t6])

        super(SeeingRobot, self).__init__(self.joint_array_writer)
        self.field = 'exist'
        self.machine = Machine(model=self, states=SeeingRobot.states, initial='resting')
        self.machine.add_transition('start', 'resting', 'searching')
        self.machine.add_transition('tag_found', 'searching', 'gazing', before='entry', after='exit')
        # self.machine.add_transition('in_reach', 'gazing', 'connecting')
        # self.machine.add_transition('tag_lost', 'connecting', 'unloading', conditions='is_loaded')
        # self.machine.add_transition('unloaded', 'unloading', 'searching')
        # self.machine.add_transition('connected', 'connecting', 'resting')


    def joint_array_writer(self, joints):
        msg = Float64MultiArray()
        msg.data = joints
        self.joint_publisher.publish(msg)

    def entry(self):
        print 'Enter ', self.state

    def exit(self):
        print 'Exit ', self.state

    def turn_around(self):
        self.move(self.plan([0, 0, 0.3, 0], math.pi/2))
        while True:
            self.move(self.plan([0, 0, 0.3, 0.5], math.pi/5))
            t_tag, success = self.filter_tag_location()
            if success:


    def recover(self):
        super(SeeingRobot, self).recover()
        self.machine.set_state('resting')


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

    r = SeeingRobot()
    # r.move(r.plan([0.1, 0, 0.2, 0]), lambda x: move_func(x, joint_publisher))


    
    pose = [0.4, 0, 0.2, 0]
    r.move(r.plan(pose, math.pi/3))

    count = 0
    search_poses = [[0.2, 0, 0.2, 0], [0.2, 0.2, 0.2, 0], [0.2, -0.2, 0.2, 0]]
    while not rospy.is_shutdown():

        if r.state == 'resting':

        elif r.state == 'searching':
            search_rate.sleep()
            count = (count + 1) % 3
            r.move(r.plan(search_poses[count]))
            tag_pose, success = r.filter_tag_location()
            
            if success: r.tag_found()

        elif r.state == 'gazing':
            

    # rospy.spin()