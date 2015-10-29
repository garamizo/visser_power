#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math

if __name__ == "__main__":

    rospy.init_node("joint_commander")
    pub = rospy.Publisher("joint_states", JointState, queue_size=5)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        time = rospy.Time.now().to_sec()

        header = Header(stamp=rospy.Time.now())
        name = ["joint1", "joint2", "joint3", "joint4", "joint5", "gripper"]
        position = [math.sin(2 * math.pi * 0.3 * time), 0, 0, 0, 0, 0]
        velocity = [0, 0, 0, 0, 0, 0]
        effort = [0, 0, 0, 0, 0, 0]
        msg = JointState(header=header, name=name, position=position,
                         velocity=velocity, effort=effort)

        pub.publish(msg)
        rate.sleep()
