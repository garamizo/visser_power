#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Propagates Float64 messages to JointState

def sim_robot():
    
    rospy.init_node("simulated_robot")
    pub = rospy.Publisher("joint_states", JointState, queue_size=5)
    
    rospy.Subscriber("joint_1/command", Float64, lambda msg: joint_cb(msg, 0))
    rospy.Subscriber("joint_2/command", Float64, lambda msg: joint_cb(msg, 1))
    rospy.Subscriber("joint_3/command", Float64, lambda msg: joint_cb(msg, 2))
    rospy.Subscriber("joint_4/command", Float64, lambda msg: joint_cb(msg, 3))
    rospy.Subscriber("joint_5/command", Float64, lambda msg: joint_cb(msg, 4))
    rospy.Subscriber("joint_7/command", Float64, lambda msg: joint_cb(msg, 5))
    
    msg = JointState()
    msg.header = Header(stamp=rospy.Time.now())
    msg.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_7"]
    msg.position = [0,]*6
    msg.velocity = [0, 0, 0, 0, 0, 0]
    msg.effort = [0, 0, 0, 0, 0, 0]
    
    def joint_cb(msg_in, id):
        msg.header = Header(stamp=rospy.Time.now())
        msg.position[id] = msg_in.data
        
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
	sim_robot()
	rospy.spin()