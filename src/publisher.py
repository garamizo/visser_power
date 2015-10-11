#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import math

if __name__ == '__main__':
  rospy.init_node('publisher')
  pub = rospy.Publisher('angles', Float64MultiArray, queue_size=5)

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    time = rospy.get_rostime().to_sec()
    msg = Float64MultiArray()
    msg.data = [math.sin(2*math.pi*0.5*time), 0, 0, 0, 0]
    pub.publish( msg )
    rate.sleep()
