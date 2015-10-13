#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import math
import numpy

if __name__ == '__main__':
  rospy.init_node('publisher')
  pub = rospy.Publisher('angles', Float64MultiArray, queue_size=5)

  direc = numpy.array([-1, 1, -1, 1, -1])
  offset = numpy.array([100, 90, 90, 70, 90]) * math.pi/180

  rate = rospy.Rate(5)
  while not rospy.is_shutdown():
    time = rospy.get_rostime().to_sec()
    msg = Float64MultiArray()
    msg.data = ( numpy.array([math.sin(2*math.pi*0.5*time), 0, 0, 0, 0])*direc + offset ) * 180/math.pi
    pub.publish( msg )
    rate.sleep()
