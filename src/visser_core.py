#!/usr/bin/env python
import rospy
import tf
from tf import transformations as tfs 
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import scipy.optimize as opt

joint_angles = [0, 0, 0, 0, 0] 

# perform DH transformation
def dh_matrix( offset, theta, length, twist ):
  Tz = tfs.translation_matrix( [0, 0, offset] )
  Rz = tfs.rotation_matrix( theta, [0, 0, 1] )
  Tx = tfs.translation_matrix( [length, 0, 0] )
  Rx = tfs.rotation_matrix( twist, [1, 0, 0] )
  return tfs.concatenate_matrices( Tz, Rz, Tx, Rx )

# perform active translation on fixed and then rotation into rotating frame
def homogeneous_matrix( trans, rot_ang, rot_dir ):
  T = tfs.translation_matrix( trans )
  R = tfs.rotation_matrix( rot_ang, rot_dir )
  return tfs.concatenate_matrices( T, R )


def sendTransform( matrix, child, parent ):
  q = tfs.quaternion_from_matrix( matrix )
  trans = tfs.translation_from_matrix( matrix )
  br.sendTransform( trans, q, rospy.Time.now(), child, parent )

def direct_kinematics( joint_angles ):
  T01 = homogeneous_matrix( [0, 0, 0], joint_angles[0], [0, 0, 1] )
  T12 = homogeneous_matrix( [0, 0, 50e-3], joint_angles[1], [0, 1, 0] )
  T23 = homogeneous_matrix( [0, 0, 100e-3], joint_angles[2], [0, 1, 0] )
  T34 = homogeneous_matrix( [100e-3, 0, 0], joint_angles[3], [0, 1, 0] )
  T45 = homogeneous_matrix( [0, 0, -30e-3], joint_angles[4], [0, 0, 1] )
  T5e = homogeneous_matrix( [0, 0, -20e-3], 0, [0, 0, 1] )
  return T01, T12, T23, T34, T45, T5e

def update_joints(angles):
  global joint_angles
  joint_angles = angles.data
  print joint_angles

def cost( joint_angles, args ):
  T2 = args
  #T_goal = T_goal.reshape(4,4)
  T01, T12, T23, T34, T45, T5e = direct_kinematics( joint_angles )
  T = tfs.concatenate_matrices( T01, T12, T23, T34, T45, T5e )
  rz, ry, rx = tfs.euler_from_matrix( T, 'szyx' )
  trans = tfs.translation_from_matrix( T )
  rz2, ry2, rx2 = tfs.euler_from_matrix( T2, 'szyx' )
  trans2 = tfs.translation_from_matrix( T2 )
  
  S1 = np.concatenate( (trans,[rz,ry]), axis=1 )
  S2 = np.concatenate( (trans2,[rz2,ry2]), axis=1 )
  return np.linalg.norm( S1-S2, ord=2 )

def core():
  global br, joint_angles
  rospy.init_node('core')
#  lt = tf.TransformListener()
  br = tf.TransformBroadcaster()
#  I = tfs.identity_matrix()
  rospy.Subscriber('angles', Float64MultiArray, update_joints)

  rate = rospy.Rate(10)
  while not rospy.is_shutdown():
    # update (tag-camera) transform 
    try:
      #      br.sendTransform( (-20e-3,0,100e-3), (-0.5,0,0,0.5), rospy.Time.now(), 'camera', 'gripper')
#      (trans,rot) = lt.lookupTransform('usb_cam', 'ar_marker_6', rospy.Time(0))
#      br.sendTransform(trans, rot, rospy.Time.now(), 'tag', 'camera')
      #print trans
      T01, T12, T23, T34, T45, T5e = direct_kinematics( joint_angles )
      sendTransform( T01, 'link1', 'world' )
      sendTransform( T12, 'link2', 'link1' )
      sendTransform( T23, 'link3', 'link2' )
      sendTransform( T34, 'link4', 'link3' )
      sendTransform( T45, 'link5', 'link4' )
      sendTransform( T5e, 'gripper', 'link5' )

      
      T = homogeneous_matrix( [90e-3, 50e-3, 120e-3], math.pi/4, [0,0,1])
      solution = opt.minimize( cost, joint_angles, args=(T,) )
      joint_angles = solution.x
      print solution.x

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      continue
    

    rate.sleep()


if __name__ == '__main__':
  core()
