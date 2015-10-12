#!/usr/bin/env python
import rospy
import tf2_ros as tf
from tf import transformations as tfs
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import scipy.optimize as opt
from geometry_msgs.msg import TransformStamped


joint_angles = np.array([0, 0, 0, 0, 0])


# perform DH transformation
def dh_matrix(offset, theta, length, twist):

    Tz = tfs.translation_matrix([0, 0, offset])
    Rz = tfs.rotation_matrix(theta, [0, 0, 1])
    Tx = tfs.translation_matrix([length, 0, 0])
    Rx = tfs.rotation_matrix(twist, [1, 0, 0])
    return tfs.concatenate_matrices(Tz, Rz, Tx, Rx)


# perform active translation on fixed and then rotation into rotating frame
def homogeneous_matrix(trans, rot_ang, rot_dir):

    T = tfs.translation_matrix(trans)
    R = tfs.rotation_matrix(rot_ang, rot_dir)
    return tfs.concatenate_matrices(T, R)


def sendTransform(matrix, child, parent, time):
    # unpack pose and broadcast to tf
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = time
    t.child_frame_id = child
    rotation = tfs.quaternion_from_matrix(matrix)
    translation = tfs.translation_from_matrix(matrix)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]

    br.sendTransform(t)


def direct_kinematics(joint_angles):
    T01 = homogeneous_matrix([0, 0, 0], joint_angles[0], [0, 0, 1])
    T12 = homogeneous_matrix([0, 0, 70e-3], joint_angles[1], [0, 1, 0])
    T23 = homogeneous_matrix([0, 0, 150e-3], joint_angles[2], [0, 1, 0])
    T34 = homogeneous_matrix([150e-3, 0, 0], joint_angles[3], [1, 0, 0])
    T45 = homogeneous_matrix([50e-3, 0, 0], joint_angles[4], [0, 1, 0])
    T5e = homogeneous_matrix([75e-3, 0, 0], 0, [0, 0, 1])
    Tec = homogeneous_matrix([-30e-3, 0, 115e-3], 2.0944, [-1, 1, -1])
    Tem = homogeneous_matrix([20e-3, 0, 0], 0, [1, 0, 0])
    return T01, T12, T23, T34, T45, T5e, Tec, Tem


def update_joints(angles):
    global joint_angles
    joint_angles = angles.data
    print joint_angles


def space_from_joints(joint_angles):
    T01, T12, T23, T34, T45, T5e, _, _ = direct_kinematics(joint_angles)
    T = tfs.concatenate_matrices(T01, T12, T23, T34, T45, T5e)
    rx, ry, rz = tfs.euler_from_matrix(T, 'sxyz')
    trans = tfs.translation_from_matrix(T)
    S = np.concatenate((trans, [rz, ry]), axis=1)
    return S

def error(joint_angles, args):
    # Build current pose vector of gripper, [x y z yaw pitch]
    S1 = space_from_joints(joint_angles)

    # Build goal pose vector
    S2 = args

    return S1 - S2


def cost(joint_angles, args):
    reg_factor = 0.01
    return np.linalg.norm(
      np.concatenate((error(joint_angles, args), joint_angles*reg_factor), axis=1), ord=2)


def core():
    global br, joint_angles
    rospy.init_node('core')

    tfBuffer = tf.Buffer()
    listener = tf.TransformListener(tfBuffer)

#  lt = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.Subscriber('angles', Float64MultiArray, update_joints)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # update (tag-camera) transform
        try:
            Tft = homogeneous_matrix([100e-3, 0, 0], 2.094395, [-1,1,1])
            sendTransform(Tft, 'female', 'tag', rospy.Time.now())

            T = tfBuffer.lookup_transform('usb_cam', 'ar_marker_6', rospy.Time(0)).transform
            matrix = tfs.concatenate_matrices( 
              tfs.translation_matrix([T.translation.x, T.translation.y, T.translation.z]), 
              tfs.quaternion_matrix([T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]))
            sendTransform(matrix, 'tag', 'camera', rospy.Time.now())
            # print trans

            T01, T12, T23, T34, T45, T5e, Tec, Tem = direct_kinematics(joint_angles)
            sendTransform(T01, 'link1', 'world', rospy.Time.now())
            sendTransform(T12, 'link2', 'link1', rospy.Time.now())
            sendTransform(T23, 'link3', 'link2', rospy.Time.now())
            sendTransform(T34, 'link4', 'link3', rospy.Time.now())
            sendTransform(T45, 'link5', 'link4', rospy.Time.now())
            sendTransform(T5e, 'gripper', 'link5', rospy.Time.now())
            sendTransform(Tec, 'camera', 'gripper', rospy.Time.now())
            sendTransform(Tem, 'male', 'gripper', rospy.Time.now())

            rx, ry, rz = 1, -math.pi/4, math.pi/4
            trans = [150e-3, 0, 150e-3]
            T = tfs.concatenate_matrices(
              tfs.euler_matrix(rx, ry, rz, 'sxyz'), tfs.translation_matrix(trans))
            S = np.concatenate((trans, [rz, ry]), axis=1)
            in_factor = 1
            St = (1-in_factor)*error(joint_angles,np.array([0,0,0,0,0])) + in_factor*S
            print "Goal in S: "
            print S
            solution = opt.minimize(cost, joint_angles, args=(St,))
            x, message, success = solution.x, solution.message, solution.success
            # x, _, success, message = opt.fsolve(error, joint_angles, args=(St,), full_output=True)
            print message
            if success:
                Ssol = space_from_joints(x)
                print "Solution in joint space:"
                print x
                print "Solution in S: "
                print Ssol
                out_factor = 0.8
                joint_angles = (1-out_factor)*np.array(joint_angles) + out_factor*np.array(x)



        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Yep'
            continue

        rate.sleep()


if __name__ == '__main__':
    core()
