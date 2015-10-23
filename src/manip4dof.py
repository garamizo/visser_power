#!/usr/bin/env python
import rospy
from tf import transformations as tfs
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import scipy.optimize as opt
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros


# perform active translation on fixed and then rotation into rotating frame
def homogeneous_matrix(trans, rot_ang, rot_dir):

    T = tfs.translation_matrix(trans)
    R = tfs.rotation_matrix(rot_ang, rot_dir)
    return tfs.concatenate_matrices(T, R)


def direct_kinematics(joint_angles):
    T01 = homogeneous_matrix([0, 0, 0], joint_angles[0], [0, 0, 1])
    T12 = homogeneous_matrix([16.5e-3, 0, 70e-3], joint_angles[1], [0, 1, 0])
    T23 = homogeneous_matrix([0, 0, 146e-3], joint_angles[2], [0, 1, 0])
    T34 = homogeneous_matrix([190e-3, 0, 0], joint_angles[3], [0, 0, 1])
    T4e = homogeneous_matrix([75e-3, -25e-3, 0], 0, [0, 0, 1])
    return T01, T12, T23, T34, T4e


# space coordinates of male connector [x, y, z, pitch, yaw]
def space_from_joints(joint_angles):
    T01, T12, T23, T34, T4e = direct_kinematics(joint_angles)
    T = tfs.concatenate_matrices(T01, T12, T23, T34, T4e)
    rz1, ry, rz2 = tfs.euler_from_matrix(T, 'szyz')
    trans = tfs.translation_from_matrix(T)
    S = np.append(trans, rz2)
    return S


def space_from_StampedTransform(st):
    T = st.transform
    quat = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]
    angles = tfs.euler_from_quaternion(quat, 'szyz')
    pos = [T.translation.x, T.translation.y, T.translation.z]
    return np.array([pos[0], pos[1], pos[2], angles[2]])


def error(joint_angles, args):
    return space_from_joints(joint_angles) - args


def cost(joint_angles, args):
    reg_factor = 0.001
    return np.linalg.norm(
      np.concatenate((error(joint_angles, args), joint_angles*reg_factor), axis=1), ord=2)


def pack_transrot(translation, rotation, time, child, parent):
    """
    :param translation: the translation of the transformtion as a tuple (x, y, z)
    :param rotation: the rotation of the transformation as a tuple (x, y, z, w)
    :param time: the time of the transformation, as a rospy.Time()
    :param child: child frame in tf, string
    :param parent: parent frame in tf, string

    Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
    """

    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = time
    t.child_frame_id = child
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]

    rotation = np.array(rotation)
    rotation = rotation / np.linalg.norm(rotation, ord=2)
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    
    return t


def pack_matrix(matrix, time, child, parent):
    trans = tfs.translation_from_matrix(matrix)
    rot = tfs.quaternion_from_matrix(matrix)
    return pack_transrot(trans, rot, time, child, parent)

def S_random():
    x = np.random.rand(1)*250e-3
    y = 0.0
    z = np.random.rand(1)*200e-3 + 100e-3
    rz = 0.0
    return np.array([x[0], y, z[0], rz])


def plan(joint_angles, space_coord):
    # return tuple of joint_angles

    # find final solution
    x, _, success, message = opt.fsolve(error, joint_angles, args=(space_coord,), full_output=True)
    
    if success:
        # break down into several intermediate joint_angles
        profile = 


def core():
    rospy.init_node('core')

    tf_buff = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buff)

    # Publish joint transformations
    bc = tf2_ros.TransformBroadcaster()
    
    # Publish static transformations
    pub_tf_static = rospy.Publisher("/tf_static", TFMessage, queue_size=5)
    static_tfs = TFMessage([])

    # Angles to control Arduino
    pub_angles = rospy.Publisher('angles', Float64MultiArray, queue_size=5)

    S_home = np.array([150e-3, 0, 200e-3, 0])

    joint_angles = np.array([0, -0.3, 0, 0])
    St = space_from_joints(joint_angles)
    state = 'moving'
    print 'moving'
    count = 5
    dest = S_home
    button = True

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # Publish static transforms
        pub_tf_static.publish(static_tfs)

        if state == 'moving':

            S_now = space_from_joints(joint_angles)
            dist = np.linalg.norm(dest - S_now, ord=2)
            vel = 30e-2/1
            dt = 0.5
            factor = 0.1
            Sgoal = dest*factor + S_now*(1-factor)
            print 'Diff', Sgoal, dest

            x, _, success, message = opt.fsolve(error, joint_angles, args=(Sgoal,), full_output=True)
            if success:
                factor = 1
                joint_angles = factor*x + (1-factor)*joint_angles
                # joint_angles = x
                residual = np.linalg.norm(error(joint_angles,dest), ord=2)
                residual2 = np.linalg.norm(error(joint_angles,Sgoal), ord=2)
                print 'Solution ', joint_angles, residual, residual2
                
                if residual < 7e-2:
                    if count == 5: 
                        state = 'home'
                        print 'home'
                    else: 
                        state = 'waiting'
                        print 'waiting' 

                # send new joint angles
                msg = Float64MultiArray()
                direc = np.array([-1, -1, -1, 1])
                offset = np.array([87, 95, 110, 87]) * math.pi/180
                msg.data = ( joint_angles*direc + offset ) * 180/math.pi
                pub_angles.publish( msg )

                # update tf
                T01, T12, T23, T34, T4e = direct_kinematics(joint_angles)
                t1 = pack_matrix(T01, rospy.Time.now(), "link1", "world")
                t2 = pack_matrix(T12, rospy.Time.now(), "link2", "link1")
                t3 = pack_matrix(T23, rospy.Time.now(), "link3", "link2")
                t4 = pack_matrix(T34, rospy.Time.now(), "link4", "link3")
                t5 = pack_matrix(T4e, rospy.Time.now(), "gripper", "link4")
                bc.sendTransform([t1, t2, t3, t4, t5])

            else:
                dest = S_home
                print 'Coudn\'t reach destination: ', message


        elif state == 'waiting':

            if button == True and count == 5:
                state = 'home'
                print 'home'
                dest = S_home

            if button == True and count < 5:
                count += 1
                dest = S_random()
                state = 'moving'
                print 'moving', dest
                rospy.sleep(1)

        elif state == 'home':
            count = 1
            state = 'moving'
            print 'moving'
            dest = S_random()
            rospy.sleep(3)
        else:
            print 'Typo!'

        rate.sleep()

if __name__ == '__main__':
    core()
