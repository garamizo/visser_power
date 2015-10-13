#!/usr/bin/env python
import rospy
import tf2_ros as tf
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
    T12 = homogeneous_matrix([0, 0, 70e-3], joint_angles[1], [0, 1, 0])
    T23 = homogeneous_matrix([0, 0, 150e-3], joint_angles[2], [0, 1, 0])
    T34 = homogeneous_matrix([150e-3, 0, 0], joint_angles[3], [1, 0, 0])
    T45 = homogeneous_matrix([50e-3, 0, 0], joint_angles[4], [0, 1, 0])
    T5e = homogeneous_matrix([75e-3, 0, 0], 0, [0, 0, 1])
    Tec = homogeneous_matrix([-30e-3, 0, 115e-3], 2.0944, [-1, 1, -1])
    Tem = homogeneous_matrix([20e-3, 0, 0], 0, [1, 0, 0])
    return T01, T12, T23, T34, T45, T5e, Tec, Tem


# space coordinates of male connector [x, y, z, pitch, yaw]
def space_from_joints(joint_angles):
    T01, T12, T23, T34, T45, T5e, Tec, Tem = direct_kinematics(joint_angles)
    T = tfs.concatenate_matrices(T01, T12, T23, T34, T45, T5e, Tem)
    rz, ry, rx = tfs.euler_from_matrix(T, 'sxyz')
    trans = tfs.translation_from_matrix(T)
    S = np.concatenate((trans, [rz, ry]), axis=1)
    return S

# space coordinates of male connector [x, y, z, pitch, yaw]
def space_from_joints_small(joint_angles):
    T01, T12, T23, T34, T45, T5e, Tec, Tem = direct_kinematics(joint_angles)
    T = tfs.concatenate_matrices(T01, T12, T23, T34, T45, T5e, Tem)
    rx, ry, rz = tfs.euler_from_matrix(T, 'sxyz')
    trans = tfs.translation_from_matrix(T)
    S = np.concatenate((trans, [rz, ry]), axis=1)
    return S


def space_from_StampedTransform(st):
    T = st.transform
    quat = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]
    angles = tfs.euler_from_quaternion(quat, 'sxyz')
    pos = [T.translation.x, T.translation.y, T.translation.z]
    return np.array([pos[0], pos[1], pos[2], angles[2], angles[1]])


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


def core():
    rospy.init_node('core')

    tf_buff = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buff)

    # Publish joint transformations
    bc = tf2_ros.TransformBroadcaster()
    
    # Publish static transformations
    pub_tf_static = rospy.Publisher("/tf_static", TFMessage, queue_size=5)
    t1 = pack_transrot([-30e-3, 0, 115e-3], [-1, 1, -1, 1], rospy.Time.now(), "usb_cam", "gripper")
    t2 = pack_transrot([20e-3, 0, 0], [0, 0, 0, 1], rospy.Time.now(), "male", "gripper")
    t3 = pack_transrot([100e-3, 0, 50e-3], [-1, 0, 1, 0], rospy.Time.now(), "female", "ar_marker_6")
    t4 = pack_transrot([75e-3, 0, 0], [0, 0, 0, 1], rospy.Time.now(), "gripper", "link5")
    static_tfs = TFMessage([t1, t2, t3, t4])

    # Angles to control Arduino
    pub_angles = rospy.Publisher('angles', Float64MultiArray, queue_size=5)

    joint_angles = np.array([0, 0, 0, 0, 0])
    state = 'searching'

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        # Publish static transforms
        pub_tf_static.publish(static_tfs)

        try:
            sf = tf_buff.lookup_transform("world", "female", rospy.Time.now()-rospy.Duration(0.1), rospy.Duration(0.1))
            Sf = space_from_StampedTransform( sf )

            if state == 'searching':
              state = 'closing'

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            joint_goal = np.array([0, 0, 0, 0, 0])
            state = 'searching'
            print 'Unrecognized tag'

        if state == 'searching':
            time = rospy.Time.now().to_sec()
            St = np.array([0, 0, 300e-3, 0.3*math.sin(2*math.pi*0.3*time), 0.2])
        elif state == 'closing':
            St = Sf + np.array([-50e-3, 0, 0, 0, 0])
            print "hi"
        elif state == 'latching':
            St = Sf
        elif state == 'end':
            St = np.array([100e-3, 0, 300e-3, 0, 0])

        # solution = opt.minimize(cost, joint_angles, args=(St,), 
        #   bounds=[(-math.pi/4, math.pi/4)]*5, method='L-BFGS-B')
        # x, message, success = solution.x, solution.message, solution.success
        x, _, success, message = opt.fsolve(error, joint_angles, args=(St,), full_output=True)
        if success:
            # joint_goal = np.arcsin(np.sin(np.array(x)))
            joint_goal = np.array(x)

            
            residual = np.linalg.norm(error(joint_angles,St), ord=2)
            print "Final evaluation: ", residual
            if residual < 5e-3:
                if state == 'closing':
                  state = 'latching'
                elif state == 'latching':
                  state = 'end'

        else:
          joint_goal = np.array([0, 0, 0, 0, 0])
          print 'Divergence:', message

        out_factor = 0.1
        joint_angles = out_factor*joint_goal + (1-out_factor)*joint_angles

        # Finally, update pose
        T01, T12, T23, T34, T45, T5e, Tec, Tem = direct_kinematics(joint_angles)
        t1 = pack_matrix(T01, rospy.Time.now(), "link1", "world")
        t2 = pack_matrix(T12, rospy.Time.now(), "link2", "link1")
        t3 = pack_matrix(T23, rospy.Time.now(), "link3", "link2")
        t4 = pack_matrix(T34, rospy.Time.now(), "link4", "link3")
        t5 = pack_matrix(T45, rospy.Time.now(), "link5", "link4")
        bc.sendTransform([t1, t2, t3, t4, t5])     

        # send new joint angles
        msg = Float64MultiArray()
        direc = np.array([-1, 1, -1, 1, -1])
        offset = np.array([100, 90, 90, 70, 90]) * math.pi/180
        msg.data = ( joint_angles*direc + offset ) * 180/math.pi
        pub_angles.publish( msg )

        print "State: ", state

        rate.sleep()

    rospy.spin()

    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     # update (tag-camera) transform
    #     try:
    #         Tft = homogeneous_matrix([100e-3, 0, 50e-3], 2.094395, [-1,1,1])
    #         sendTransform(Tft, 'female', 'tag', rospy.Time.now())

    #         Tst = tfBuffer.lookup_transform('usb_cam', 'ar_marker_6', rospy.Time(0))
    #         T = Tst.transform
    #         print T
    #         matrix = tfs.concatenate_matrices( 
    #           tfs.translation_matrix([T.translation.x, T.translation.y, T.translation.z]), 
    #           tfs.quaternion_matrix([T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]))
    #         sendTransform(matrix, 'tag', 'camera', rospy.Time.now())
    #         # print trans

    #         T01, T12, T23, T34, T45, T5e, Tec, Tem = direct_kinematics(joint_angles)
    #         sendTransform(T01, 'link1', 'world', rospy.Time.now())
    #         sendTransform(T12, 'link2', 'link1', rospy.Time.now())
    #         sendTransform(T23, 'link3', 'link2', rospy.Time.now())
    #         sendTransform(T34, 'link4', 'link3', rospy.Time.now())
    #         sendTransform(T45, 'link5', 'link4', rospy.Time.now())
    #         sendTransform(T5e, 'gripper', 'link5', rospy.Time.now())
    #         sendTransform(Tec, 'camera', 'gripper', rospy.Time.now())
    #         sendTransform(Tem, 'male', 'gripper', rospy.Time.now())

    #         # rx, ry, rz = 1, -math.pi/4, math.pi/4
    #         # trans = [150e-3, 0, 150e-3]
    #         # T = tfs.concatenate_matrices(
    #         #   tfs.euler_matrix(rx, ry, rz, 'sxyz'), tfs.translation_matrix(trans))
    #         # S = np.concatenate((trans, [rz, ry]), axis=1)

    #         T = tfBuffer.lookup_transform('world', 'female', rospy.Time(0)).transform
    #         quat = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]
    #         angles = tfs.euler_from_quaternion(quat, 'sxyz')
    #         pos = [T.translation.x, T.translation.y, T.translation.z]
    #         S = np.array([pos[0], pos[1], pos[2], angles[1], angles[2]])

    #         in_factor = 0.8
    #         St = (1-in_factor)*error(joint_angles,np.array([0,0,0,0,0])) + in_factor*S
    #         print "Goal in S: "
    #         print S
    #         solution = opt.minimize(cost, joint_angles, args=(St,), 
    #           bounds=[(-math.pi/4, math.pi/4)]*5, method='L-BFGS-B')
    #         x, message, success = solution.x, solution.message, solution.success
    #         # x, _, success, message = opt.fsolve(error, joint_angles, args=(St,), full_output=True)
    #         print message
    #         if success:
    #             Ssol = space_from_joints(x)
    #             print "Solution in joint space:"
    #             print x
    #             print "Solution in S: "
    #             print Ssol
    #             out_factor = 0.7
    #             joint_angles = (1-out_factor)*np.array(joint_angles) + out_factor*np.array(x)
    #             msg = Float64MultiArray()
    #             direc = np.array([-1, 1, -1, 1, -1])
    #             offset = np.array([100, 90, 90, 70, 90]) * math.pi/180
    #             msg.data = ( joint_angles*direc + offset ) * 180/math.pi
    #             pub.publish( msg )



    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         continue

    #     rate.sleep()


if __name__ == '__main__':
    core()
