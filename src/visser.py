#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import robot
from transitions import Machine
import tf2_ros
import threading

class SeeingRobot(robot.Robot):

    states = ['resting', 'searching', 'gazing', 'loading', 'connecting', 'unloading']

    def __init__(self):

        super(SeeingRobot, self).__init__()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.field = 'exist'
        self.machine = Machine(model=self, states=SeeingRobot.states, initial='resting')
        self.machine.add_transition('start', 'resting', 'searching')
        self.machine.add_transition('connector_found', 'searching', 'gazing', before='entry', after='exit')
        # self.machine.add_transition('in_reach', 'gazing', 'connecting')
        # self.machine.add_transition('tag_lost', 'connecting', 'unloading', conditions='is_loaded')
        # self.machine.add_transition('unloaded', 'unloading', 'searching')
        # self.machine.add_transition('connected', 'connecting', 'resting')

    def entry(self):
        print 'Enter ', self.state

    def exit(self):
        print 'Exit ', self.state


    def recover(self):
        super(SeeingRobot, self).recover()
        self.machine.set_state('resting')

    def get_transform(self, child, parent, duration):

        def matrix_from_StampedTransform(msg):
            T = msg.transform
            trans = [T.translation.x, T.translation.y, T.translation.z]
            quat = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]

            return tfs.concatenate_matrices(
                tfs.translation_matrix(trans),
                tfs.quaternion_matrix(quat))

        t0 = rospy.Time.now();
        while rospy.Time.now() - t0 < duration:
            try:
                msg = self.tf_buffer.lookup_transform(parent, child, rospy.Time.now(), rospy.Duration(self.dt))
                T = matrix_from_StampedTransform(msg)
                return T, True

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        return None, False


if __name__ == '__main__':

    r = SeeingRobot()

    # scan for female connector
    print("Scanning for connector")
    search_poses = ["search1", "search2", "search1", "search3"]
    pose_idx = 0
    connector_inrange = False
    while not connector_inrange:
        pose_idx = (pose_idx + 1) % len(search_poses)
        pose, success = r.get_transform(search_poses[pose_idx], "map", rospy.Duration(1))
        plan, success = r.plan(r.space_from_matrix(pose), 1)
        if success:
            r.move(plan)
        pose, connector_found = r.get_transform("_female", "map", rospy.Duration(1))
        if connector_found: # has to be reachable
            print "Connector found!"
            plan, connector_inrange = r.plan(r.space_from_matrix(pose), 0.5)
        rospy.sleep(1)


    # move close to connector when in range
    print("Connector is reachable!")

    # sections = 5
    # block_size = len(plan)/sections
    # for k in range(0, sections):
    #     plan2 = plan[k*block_size:(k+1)*block_size]
    #     r.move(plan2)
    #     rospy.sleep(0.5)
    #     pose2, connector_found = r.get_transform("_female", "map", rospy.Duration(1))
    #     if connector_found:
    #         pose = pose2

    r.move(plan)
    pose, connector_found = r.get_transform("_female", "map", rospy.Duration(1))

    # reajusting plan
    print "Reajusting plan"
    if connector_found:
        plan, success = r.plan(r.space_from_matrix(pose), 1)
        if success:
            r.move(plan)


    print("Finalizing")

    rospy.spin()



# if __name__ == '__main__':

#     r = SeeingRobot()

#     count = 0
#     search_poses = ["search1", "search2", "search1", "search3"]

#     search_rate = rospy.Rate(10)
#     while not rospy.is_shutdown():

#         if r.state == 'resting':

#             r.start()

#         elif r.state == 'searching':

#             search_rate.sleep()
#             count = (count + 1) % 4

#             pose, success = r.get_transform(search_poses[count], "map")
#             if success:
#                 plan, success = r.plan(r.space_from_matrix(pose), 1)
#                 r.move(plan)
            
#                 rospy.sleep(3)
#                 pose, success = r.get_transform("_female", "map")
#                 if success:
#                     r.connector_found(pose)
#             #if success: r.tag_found()

#         elif r.state == 'gazing':
#             print 'gazing'


    # rospy.spin()