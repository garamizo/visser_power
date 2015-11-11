#!/usr/bin/env python
import rospy
from tf import transformations as tfs
import numpy
import math
import robot2 as robot
from transitions import Machine
import tf2_ros
import threading

class SeeingRobot(robot.Robot):

    states = ['resting', 'searching', 'gazing', 'loading', 'connecting', 'unloading']

    def __init__(self):

        super(SeeingRobot, self).__init__()
        self.joints = numpy.array([0, -0.3, 0, 0, 0])
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def entry(self):
        print 'Enter ', self.state

    def exit(self):
        print 'Exit ', self.state


    def recover(self):
        super(SeeingRobot, self).recover()
        self.machine.set_state('resting')

    def get_transform(self, child, parent, when, timeout):

        def matrix_from_StampedTransform(msg):
            T = msg.transform
            trans = [T.translation.x, T.translation.y, T.translation.z]
            quat = [T.rotation.x, T.rotation.y, T.rotation.z, T.rotation.w]

            return tfs.concatenate_matrices(
                tfs.translation_matrix(trans),
                tfs.quaternion_matrix(quat))

        try:
            msg = self.tf_buffer.lookup_transform(parent, child, when, timeout) 
            pose = self.space_from_matrix(matrix_from_StampedTransform(msg))
            return pose, True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        return None, False

    def track(self, target, stop_event):

        exp_factor = 0.1
        while not rospy.is_shutdown() and not stop_event.is_set():
            pose, success = self.get_transform(target, "base", rospy.Time.now() - rospy.Duration(0.1), rospy.Duration(0.1))
            if success:
                try:
                    self.target_pose = exp_factor*pose + (1-exp_factor)*self.target_pose
                except AttributeError:
                    self.target_pose = pose
            stop_event.wait(self.dt)


    def robust_move(self, target):

        # thread to average the pose in the background
        t_stop = threading.Event()
        t = threading.Thread(target=self.track, args=(target, t_stop))
        t.start()

        was_found = False
        while not rospy.is_shutdown():
            pose, on_sight = self.get_transform(target, "base", rospy.Time.now() - rospy.Duration(0.1), rospy.Duration(0))
            if on_sight:
                next_pose = pose
            else:
                try:
                    next_pose = self.target_pose
                except (AttributeError, NameError):  # never saw it
                    was_found = False
                    break

            plan, reachable = self.plan(next_pose, 0.3)
            if reachable:
                print "Still on sight. Moving to it."
                self.move(plan)
                print "Plan length: ", len(plan)
                if len(plan) <= 1:  # nothing will change ever again
                    was_found = True
                    del self.target_pose
                    break

        t_stop.set()
        rospy.sleep(0.2)  # wait for thread to close
        return was_found


    def robust_move2(self, target):

        tinit = rospy.Time.now()
        while not rospy.is_shutdown():

            pose_s, ever_on_sight = self.get_transform(target, "base", rospy.Time(0), rospy.Duration(self.dt))
            if not ever_on_sight:
                return False

            # compute average
            count = 1
            time0 = rospy.Time.now() - rospy.Duration(0.2)
            for k in range(0, 20):
                pose, on_sight = self.get_transform(target, "base", time0-rospy.Duration(0.05*k), rospy.Duration(0))
                if on_sight:
                    pose_s = pose_s + pose
                    count = count + 1
            target_pose = pose_s / count

            # move to inbetween point
            next_pose = 0.3*target_pose + 0.7*self.space_coordinates(self.joints)
            half_plan, reachable = self.plan(next_pose, 0.2)
            if reachable:
                self.move(half_plan)

            error = numpy.linalg.norm(target_pose-self.space_coordinates(self.joints))
            if error < 0.001:
                return True

            if rospy.Time.now() - tinit > rospy.Duration(30):
                return False

        return False

    def match_pose(self, target_id):
        tinit = rospy.Time.now()
        while not rospy.is_shutdown():

            pose, on_sight = self.get_transform(target_id, "base", rospy.Time(0), rospy.Duration(0.1))
            if not on_sight:
                return False

            pose_error = pose - self.space_coordinates(self.joints)
            factor_p = 0.1
            next_pose = self.space_coordinates(self.joints) + factor_p*pose_error

            plan, reachable = self.plan(next_pose, 0.2)
            if reachable:
                self.move(plan)

            if numpy.linalg.norm(pose_error) < 0.02:
                return True

            if rospy.Time.now() - tinit > rospy.Duration(30):
                return False

        return False  




if __name__ == '__main__':

    r = SeeingRobot()

    # try:
        # scan for female connector

    print "Setting up..."
    # get searching poses
    rospy.sleep(2)
    pose1, success = r.get_transform("search1", "base", rospy.Time(0), rospy.Duration(1))
    pose2, success = r.get_transform("search2", "base", rospy.Time(0), rospy.Duration(1))
    pose3, success = r.get_transform("search3", "base", rospy.Time(0), rospy.Duration(1))
    search_poses = [pose1, pose2, pose1, pose3]

    print("Scanning for connector")

    pose_idx = 0
    near_female = False
    while not near_female and not rospy.is_shutdown():
        pose_idx = (pose_idx + 1) % len(search_poses)
        plan, success = r.plan(search_poses[pose_idx], 0.5)
        if success:
            print "Moving to search pose #", pose_idx
            r.move(plan)

        rospy.sleep(1)
        near_female = r.robust_move("_female")

    print "Performing final connection..."
    rospy.sleep(2)
    r.robust_move("female")

    print "Concluded"
    
    rospy.sleep(2)
    r.robust_move("_female")
    plan, success = r.plan(pose1, 0.5)
    r.move(plan)


    # except:
    #     print "Oops!"
    #     pass

    rospy.spin()

# if __name__ == '__main__':

#     try:
#         r = SeeingRobot()

#         # scan for female connector
#         print("Scanning for connector")
#         search_poses = ["search1", "search2", "search1", "search3"]
#         pose_idx = 0
#         connector_inrange = False
#         while not connector_inrange:
#             pose_idx = (pose_idx + 1) % len(search_poses)
#             pose, success = r.get_transform(search_poses[pose_idx], "map", rospy.Duration(1))
#             plan, success = r.plan(r.space_from_matrix(pose), 1)
#             if success:
#                 r.move(plan)

#             pose, connector_found = r.get_transform("_female", "map", rospy.Duration(1))
#             if connector_found: # has to be reachable
#                 print "Connector found!"
#                 plan, connector_inrange = r.plan(r.space_from_matrix(pose), 0.1)
#             rospy.sleep(1)


#         print "Immediate: ", r.space_from_matrix(pose)
#         try:
#             print "From callback: ", r.target_pose
#         except AttributeError:
#             print "target_pose not defined"

#         # move close to connector when in range
#         print("Connector is reachable! Moving to _female")
#         r.move(plan)

#         pose, still_on_sight = r.get_transform("_female", "map", rospy.Duration(1))
#         if still_on_sight:
#             plan, success = r.plan(r.space_from_matrix(pose), 0.3)
#             if success:
#                 print "Still on sight. Moving to it."
#                 r.move(plan)

#         # reajusting plan
#         print "Reajusting plan"
#         rospy.sleep(2)
#         plan, success = r.plan(r.target_pose, 0.3)
#         print r.target_pose
#         if success:
#             r.move(plan)

#         pose, still_on_sight = r.get_transform("_female", "map", rospy.Duration(1))
#         if still_on_sight:
#             plan, success = r.plan(r.space_from_matrix(pose), 0.3)
#             if success:
#                 print "Still on sight. Moving to it."
#                 r.move(plan)


#         print("Finalizing")
        
#     except:
#         pass

#     rospy.spin()

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