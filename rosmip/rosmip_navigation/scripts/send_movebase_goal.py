#!/usr/bin/env python

import math, rospy, move_base_msgs.msg
import actionlib
from actionlib_msgs.msg import *
import tf

class GoForwardAvoid():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
	rospy.on_shutdown(self.shutdown)
	
	#tell the action client that we want to spin a thread by default
	self.move_base = actionlib.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)
	rospy.loginfo("wait for the action server to come up")
	#allow up to 5 seconds for the action server to come up
	self.move_base.wait_for_server(rospy.Duration(5))

        goals = [[1.75, 0.25, math.pi/2],
                 [1.75, 1.75, math.pi],
                 [0.25, 1.75, 1.5*math.pi],
                 [0.25, 0.25, 0]]

        for g in goals:
            self.send_goal(g)

    def send_goal(self, g):
        rospy.loginfo("Sending goal {}".format(g))
        goal = move_base_msgs.msg.MoveBaseGoal()
	goal.target_pose.header.frame_id = 'map'
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = g[0] #
        goal.target_pose.pose.position.y = g[1] #
        q = tf.transformations.quaternion_from_euler(0, 0, g[2])
        o = goal.target_pose.pose.orientation
	o.x, o.y, o.z, o.w = q
        self.move_base.send_goal(goal)
	success = self.move_base.wait_for_result(rospy.Duration(60)) 
	if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("The base failed to move for some reason")
    	else:
	    # We made it!
	    state = self.move_base.get_state()
	    if state == GoalStatus.SUCCEEDED:
		rospy.loginfo("Hooray, the base moved to the goal")



    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    try:
        GoForwardAvoid()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")

