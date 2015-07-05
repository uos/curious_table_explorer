#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import tf

from tf.transformations import quaternion_from_euler

from numpy import *

from std_msgs.msg import *
from geometry_msgs.msg import *

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan

from curious_table_explorer.msg import SurroundPointAction
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal

def real_wait_for_transform(tfl, to_frame, from_frame, time, timeout):
	now= rospy.Time.now()
	while rospy.Time.now() < now+timeout and not tfl.canTransform(to_frame, from_frame, time):
		rospy.sleep(rospy.Duration(.01))

class Surrounder:
	def __init__(self, min_dist= 0.7, max_dist= 1.5):
		self.tfl= tf.TransformListener()

		self.min_dist= min_dist
		self.max_dist= max_dist

		self.pose_pub= rospy.Publisher('/planning_pose', PoseStamped, queue_size= 20)

		rospy.wait_for_service('/move_base_node/make_plan')
		self.move_base_planner= rospy.ServiceProxy('/move_base_node/make_plan', GetPlan)

		self.move_base_client= actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		self.move_base_client.wait_for_server()

		self.move_base_straight_client= actionlib.SimpleActionClient('/move_base_straight', MoveBaseAction)
		self.move_base_client.wait_for_server()

		self.object_recognition_client= actionlib.SimpleActionClient('recognize_objects', ObjectRecognitionAction)
		self.object_recognition_client.wait_for_server()

		self.server= actionlib.SimpleActionServer('surround_point', SurroundPointAction, self.execute, False)
		self.server.start()
		rospy.loginfo('started surround_point action server')

	def check_preempted(self):
		req= self.server.is_preempt_requested() or rospy.is_shutdown()
		if req:
			self.server.set_preempted()
		return req

	def get_plan(self, robot_pose, goal_pose):
		plan= None
		i= 0
		while plan == None and i < 10 and not self.check_preempted():
			try:
				# the tolerance would plan multiple times and this takes too much time here
				plan= self.move_base_planner(start= robot_pose, goal= goal_pose, tolerance= .00).plan
			except rospy.ServiceException:
				rospy.logwarn("failed to call move_base planning service. trying again...")
				i+= 1
		return plan

	def execute(self, goal):
		rospy.loginfo('surrounding point %s' % str(goal.point))

		# provide a useful delta if none was specified in the goal
		if goal.delta == 0.0:
			goal.delta= 0.5

		now= rospy.Time.now()
		robot_pose= PoseStamped( header= Header(stamp= now, frame_id= 'base_footprint'), pose= Pose(orientation= Quaternion(0,0,0,1)) )
		try:
			real_wait_for_transform(self.tfl, 'map', robot_pose.header.frame_id, now, rospy.Duration(5.0))
			robot_pose= self.tfl.transformPose('map', robot_pose)

			real_wait_for_transform(self.tfl, 'map', goal.point.header.frame_id, goal.point.header.stamp, rospy.Duration(5.0))
			goal.point= self.tfl.transformPoint('map', goal.point)
		except tf.Exception:
			rospy.logerr("Failed to lookup map-transforms. Aborting action.")
			self.server.set_aborted()
			return True

		center_point= array([goal.point.point.x, goal.point.point.y])
		robot_vec= array([robot_pose.pose.position.x, robot_pose.pose.position.y]) - center_point
		current_angle= arctan2(robot_vec[1], robot_vec[0])

		for delta in linspace(0.0, 2*pi, 2*pi/goal.delta, endpoint= False):
			angle= (current_angle+delta) % (2*pi)
			vec= array([ cos(angle), sin(angle) ])
			valid_distance_found= 0
			for dist in linspace(self.max_dist, self.min_dist, 10):
				if self.check_preempted():
					return

				rp= center_point+dist*vec
				ro= (angle+pi) % (2*pi)
				test_pose= PoseStamped(header= robot_pose.header, pose= Pose(Point(rp[0], rp[1], 0.0), Quaternion( *quaternion_from_euler(0.0,0.0, ro) )))
				self.pose_pub.publish(test_pose)

				plan= self.get_plan(robot_pose, test_pose)
				if plan == None or len(plan.poses) == 0:
					continue
				self.move_base_client.send_goal_and_wait( MoveBaseGoal(test_pose), rospy.Duration(40.0))
				if self.move_base_client.get_state() != GoalStatus.SUCCEEDED:
					rospy.loginfo("move base failed - (%d) %s" % (self.move_base_client.get_state(), self.move_base_client.get_goal_status_text() ) )
					continue

				# try to get nearer to the target
				for near_dist in linspace(self.min_dist, dist, 5, endpoint= False):
					nrp= center_point+ near_dist*vec
					near_test_pose= PoseStamped(header= robot_pose.header, pose= Pose(Point(nrp[0], nrp[1], 0.0), Quaternion( *quaternion_from_euler(0.0,0.0, ro) )))
					plan= self.get_plan( test_pose, near_test_pose )
					if plan == None or len(plan.poses) == 0:
						continue
					# keep a tolerance of 5cm from the nearst "reachable" point - in practice this leads to collisions
					nrp= center_point+ min(near_dist+.05, dist)*vec
					near_test_pose= PoseStamped(header= robot_pose.header, pose= Pose(Point(nrp[0], nrp[1], 0.0), Quaternion( *quaternion_from_euler(0.0,0.0, ro) )))
					self.move_base_straight_client.send_goal_and_wait( MoveBaseGoal(near_test_pose), rospy.Duration(20.0))
					break

				# object recognition and move_base straight are not strongly synchronized, so make sure we get a clear picture without ego-motion
				rospy.sleep(rospy.Duration(2.5))
				self.object_recognition_client.send_goal_and_wait( ObjectRecognitionGoal(), rospy.Duration() )
				break # next delta
		self.server.set_succeeded()

if __name__ == '__main__':
	rospy.init_node('surrounder')
	server= Surrounder()
	rospy.spin()
