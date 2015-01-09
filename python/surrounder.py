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

from my_table_objects.msg import SurroundPointAction
from object_recognition_msgs.msg import ObjectRecognitionAction, ObjectRecognitionGoal

class Surrounder:
	def __init__(self):
		self.tfl= tf.TransformListener()
		self.pose_pub= rospy.Publisher('/planning_pose', PoseStamped, queue_size= 20)

		rospy.wait_for_service('/move_base_node/make_plan')
		self.move_base_planner= rospy.ServiceProxy('/move_base_node/make_plan', GetPlan)

		self.move_base_client= actionlib.SimpleActionClient('/move_base', MoveBaseAction)
		self.move_base_client.wait_for_server()

		self.object_recognition_client= actionlib.SimpleActionClient('recognize_objects', ObjectRecognitionAction)
		self.object_recognition_client.wait_for_server()

		self.server= actionlib.SimpleActionServer('surround_point', SurroundPointAction, self.execute, False)
		self.server.start()
		rospy.loginfo('started surround_point action server')

	def check_preempted(self):
		req= self.server.is_preempt_requested()
		if req:
			self.server.set_preempted()
		return req

	def get_plan(self, robot_pose, goal_pose):
		plan= None
		i= 0
		while plan == None and i < 10:
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
			self.tfl.waitForTransform('map', robot_pose.header.frame_id, now, rospy.Duration(5.0))
			robot_pose= self.tfl.transformPose('map', robot_pose)

			self.tfl.waitForTransform('map', goal.point.header.frame_id, goal.point.header.stamp, rospy.Duration(5.0))
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
			for dist in linspace(0.7, 1.5, 7):
				if self.check_preempted():
					return

				rp= center_point+dist*vec
				ro= (angle+pi) % (2*pi)
				test_pose= PoseStamped(header= robot_pose.header, pose= Pose(Point(rp[0], rp[1], 0.0), Quaternion( *quaternion_from_euler(0.0,0.0, ro) )))
				self.pose_pub.publish(test_pose)

				plan= self.get_plan(robot_pose, test_pose)
				if plan == None:
					continue

				if len(plan.poses) > 0:
					# skip the first two found solutions - They are too near to obstacles to approach in practice
					if valid_distance_found < 2:
						valid_distance_found+= 1
						continue
					rospy.loginfo("found working dist %s for angle %s" % (dist, angle))
					self.move_base_client.send_goal_and_wait( MoveBaseGoal(test_pose), rospy.Duration(40.0))
					if self.move_base_client.get_state() == GoalStatus.SUCCEEDED:
						# move_base often returns one or two seconds _before_ it stopped moving. Make sure we get a "clean" picture for recognition
						rospy.sleep(rospy.Duration(2.5))
						self.object_recognition_client.send_goal_and_wait( ObjectRecognitionGoal(), rospy.Duration() )
						break
					else:
						rospy.loginfo("%d - %s" % (self.move_base_client.get_state(), self.move_base_client.get_goal_status_text()))
		self.server.set_succeeded()

if __name__ == '__main__':
	rospy.init_node('surrounder')
	server= Surrounder()
	rospy.spin()
