import rospy
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'policy/')))
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from left_or_right_rule_policy import LeftOrRightRule

class ScanTwistCenterControlPolicyNode:

	def __init__(self,scan_topic,pub_topic,policy="LHR",helper_controller=None,**kwargs):

		if policy not in ["LHR","RHR"]:
			raise ValueError("Policy Supported : LHR and RHR only!")

		self.direction = 1 if policy=="LHR" else -1

		self.scan_topic_name = scan_topic
		self.pub_topic_name = pub_topic
		self.dist_to_wall_desired = kwargs['distance_to_wall_desired'] if 'distance_to_wall_desired' in kwargs else 0.0
		self.max_speed = kwargs['max_speed'] if 'max_speed' in kwargs else 0.0

		if policy in ["LHR","RHR"]:
			self.policy = LeftOrRightRule(type=policy)

		if helper_controller:
			self.helper_controller = helper_controller(direction=self.direction,distance_to_wall=self.dist_to_wall_desired)


		self.min_distance = 0.0
		self.angle_with_closest_obstacle = 0.0
		self.distance_front = 0.0
		self.scan_pub = None
		self.cmd_vel_pub = None


	def reset(self):
		self.twist = Twist()
		self.cmd_vel_pub(self.twist)

		self.min_distance = 0.0
		self.angle_with_closest_obstacle = 0.0
		self.distance_front = 0.0

	def start(self):

		self.scan_sub = rospy.Subscriber(self.scan_topic_name,LaserScan,self.call_back)

		self.cmd_vel_pub = rospy.Publisher(self.pub_topic_name,Twist,queue_size=1)

		self.twist = Twist()

		while not rospy.is_shutdown():
			continue
		self.cmd_vel_pub.publish(Twist())	


	def call_back(self,msg):
		self.twist = Twist()

		ranges = [0 for i in range(360)]

		for i in range(180):
			if msg.ranges[i+180]==0:
				ranges[i]=100
			else:
				ranges[i] = msg.ranges[i+180]

			if msg.ranges[i]==0:
				ranges[i+180]=100
			else:
				ranges[i+180]=msg.ranges[i]


		size = len(ranges)

		read_from_idx = size*(self.direction+1)/4
		read_to_idx = size*(self.direction+3)/4

		half_ranges = ranges[read_from_idx:read_to_idx]

		min_idx = ranges.index(min(half_ranges))

		self.distance_front = ranges[size/2]
		self.min_distance = ranges[min_idx]
		self.angle_with_closest_obstacle = (min_idx-size/2)*msg.angle_increment

		if self.helper_controller:
			angular_z = self.helper_controller.step(self.min_distance,self.angle_with_closest_obstacle)
			self.twist.angular.z+=angular_z

		self.execute()

	def execute(self):

		space_ahead = self.distance_front//self.dist_to_wall_desired

		linear_x,angular_z = self.policy.step(space_ahead,self.angle_with_closest_obstacle)

		linear_x*=self.max_speed

		self.twist.linear.x+=linear_x
		self.twist.angular.z+=angular_z

		self.cmd_vel_pub.publish(self.twist)

		print("Execute: ")
		print(self.twist)
		print("***************************************")
