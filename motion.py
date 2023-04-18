from asyncore import write
import imp
from multiprocessing.connection import wait
from rclpy.node import Node
from geometry_msgs.msg import Twist
import rclpy
from rclpy.qos import QoSProfile
import math
from localization import Localization
import numpy as np
import utm
import csv
import time

class Motion(Node):
	robot = None
	
	def __init__(self):
		super().__init__('motion')
		self._publisher = self.create_publisher(Twist, '/cmd_vel', QoSProfile(depth=10))
		self.robot=Localization()
		while not (self.robot.odom and self.robot.yawdeg and self.robot.gnss):			
			print("waiting")
			rclpy.spin_once(self.robot)
   
		self.robot.reset()
		rclpy.spin_once(self.robot)

		print("okker")
	
	def create_twist(self, lin_x=0.0, lin_y=0.0, lin_z=0.0, ang_x=0.0, ang_y=0.0, ang_z=0.0):
		twist = Twist()
		twist.linear.x = lin_x
		twist.linear.y = lin_y
		twist.linear.z = lin_z
		twist.angular.x = ang_x
		twist.angular.y = ang_y
		twist.angular.z = ang_z
		return twist
	
	def stop_twist(self):
		twist = self.create_twist()
		self._publisher.publish(twist)
	
	def turn(self, speed, goal_angle):
		if goal_angle >= 179:
			goal_angle = 179.5
		elif goal_angle <= -179:
			goal_angle = 179.5
		rclpy.spin_once(self.robot)
		curr_angle = self.robot.local_deg
		diff_angle = goal_angle - curr_angle
		print("diff",diff_angle)
		if diff_angle > 0 and diff_angle <= 180:
			while self.robot.local_deg < goal_angle:
				self.turn_one(speed=-speed)
		elif diff_angle < 0 and diff_angle >= -180:
			while self.robot.local_deg > goal_angle:
				self.turn_one(speed=speed)
		elif diff_angle > 180:
			while self.robot.local_deg < 0 or self.robot.local_deg > goal_angle:
				self.turn_one(speed=speed)
		elif diff_angle < -180:
			while self.robot.local_deg > 0 or self.robot.local_deg < goal_angle:
				self.turn_one(speed=-speed)
		self.stop_twist()

		print(self.robot.local_deg)

	def PID_turn(self, goal_angle, eps = 2.0, stop=True, local=False):
		kp_angular = 0.02
		kd_angular = 0.002
		ki_angular = 0.0

		prev_error_angle = 0
		angle = goal_angle
		if local:
			error_angle = angle - self.robot.local_deg
		else:
			error_angle = angle - self.robot.global_deg
  
		while abs(error_angle) > eps:
			angular_velocity_control = kp_angular*error_angle + kd_angular*(error_angle - prev_error_angle)
			self.prev_error_angle = error_angle
			if angular_velocity_control > 1.0:
				angular_velocity_control = 1.0
			elif angular_velocity_control < -1.0:
				angular_velocity_control = -1.0
			twist = self.create_twist(ang_z=-angular_velocity_control)
			self._publisher.publish(twist)
			rclpy.spin_once(self.robot)
			if local:
				error_angle = angle - self.robot.local_deg
			else:
				error_angle = angle - self.robot.global_deg
		
		if stop:
			self.stop_twist()

	def turn_one(self, speed):
			twist = self.create_twist(ang_z=speed)
			self._publisher.publish(twist)
			rclpy.spin_once(self.robot)

	
	def straight(self, speed, dist):
		rclpy.spin_once(self.robot)
		orig_x = self.robot.odom_pose.position.x
		orig_y = self.robot.odom_pose.position.y
		while self.distance(orig_x,orig_y) < abs(dist):
			print(self.robot.odom_pose.position.x, self.robot.odom_pose.position.y)
			twist = self.create_twist(lin_x=abs(speed))
			self._publisher.publish(twist)
			rclpy.spin_once(self.robot)
			print("dist:", self.distance(orig_x,orig_y))
		self.stop_twist()
		#print(self.pos_x, self.pos_y)

	def distance(self, x, y):
		rclpy.spin_once(self.robot)
		dist = math.sqrt((self.robot.odom_pose.position.x-x)**2+(self.robot.odom_pose.position.y-y)**2)
		return dist

	def distance_UTM(self, x, y):
		rclpy.spin_once(self.robot)
		dist = math.sqrt((self.robot.pose.position.x-x)**2+(self.robot.pose.position.y-y)**2)
		return dist

	def PID_straight(self, dist, goal_angle = 0.0, forward=True):
		kp_linear = 0.6
		kd_linear = 0.06
		ki_linear = 0.0
		
		kp_angular = 0.02
		kd_angular = 0.002
		ki_angular = 0.0

		prev_error_position = 0
		prev_error_angle = 0
		orig_x = self.robot.odom_pose.position.x
		orig_y = self.robot.odom_pose.position.y
		
		if forward:
			angle = self.robot.local_deg
		else:
			angle = goal_angle
   
		while self.distance(orig_x,orig_y) < abs(dist):
			print("dist", self.distance(orig_x,orig_y))
			error_position = dist - self.distance(orig_x,orig_y)
			error_angle = angle - self.robot.local_deg

			linear_velocity_control = kp_linear*error_position + kd_linear*(error_position - prev_error_position)
			angular_velocity_control = kp_angular*error_angle + kd_angular*(error_angle - prev_error_angle)

			self.prev_error_angle = error_angle
			self.prev_error_position = error_position
			if linear_velocity_control > 1.0:
				linear_velocity_control = 1.0
			elif linear_velocity_control < 0.1 and linear_velocity_control >= 0:
				linear_velocity_control = 0.1
			elif linear_velocity_control > -0.1 and linear_velocity_control < 0:
				linear_velocity_control = -0.1
			elif linear_velocity_control < -1.0:
				linear_velocity_control = -1.0

			if angular_velocity_control > 1.0:
				angular_velocity_control = 1.0
			elif angular_velocity_control < -1.0:
				angular_velocity_control = -1.0

			print(angular_velocity_control)
			twist = self.create_twist(lin_x=linear_velocity_control, ang_z=-angular_velocity_control)
			self._publisher.publish(twist)
			rclpy.spin_once(self.robot)
   
		self.stop_twist()
  
	def PID_GOTO_UTM(self, goal_x, goal_y, eps=0.1):
		kp_linear = 0.6
		kd_linear = 0.06
		ki_linear = 0.0
		
		kp_angular = 0.06
		kd_angular = 0.006
		ki_angular = 0.0

		prev_error_position = 0
		prev_error_angle = 0

		actX =  self.robot.pose.position.x
		actY =  self.robot.pose.position.y
		angle = math.degrees(np.arctan2(goal_x-self.robot.pose.position.x,goal_y-self.robot.pose.position.y))

		print(angle, self.robot.global_deg, self.robot.pose.position.x, self.robot.pose.position.y)
		self.PID_turn(goal_angle=angle, eps = 2.0, stop=True, local=False)
		print(angle, self.robot.global_deg)

		while self.distance_UTM(goal_x, goal_y) > eps:
			error_position = self.distance_UTM(goal_x, goal_y)
			angle = math.degrees(np.arctan2(goal_x-self.robot.pose.position.x,goal_y-self.robot.pose.position.y))

			error_angle = angle - self.robot.global_deg
			#print(error_position)

			linear_velocity_control = kp_linear*error_position + kd_linear*(error_position - prev_error_position)
			angular_velocity_control = kp_angular*error_angle + kd_angular*(error_angle - prev_error_angle)

			self.prev_error_angle = error_angle
			self.prev_error_position = error_position

			self.prev_error_angle = error_angle
			self.prev_error_position = error_position
			if linear_velocity_control > 1.5:
				linear_velocity_control = 1.5
			elif linear_velocity_control < 0.1 and linear_velocity_control >= 0:
				linear_velocity_control = 0.1
			elif linear_velocity_control > -0.1 and linear_velocity_control < 0:
				linear_velocity_control = -0.1
			elif linear_velocity_control < -1.0:
				linear_velocity_control = -1.0

			if angular_velocity_control > 2.0:
				angular_velocity_control = 2.0
			elif angular_velocity_control < -2.0:
				angular_velocity_control = -2.0
			if error_position < 2.0:
				linear_velocity_control = linear_velocity_control / 10
				angular_velocity_control = angular_velocity_control /10
			twist = self.create_twist(lin_x=linear_velocity_control, ang_z=-angular_velocity_control)
			self._publisher.publish(twist)
			rclpy.spin_once(self.robot)
   
		self.stop_twist()

def main():
	rclpy.init(args=None)
	rosbot=Motion()
	#rosbot.turn(speed=0.3, goal_angle=45)
	rosbot.straight(speed=0.5, dist=1.5)
	#rosbot.turn(speed=0.3, goal_angle=90)
	#rosbot.turn(speed=1.0, goal_angle=-90)
	#rosbot.PID_straight(dist=1.0, goal_angle=-45.0, forward=False)
	#rosbot.PID_GOTO_LOCAL(goal_x=1.0, goal_y=1.0, eps=0.2)
	#print(rosbot.robot.odom_pose.position.x, rosbot.robot.odom_pose.position.y)
	#rosbot.PID_turn(goal_angle=5.0, local=True)
	#print(rosbot.robot.global_deg, rosbot.robot.odom_pose.position.x, rosbot.robot.odom_pose.position.y)
	#time.sleep(5)
	#rosbot.PID_straight(dist=0.0, forward=True)
	#print(rosbot.robot.odom_pose.position.x, rosbot.robot.odom_pose.position.y)
	#rosbot.PID_turn(goal_angle=10, local= True)
	#rosbot.PID_straight(dist=10.0, forward=True)
 
	#rosbot.PID_GOTO_UTM(goal_x= 697133.0729985585, goal_y=  5285779.006394195, eps=0.3)
	#rosbot.PID_GOTO_UTM(goal_x=697146.7707286837, goal_y= 5285782.630980955, eps=0.2)
	#rosbot.PID_turn(goal_angle=92.2, eps = 1.0, stop=True, local=False)
	rosbot.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
