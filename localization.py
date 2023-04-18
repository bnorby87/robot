from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import rclpy
from rclpy.node import Node
import numpy as np
import utm
from std_msgs.msg import Float32
import math

class Localization(Node):
	#rear_pose = Pose()
	#front_pose = Pose()
	#odom_theta = 0.0
	
	odom_pose = Pose()
 
	reset_deg = 0.0
   
	pose = Pose()
	#x: east, y: north
	theta = 0.0
	global_deg = 0.0
	local_deg = 0.0

	#gps_rear=False
	#gps_front=False
	odom=False
	yawdeg = False
	gnss = False

	def __init__(self):
		super().__init__('localization')
		#self.rear_subscription = self.create_subscription(NavSatFix, '/drotek_rear/fix', self.rear_callback, 10)
		#self.front_subscription = self.create_subscription(NavSatFix, '/drotek_front/fix', self.front_callback, 10)
		#self.odom_subscription = self.create_subscription(PoseStamped, '/pose', self.odom_callback, 1)
		self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom2_callback, 1)
		#self.imu_subscription = self.create_subscription(Float32, '/PX4_yaw_deg', self.yawdeg_callback, 1)
		self.imu_subscription = self.create_subscription(Float32, '/yaw_deg', self.yawdeg_callback, 1)
		#self.pose_subscription = self.create_subscription(Pose, '/PX4_pose', self.pose_callback, 1)
		self.pose_subscription = self.create_subscription(Pose, '/pose', self.pose_callback, 1)



	def odom_callback(self, msg):
		#print("pose x = " + str(msg.pose.position.x))
		#print("pose y = " + str(msg.pose.position.y))
		#print("pose z = " + str(msg.pose.position.z))
		#print("orientacion x = " + str(msg.pose.orientation.x))
		#print("orientacion y = " + str(msg.pose.orientation.y))
		#print("orientacion z = " + str(msg.pose.orientation.z))
		#print("orientacion w = " + str(msg.pose.orientation.w))
		self.odom_pose.position.x = msg.pose.position.x
		self.odom_pose.position.y = msg.pose.position.y
		self.odom = True

	def odom2_callback(self, msg):
		#print("pose x = " + str(msg.pose.position.x))
		#print("pose y = " + str(msg.pose.position.y))
		#print("pose z = " + str(msg.pose.position.z))
		#print("orientacion x = " + str(msg.pose.orientation.x))
		#print("orientacion y = " + str(msg.pose.orientation.y))
		#print("orientacion z = " + str(msg.pose.orientation.z))
		#print("orientacion w = " + str(msg.pose.orientation.w))
		self.odom_pose.position.x = msg.pose.pose.position.x
		self.odom_pose.position.y = msg.pose.pose.position.y
		self.odom = True
  
	def reset(self):	
		self.reset_deg = self.global_deg
		self.local_deg = 0.0


	def yawdeg_callback(self, msg):
		self.global_deg = msg.data
		self.theta = math.radians(msg.data)
		
		x = self.global_deg - self.reset_deg
		if x > 0:
			self.local_deg = x//180*-180+x%180
		else:
			self.local_deg = x//-180*180+x%-180
		self.yawdeg= True
		#print(self.local_deg) 

	def pose_callback(self, msg):
		#print("pose x = " + str(msg.position.x))
		#print("pose y = " + str(msg.position.y))
		#print("pose z = " + str(msg.position.z))
		if  (msg.position.x * msg.position.y) > 1:
			self.pose.position.x = msg.position.x
			self.pose.position.y = msg.position.y
			self.gnss = True

	"""
	def rear_callback(self, msg):
		#print("latitude = " + str(msg.latitude))
		#print("longitude = " + str(msg.longitude))
		#print("altitude = " + str(msg.altitude))
		#print("position = " + str(msg.position_covariance))
		latitude = msg.latitude
		longitude = msg.longitude
		self.rear_pose.position.x, self.rear_pose.position.y = utm.from_latlon(latitude, longitude)[0:2]
		self.gps_rear = True
		self.set_pose()

	def front_callback(self, msg):
		#print("latitude = " + str(msg.latitude))
		#print("longitude = " + str(msg.longitude))
		#print("altitude = " + str(msg.altitude))
		#print("position = " + str(msg.position_covariance))
		latitude = msg.latitude
		longitude = msg.longitude
		self.front_pose.position.x, self.front_pose.position.y = utm.from_latlon(latitude, longitude)[0:2]
		self.gps_front = True
		self.set_pose()
	
	def set_pose(self):
		self.pose.position.x = (self.front_pose.position.x + self.rear_pose.position.x) / 2  
		self.pose.position.y = (self.front_pose.position.y + self.rear_pose.position.y) / 2
		self.theta = np.arctan2(self.front_pose.position.y  - self.rear_pose.position.y , self.front_pose.position.x  - self.rear_pose.position.x)
		#print (math.degrees(self.theta))
	"""

def main():
	rclpy.init(args=None)
	teszt=Localization()
	#rclpy.spin(teszt)
	rclpy.spin_once(teszt)
	print(teszt.theta)
	teszt.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
