import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from vnpy import *

class VECTNAV_Publisher(Node):
    def __init__(self):
        super().__init__('VECTNAV')
        self.publisher_imu = self.create_publisher(Float32, 'yaw_deg', 1)
    
    def IMU(self):
        msg = Float32()
        s = VnSensor()
        s.connect('/dev/ttyUSB6', 115200)
        print("conn_ok")
        while True:
            ypr = s.read_yaw_pitch_roll()
            msg.data  =  ypr.x
            self.publisher_imu.publish(msg)
            #print(ypr.x)
	

def main(args=None):
    rclpy.init(args=args)
    VN = VECTNAV_Publisher()
    VN.IMU()
	#PX4.destroy_node()
	#rclpy.shutdown()

if __name__ == '__main__':
    main()
