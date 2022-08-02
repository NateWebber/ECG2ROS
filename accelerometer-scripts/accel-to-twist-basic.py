import rclpy
from rclpy.node import Node
from polar2ros_interfaces.msg import Accelerometer
from geometry_msgs.msg import Twist

"""
This script runs a ROS node to receive accelerometer data from the Polar H10
It then converts that information into a Twist, which it publishes to the topic 'cmd_vel', which controls turtlebots
This is the basic version of the script, that linearly converts accelerometer data into drive commands
This diagram of the three accelerometer axes may be useful:
https://user-images.githubusercontent.com/12439855/149276237-13617f1d-34b1-4757-86e1-8bca0e064953.jpg
"""

class AccelConversionNode(Node):
    def __init__(self):
        super().__init__('accel_conversion_node')
        self.subscriber = self.create_subscription(
            Accelerometer, 'biosensors/polar_h10/ACC_DATA', self.subscribe_callback, 100
            )

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #self.last_message = Accelerometer(x=1000, y=0, z=0) #default of 1000mg (1G) on x, 0 on y and z (this is "stationary")
        self.last_twist = Twist()

    def subscribe_callback(self, accel_data):

        to_publish = Twist()

        """
        the magic number of 600 here is based on subjective observation that values tend to be in the +-600 range
        more flexible users, for instance yoga practitioners or contortionists, may wish to change this value
        """

        #the z axis for the Polar H10 is "forwards" (positive z) and "backwards" (negative z)
        if accel_data.z > 0:
            to_publish.linear.x = 0.26 * abs(accel_data.z / 600)
        elif accel_data.z < 0:
            to_publish.linear.x = -0.26 * abs(accel_data.z / 600)
        else:
            to_publish.linear.x = 0.0

        #the y axis for the Polar H10 is "left" (negative y) and "right" (positive y)
        if accel_data.y < 0:
            to_publish.angular.z = 1.82 * abs(accel_data.y / 600)
        elif accel_data.y > 0:
            to_publish.angular.z = -1.82 * abs(accel_data.y / 600)
        else:
            to_publish.angular.z = 0.0

        self.last_twist = to_publish #in this script we don't actually use the previous message, so it doesn't really need to be here
        #print(f'publishing: {to_publish}')
        self.publisher.publish(to_publish)
        
if __name__ == '__main__':
    rclpy.init()
    converter = AccelConversionNode()
    rclpy.spin(converter)
