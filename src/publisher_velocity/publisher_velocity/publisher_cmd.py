import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Define a class that inherits from Node to create a ROS2 node
class PublisherCMD(Node):

    def __init__(self):
        # Initialize the node with the name 'publisher_cmd'
        super().__init__('publisher_cmd')
        
        # Create a publisher that will publish messages of type Twist to the '/cmd_vel' topic
        # The QoS (Quality of Service) profile is set to 10
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # message type, topic_name, QoS
        
        # Set the timer period to 0.01 seconds (10 milliseconds)
        timer_period = 0.01  # seconds
        
        # Create a timer that calls the timer_callback function every timer_period seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    

    # Define the callback function that will be called by the timer
    def timer_callback(self):
        
        msg = Twist() # Create a new Twist message
        msg.linear.x = 0.1 # Set the linear x component of the message to 0.1
        self.publisher_.publish(msg) # Publish the message to the '/cmd_vel' topic
        self.get_logger().info('Publishing: "%s"' % msg) # Log the published message

        # Angular velocity
        msg = Twist() 
        msg.angular.x = 0.1
        self.publisher_.publish(msg) 
        self.get_logger().info('Publishing: "%s"' % msg)

# Define the main function that will be executed when the script runs
def main(args=None):
    
    rclpy.init(args=args) # Initialize the rclpy library
    publisher_cmd = PublisherCMD() # Create an instance of the PublisherCMD class
    rclpy.spin(publisher_cmd) # Spin the node to keep it alive and processing callbacks
    publisher_cmd.destroy_node() # Destroy the node explicitly
    rclpy.shutdown() # Shutdown the rclpy library

# Check if the script is being run directly (as the main module)
if __name__ == '__main__':
    main()
