import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel_received', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: ' + str(msg.linear.x))
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    
    rclpy.spin(minimal_subscriber)
    

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
