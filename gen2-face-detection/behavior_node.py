import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from eyes import Eyes
import os

class PointSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'point_topic',
            self.point_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def point_callback(self, msg):
        self.get_logger().info('Received point: (x: "%s", y: "%s")' % (msg.x, msg.y))
   
def main(args=None):
    rclpy.init(args=args)
    point_subscriber = PointSubscriber()
    try:
        rclpy.spin(point_subscriber)
    except KeyboardInterrupt:
        pass  # Handle Ctrl+C gracefully
    finally:
        # Cleanup and shutdown
        point_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
