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
        self.latest_point = Point()  # To store the latest point
        self.eyes = Eyes()
        os.system('cls' if os.name == 'nt' else 'clear')

    def point_callback(self, msg):
        self.latest_point.x = msg.x
        self.latest_point.y = msg.y
        # self.get_logger().info('Received point: (x: "%s", y: "%s")' % (msg.x, msg.y))
   
        print('    ', end='', flush=True)
        self.eyes.draw_eyes(-self.latest_point.x, self.latest_point.y)
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
