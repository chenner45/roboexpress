import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from eyes import Eyes
import os
from datetime import datetime
import time
class BlinkTimer():
    def __init__(self, blink_interval, blink_duration):
        self.prev_blink = datetime.now()
        self.blink_interval = blink_interval
        self.blink_duration = blink_duration
        self.cur_time = None

    def time_to_blink(self):
        self.cur_time = datetime.now()
        if (self.cur_time - self.prev_blink).total_seconds() * 1000 < self.blink_interval:
            return False

        if (self.cur_time - self.prev_blink).total_seconds() * 1000 > self.blink_interval + self.blink_duration:
            self.prev_blink = self.cur_time
            return False
        return True

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

        self.blink_timer = BlinkTimer(3000, 500) # value in ms for how long to wait between blinks

    def point_callback(self, msg):
        self.latest_point.x = msg.x
        self.latest_point.y = msg.y
        # self.get_logger().info('Received point: (x: "%s", y: "%s")' % (msg.x, msg.y))
   
        print('    ', end='', flush=True)

        # to blink or not to blink
        oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, _, _ = self.eyes.calc_eyes(-self.latest_point.x, self.latest_point.y)
        if self.blink_timer.time_to_blink():
            time_from_blink = (self.blink_timer.cur_time - self.blink_timer.prev_blink).total_seconds() - self.blink_timer.blink_interval / 1000

            if time_from_blink <= (self.blink_timer.blink_duration / 1000) / 2:
                y_restrict = (2 * time_from_blink / (self.blink_timer.blink_duration / 1000)) * self.eyes.height
            else:
                y_restrict = (2 - 2 * time_from_blink / (self.blink_timer.blink_duration / 1000)) * self.eyes.height

            self.eyes.draw_eyes(oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, 0, y_restrict)

        else:
            self.eyes.draw_eyes(oval1_center, oval2_center, oval1_radius_x, oval2_radius_x, oval1_radius_y, oval2_radius_y, 0, 0)

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
