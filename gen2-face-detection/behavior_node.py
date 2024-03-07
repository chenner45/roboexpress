import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from eyes import Eyes
from geometry_msgs.msg import Twist
import os
from datetime import datetime

class StateMachine:
    def __init__(self, mode):
        self.mode = mode # possible values: "stationary", "following"
    
    

class PointSubscriber(Node):
    def __init__(self):
        super().__init__('point_subscriber')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Point,
            'point_topic',
            self.point_callback,
            10)
        self.subscription  # prevent unused variable warning
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.latest_msg = None
        self.latest_msg_time = None
        self.mode = "search"
        self.found_det = False
        self.prev_turn_change_time = None
        self.scan_dir = None
        
    def timer_callback(self):
        # if self.latest_msg is not None:
        msg = Twist()
        # modes: search, track

        if self.mode == "search" and not self.found_det:
            # Scan the environment left to right (stationary)

            # If a detection is found, then stop the robot, switch to the follow
            if self.latest_msg_time is not None and (datetime.now() - self.latest_msg_time).total_seconds() < 2:
                self.mode = "track"
                self.found_det = True
            else:
                # scan left to right over an interval 
                if self.prev_turn_change_time is None or (datetime.now() - self.prev_turn_change_time).total_seconds() >= 3:
                    self.prev_turn_change_time = datetime.now()
                    if self.scan_dir is None:
                        self.scan_dir = 1
                    else:
                        self.scan_dir *= -1
                msg.linear.x = self.scan_dir * 0.01

            
        elif self.mode == "track":
            # If we haven't found a detection in a while, then stop and go back to search
            if (datetime.now() - self.latest_msg_time).total_seconds() > 2:
                self.mode = "search"
                self.found_det = False
                self.prev_turn_change_time = None
            else:
                # reading latest message
                # setting linear velocity: (m/s)
                if self.latest_msg.x < 0:
                    msg.linear.x = -0.01 
                else:
                    msg.linear.x = 0.01
            
        msg.angular.z = 0.0  # Angular velocity (rad/s)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg} for mode: {self.mode}')
        # else:
        #     self.get_logger().info("No input message received yet.")

    def point_callback(self, msg):
        # self.get_logger().info('Received point: (x: "%s", y: "%s")' % (msg.x, msg.y))
        if self.latest_msg is None:
            self.latest_msg = Point()
        self.latest_msg.x = msg.x
        self.latest_msg.y = msg.y
        self.latest_msg_time = datetime.now()

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
