import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from animations import blink, static, heart_eyes

# resize input to smaller size for faster inference
class JoyEyes(Node):
    def __init__(self):
        super().__init__('face_detection_node')
        self.publisher_point = self.create_publisher(Point, 'point_topic', 10)
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        # When pressed, 
        self.smoothing_factor = 0.15
        self.ema_horizontal, self.ema_vertical = 0, 0
        self.static_eyes = False
        self.heart_eyes_bool = False
    def joy_callback(self, msg):
        # right arrow is -1 index 6
        # animate_bool = msg.buttons[4]
        # heart_eyes_cntrl = msg.buttons[5]
        animate_bool = msg.axes[6] == 1
        heart_eyes_cntrl = msg.axes[6] == -1
        
        horizontal = msg.axes[6]
        vertical = msg.axes[7]

        if animate_bool:
            # run animate
            blink()
            self.static_eyes = False
            self.heart_eyes_bool = False
        elif heart_eyes_cntrl:
            if not self.heart_eyes_bool:
                self.heart_eyes_bool = True
                heart_eyes()
        else:
            if not self.static_eyes and not self.heart_eyes_bool:
                static()
                self.static_eyes = True
        # TODO: We want to detect when the horizontal is pressed meaning its value is (-1 or 1), then
        # we want to start publishing the point msg.x = ? where we can take the moving average of the values to get
        # a continuous value
        # Calculate moving average
        # self.ema_horizontal = (horizontal * self.smoothing_factor) + (self.ema_horizontal * (1 - self.smoothing_factor))        
        # self.ema_vertical = (vertical * self.smoothing_factor) + (self.ema_vertical * (1 - self.smoothing_factor))
        
        # msg = Point()
        # msg.x = self.ema_horizontal
        # msg.y = -self.ema_vertical
        # # print(self.ema_horizontal, self.ema_vertical)
        # self.publisher_point.publish(msg)
    def timer_callback(self):
        msg = Point()   
        self.publisher_point.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = JoyEyes()
    print("starting")
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=='__main__':
    main()            
