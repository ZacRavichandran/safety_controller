import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

class JoyTwistMux(Node):
    def __init__(self):
        super().__init__('joy_twist_mux')
        
        self.auto_cmd_sub = self.create_subscription(
            TwistStamped, '/j100_0000/autonomous/cmd_vel', self.auto_cmd_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, '/j100_0000/joy_teleop/joy', self.joy_callback, 10)
        
        self.auto_mode_pub = self.create_publisher(TwistStamped, '/j100_0000/auto_mode/cmd_vel', 10)

        self.auto_cmd = TwistStamped()

    def auto_cmd_callback(self, msg: TwistStamped):
        self.auto_cmd = msg

    def joy_callback(self, msg: Joy):
        axis_value = msg.axes[4]

        if axis_value == 1.0:
            self.auto_mode_pub.publish(self.auto_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTwistMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
