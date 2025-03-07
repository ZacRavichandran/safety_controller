import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistStamped

class JoyTwistMux(Node):
    def __init__(self):
        super().__init__('joy_twist_mux')
        
        self.joy_sub = self.create_subscription(Joy, '/joy_teleop/joy', self.joy_callback, 10)
        self.jackal_sub = self.create_subscription(Twist, '/jackal_velocity_controller/cmd_vel_unstamped', self.jackal_callback, 10)
        self.mpc_sub = self.create_subscription(Twist, '/planners/mpc_cmd_vel_unstamped', self.mpc_callback, 10)
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/jackal_velocity_controller/cmd_vel', 10)
        
        self.axis_5_value = None
        self.controller_cmd = None
        self.mpc_cmd = None

        self.zero = Twist()
    
    def joy_callback(self, msg: Joy):
        self.axis_5_value = msg.axes[5]
        self.publish_selected_cmd()
    
    def jackal_callback(self, msg: Twist):
        self.controller_cmd = msg
        self.publish_selected_cmd()
    
    def mpc_callback(self, msg: Twist):
        self.mpc_cmd = msg
        self.publish_selected_cmd()
    
    def publish_selected_cmd(self):
        twist_msg = None
        
        if self.axis_5_value == 1 and self.mpc_cmd is not None:
            twist_msg = self.mpc_cmd
        elif self.axis_5_value == -1 and self.controller_cmd is not None:
            twist_msg = self.controller_cmd
        else:
            twist_msg = self.zero
        
        stamped_msg = TwistStamped()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'
        stamped_msg.twist = twist_msg
        self.cmd_pub.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyTwistMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
