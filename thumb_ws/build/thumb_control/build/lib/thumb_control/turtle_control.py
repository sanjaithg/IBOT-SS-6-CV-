#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')
        self.subscription = self.create_subscription(
            String,
            '/thumb_direction',
            self.direction_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.current_direction = "stop"
        
    def direction_callback(self, msg):
        self.current_direction = msg.data
        self.move_turtle()
        
    def move_turtle(self):
        twist = Twist()
        if self.current_direction == "up":
            twist.linear.x = 1.0
            twist.angular.z = 0.0
        elif self.current_direction == "right":
            twist.linear.x = 0.0
            twist.angular.z = -5.0
        elif self.current_direction == "left":
            twist.linear.x = 0.0
            twist.angular.z = 5.0
        elif self.current_direction == "down":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_control = TurtleControl()
    rclpy.spin(turtle_control)
    turtle_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()