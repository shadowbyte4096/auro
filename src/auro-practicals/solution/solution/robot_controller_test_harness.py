import sys
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration

from assessment_interfaces.msg import ItemHolder, ItemHolders
from solution_interfaces.msg import NearestItemTypes, Item
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

class State(Enum):
    SET_GOAL = 0
    NAVIGATING = 1

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('robot_name', "")

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        #self.get_logger().info(f"x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}, name: {self.robot_name}")

        self.pos_x = self.initial_x
        self.pos_y = self.initial_y
        self.pos_yaw = self.initial_yaw
        self.yaw = self.initial_yaw

        self.nearest_red = Item()

        self.initial_yaw = None
        self.initial_x = None

        self.nearest_item = Item()

        # self.navigator = BasicNavigator()

        # initial_pose = PoseStamped()
        # initial_pose.header.frame_id = 'map'
        # initial_pose.header.stamp = self.get_clock().now().to_msg()
        # initial_pose.pose.position.x = float(self.initial_x)
        # initial_pose.pose.position.y = float(self.initial_y)
        # initial_pose.pose.orientation.z = float(self.initial_yaw)
        # self.navigator.setInitialPose(initial_pose)

        # self.state = State.SET_GOAL

        # self.navigator.waitUntilNav2Active()

        self.nearest_items_subscriber = self.create_subscription(
            NearestItemTypes,
            'items/near',
            self.nearest_items_callback,
            10)
        
        self.holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holder_callback,
            10)
        
        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def nearest_items_callback(self, msg):
        self.nearest_item = msg.nearest
        self.nearest_blue = msg.blue
        self.nearest_green = msg.green
        self.nearest_red = msg.red
    
    def item_holder_callback(self, msg):
        holders = msg.data
        self.holding = ItemHolder()
        for holder in holders:
            if holder.robot_id == self.robot_name:
                self.holding = holder
                break
    
    def odom_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w])
        self.yaw = -math.degrees(yaw)
        if self.yaw < 0:
            self.yaw += 360 #normalise to 0-360

    def control_loop(self):
        # self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
        # if (self.yaw < 30):
        #     msg = Twist()
        #     self.turn_direction = -1
        #     msg.angular.z = 0.01 * self.turn_direction
        #     self.cmd_vel_publisher.publish(msg)
        # else:
        #     msg = Twist()
        #     self.cmd_vel_publisher.publish(msg)
        if (self.nearest_item.visible):
            # if (self.initial_x == None):
            #     self.initial_yaw = self.yaw
            #     self.initial_x = self.nearest_item.x
            self.get_logger().info(f"Current x: {self.nearest_item.x}")
            self.get_logger().info(f"Current y: {self.yaw}")


    def destroy_node(self):
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()