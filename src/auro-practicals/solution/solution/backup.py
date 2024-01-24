import sys
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration

from assessment_interfaces.msg import ItemHolder, ItemHolders
from solution_interfaces.msg import NearestItemTypes
from geometry_msgs.msg import PoseStamped, Item
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

class State(Enum):
    SET_GOAL = 0
    NAVIGATING = 1

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        for x in range(100):
            self.get_logger().info("AHHHHHHHH")
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('robot_name', "")

        self.initial_x = self.get_parameter('x').get_parameter_value().double_value
        self.initial_y = self.get_parameter('y').get_parameter_value().double_value
        self.initial_yaw = self.get_parameter('yaw').get_parameter_value().double_value
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        # self.pos_x = self.initial_x
        # self.pos_y = self.initial_y
        # self.pos_yaw = self.initial_yaw

        self.navigator = BasicNavigator()

        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.initial_x
        initial_pose.pose.position.y = self.initial_y
        initial_pose.pose.orientation.z = self.initial_yaw
        self.navigator.setInitialPose(initial_pose)

        # self.nearest_item = Item()

        self.state = State.SET_GOAL

        self.navigator.waitUntilNav2Active()

        # self.nearest_items_subscriber = self.create_subscription(
        #     NearestItemTypes,
        #     'items/near',
        #     self.nearest_items_callback,
        #     10)
        
        # self.holders_subscriber = self.create_subscription(
        #     ItemHolders,
        #     '/item_holders',
        #     self.item_holder_callback,
        #     10)
        
        # self.odom_subscriber = self.create_subscription(
        #     Odometry,
        #     'odom',
        #     self.odom_callback,
        #     10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    # def nearest_items_callback(self, msg):
    #     self.nearest_item = msg.nearest
    #     self.nearest_blue = msg.blue
    #     self.nearest_green = msg.green
    #     self.nearest_item = None if not self.nearest_item.visible else self.nearest_item
    #     self.nearest_blue = None if not self.nearest_blue.visible else self.nearest_blue
    #     self.nearest_green = None if not self.nearest_green.visible else self.nearest_green
    
    # def item_holder_callback(self, msg):
    #     holders = msg.data
    #     self.holding = ItemHolder()
    #     for holder in holders:
    #         if holder.robot_id == self.robot_name:
    #             self.holding = holder
    #             break
    
    # def odom_callback(self, msg):
    #     (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,
    #                                                 msg.pose.pose.orientation.y,
    #                                                 msg.pose.pose.orientation.z,
    #                                                 msg.pose.pose.orientation.w])
        
    #     self.yaw = -math.degrees(yaw)
    #     if self.yaw < 0:
    #         self.yaw += 360 #normalise to 0-360
        

    # def findBallPosition(self, ball):
    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = 'map'
    #     goal_pose.header.stamp = self.get_clock().now().to_msg()
    #     if ball.visible:
    #         estimated_distance = 69.0 * float(ball.diameter) ** -0.89
    #         angle = self.yaw + 0.095 * ball.x
            
    #         x = estimated_distance * math.cos(math.radians(angle))
    #         y = estimated_distance * math.sin(math.radians(angle))

    #         goal_pose.pose.position.x = x + self.pos_x
    #         goal_pose.pose.position.y = y + self.pos_y
    #         goal_pose.pose.orientation.w = angle
    #     else:
    #         goal_pose.pose.position.x = self.pos_x + 1
    #         goal_pose.pose.position.y = self.pos_y + 1
    #         goal_pose.pose.orientation.w = self.yaw
    #     return goal_pose

    def control_loop(self):
        self.get_logger().info(f"Initial pose - x: {self.initial_x}, y: {self.initial_y}, yaw: {self.initial_yaw}")
        # match self.state:

        #     case State.SET_GOAL:

        #         goal_pose = self.findBallPosition(self.nearest_item)

        #         self.navigator.goToPose(goal_pose)

        #         self.state = State.NAVIGATING
        #     case State.NAVIGATING:

        #         if not self.navigator.isTaskComplete():
        #             feedback = self.navigator.getFeedback()
        #             print('Estimated time of arrival: ' + '{0:.0f}'.format(Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9) + ' seconds.')
        #         else:

        #             result = self.navigator.getResult()

        #             if result == TaskResult.SUCCEEDED:
        #                 print('Goal succeeded!')
        #             elif result == TaskResult.CANCELED:
        #                 print('Goal was canceled!')
        #             elif result == TaskResult.FAILED:
        #                 print('Goal failed!')
        #             else:
        #                 print('Goal has an invalid return status!')

        #     case _:
        #         pass


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