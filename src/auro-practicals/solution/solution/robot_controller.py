import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles
from enum import Enum

from assessment_interfaces.msg import Item, ItemList
from assessment_interfaces.msg import HomeZone
from assessment_interfaces.msg import ItemHolder, ItemHolders
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion

X_TO_YAW_MULT = -0.261
GOAL_YAW_ACCEPTABLE_RANGE = 0.3
TURNING_SPEED = 0.1
FAST_TURNING_SPEED = 0.2
FORWARD_SPEED = 0.3

class State(Enum):
    LOOKING_FOR_BALL = 0
    LOOKING_FOR_SPAWN = 1
    TURNING_TO_GOAL = 2
    HEADING_TO_GOAL = 3

class GoalType(Enum):
    Ball = 0
    Spawn = 1

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.get_logger().info(f"STARTING ROBOT CONTROLLER")

        self.state = State.LOOKING_FOR_BALL
        self.GoalType = GoalType.Ball
        self.goal = 0.0
        self.yaw = 0.0
        self.homeMessage = HomeZone()
        self.nearest_item = None
        self.holding_ball = False
        self.delay = True
        self.delayTimer = -10
        
        self.item_subscriber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.holders_subscriber = self.create_subscription(
            ItemHolders,
            '/item_holders',
            self.item_holder_callback,
            10)
        
        self.home_subscriber = self.create_subscription(
            HomeZone,
            'home_zone',
            self.spawn_callback,
            10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w])
        
        #self.get_logger().info(f"AAAAHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH!!") 
        self.yaw = yaw * 144 #i thought it was in radiens but now it goes to >400 so god knows


    def item_callback(self, items):
        self.nearest_item = None
        for item in items.data:
            if self.nearest_item == None:
                self.nearest_item = item
            elif item.diameter > self.nearest_item.diameter:
                self.nearest_item = item
    
    def item_holder_callback(self, msg):
        holders = msg.data
        self.holding_ball = False
        for holder in holders:
            if holder.holding_item:
                self.holding_ball = True

    def spawn_callback(self, msg):
        self.homeMessage = msg

    def control_loop(self):
        ####  difference in yaw = item.x * -0.261  ######


        self.get_logger().info(f"State: {self.state}")

        if (self.delay):
            self.delayTimer += 1
            if self.delayTimer > 3:
                self.delay = False
                self.delayTimer = 0
        else:
            match self.state:
                case State.LOOKING_FOR_BALL:
                    if self.nearest_item == None:
                        msg = Twist()
                        msg.angular.z = FAST_TURNING_SPEED
                        self.cmd_vel_publisher.publish(msg)
                    else:
                        self.goal = self.yaw + (self.nearest_item.x * X_TO_YAW_MULT)
                        msg = Twist()
                        msg.angular.z = 0.0
                        self.cmd_vel_publisher.publish(msg)
                        self.state = State.TURNING_TO_GOAL
                        self.delay = True
                case State.TURNING_TO_GOAL:
                    if self.nearest_item != None:
                        self.get_logger().info(f"Item: {self.nearest_item.x * X_TO_YAW_MULT}")
                    self.get_logger().info(f"Yaw: {self.yaw}, Goal: {self.goal}")
                    if self.yaw > self.goal + GOAL_YAW_ACCEPTABLE_RANGE:
                        msg = Twist()
                        msg.angular.z = -TURNING_SPEED
                        self.cmd_vel_publisher.publish(msg)
                    elif self.yaw < self.goal - GOAL_YAW_ACCEPTABLE_RANGE:
                        msg = Twist()
                        msg.angular.z = TURNING_SPEED
                        self.cmd_vel_publisher.publish(msg)
                    else:
                        msg = Twist()
                        msg.angular.z = 0.0
                        self.cmd_vel_publisher.publish(msg)
                        self.state = State.HEADING_TO_GOAL
                        self.delay = True
                case State.HEADING_TO_GOAL:
                    self.get_logger().info(f"Yaw: {self.yaw}, Goal: {self.goal}")
                    match self.GoalType:
                        case GoalType.Ball:
                            if not self.holding_ball:
                                msg = Twist()
                                msg.linear.x = FORWARD_SPEED
                                self.cmd_vel_publisher.publish(msg)
                            else:
                                self.state = State.LOOKING_FOR_SPAWN
                                self.GoalType = GoalType.Spawn
                                self.delay = True
                        case GoalType.Spawn:
                            if self.holding_ball:
                                msg = Twist()
                                msg.linear.x = FORWARD_SPEED
                                self.cmd_vel_publisher.publish(msg)
                            else:
                                self.state = State.LOOKING_FOR_BALL
                                self.GoalType = GoalType.Ball
                                self.delay = True
                case State.LOOKING_FOR_SPAWN:
                    self.get_logger().info(f"homezoneVisible: {self.homeMessage.visible}")
                    self.get_logger().info(f"homezoneX: {self.homeMessage.x}") 
                    if not self.homeMessage.visible:
                        msg = Twist()
                        msg.angular.z = FAST_TURNING_SPEED
                        self.cmd_vel_publisher.publish(msg)
                    else:

                        self.goal = self.yaw + (self.homeMessage.x * -X_TO_YAW_MULT)
                        msg = Twist()
                        msg.angular.z = 0.0
                        self.cmd_vel_publisher.publish(msg)
                        self.state = State.TURNING_TO_GOAL
                        self.delay = True
    
        

    def destroy_node(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(f"Stopping: {msg}")
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