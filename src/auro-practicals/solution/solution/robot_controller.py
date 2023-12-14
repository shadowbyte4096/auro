import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles

from assessment_interfaces.msg import Item, ItemList
from assessment_interfaces.msg import HomeZone
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion

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
        self.items = []
        self.homeMessage = HomeZone()
        
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

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([self.pose.orientation.x,
                                                    self.pose.orientation.y,
                                                    self.pose.orientation.z,
                                                    self.pose.orientation.w])
        
        self.yaw = yaw

    def item_callback(self, items):
        self.nearest_item = None
        for item in items:
            if item.diameter > self.nearest_item.diameter:
                self.nearest_item = item

    def spawn_callback(self, msg):
        self.homeMessage = msg

    def control_loop(self):
        self.get_logger().info(f"State: {self.state}")
        match self.state:
            case State.LOOKING_FOR_BALL:
                if self.nearest_item == None:
                    msg = Twist()
                    msg.angular.z = 1
                    self.cmd_vel_publisher.publish(msg)
                else:
                    self.goal = self.yaw + self.nearest_item.x
                    msg = Twist()
                    msg.angular.z = 0
                    self.cmd_vel_publisher.publish(msg)
            case State.TURNING_TO_GOAL:
                if self.yaw > self.goal + 1:
                    msg = Twist()
                    msg.angular.z = -0.3
                    self.cmd_vel_publisher.publish(msg)
                elif self.yaw < self.goal - 1:
                    msg = Twist()
                    msg.angular.z = 0.3
                    self.cmd_vel_publisher.publish(msg)
                else:
                    msg = Twist()
                    msg.angular.z = 0
                    self.cmd_vel_publisher.publish(msg)
                    self.state = State.HEADING_TO_GOAL
            case State.HEADING_TO_GOAL:
                match self.GoalType:
                    case GoalType.Ball:
                        if not self.holding_ball:
                            msg = Twist()
                            msg.linear.x = 0.3
                            self.cmd_vel_publisher.publish(msg)
                        else:
                            self.state = State.LOOKING_FOR_SPAWN
                            self.GoalType = GoalType.Spawn
                    case GoalType.Spawn:
                        if self.holding_ball:
                            msg = Twist()
                            msg.linear.x = 0.3
                            self.cmd_vel_publisher.publish(msg)
                        else:
                            self.state = State.LOOKING_FOR_BALL
                            self.GoalType = GoalType.Ball
            case State.LOOKING_FOR_SPAWN:
                if not self.homeMessage.visible:
                    msg = Twist()
                    msg.angular.z = 1
                    self.cmd_vel_publisher.publish(msg)
                else:
                    self.goal = self.yaw + self.homeMessage.x
                    msg = Twist()
                    msg.angular.z = 0
                    self.cmd_vel_publisher.publish(msg)


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