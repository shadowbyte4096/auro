import sys
import math
import random

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSPresetProfiles
from enum import Enum

from assessment_interfaces.msg import HomeZone
from assessment_interfaces.msg import ItemHolder, ItemHolders
from solution_interfaces.msg import NearestItemTypes
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion

#X_TO_YAW_MULT = -0.261
#GOAL_YAW_ACCEPTABLE_RANGE = 0.3
TURNING_SPEED = 2.0
FORWARD_SPEED = 0.3
OBJECT_AVOIANCE_FACTOR_SIDES = -0.2
OBJECT_AVOIANCE_FACTOR_FRONT = 0.2
LAST_SEEN_GOAL_TIMER = 10

TURN_LEFT = 1 # Postive angular velocity turns left
TURN_RIGHT = -1 # Negative angular velocity turns right

class State(Enum):
    LOOKING_FOR_BALL = 0
    LOOKING_FOR_SPAWN = 1
    HEADING_TO_BALL = 2
    HEADING_TO_SPAWN = 3

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        self.get_logger().info(f"STARTING ROBOT CONTROLLER")

        self.state = State.LOOKING_FOR_BALL
        self.goal = 0.0
        self.yaw = 0.0
        self.homeMessage = HomeZone()
        self.nearest_item = None
        self.holding = ItemHolder()
        self.delay = True
        self.delayTimer = -10
        self.colour_filter = None
        self.object_avoidance_sides = 0.0
        self.object_avoidance_front = 0.0
        self.time_since_goal_seen = 0
        self.turn_direction = TURN_LEFT
                
        self.nearest_items_subscriber = self.create_subscription(
            NearestItemTypes,
            'items/near',
            self.nearest_items_callback,
            10)

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value)

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

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def odom_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w])
        
        #self.yaw = yaw * 144
        self.yaw = -math.degrees(yaw)
        if self.yaw < 0:
            self.yaw += 360 #normalise to 0-360
    
    def nearest_items_callback(self, msg):
        self.nearest_item = msg.nearest
        self.nearest_blue = msg.blue
        self.nearest_green = msg.green
        self.nearest_item = None if not self.nearest_item.visible else self.nearest_item
        self.nearest_blue = None if not self.nearest_blue.visible else self.nearest_blue
        self.nearest_green = None if not self.nearest_green.visible else self.nearest_green
    
    def item_holder_callback(self, msg):
        holders = msg.data
        self.holding = ItemHolder()
        for holder in holders:
            self.holding = holder

    def spawn_callback(self, msg):
        self.homeMessage = msg
    
    def scan_callback(self, msg):
        left_object  = min(msg.ranges[0:90])
        right_object = min(msg.ranges[271:360])
        self.object_avoidance_sides = (right_object ** OBJECT_AVOIANCE_FACTOR_SIDES) - (left_object ** OBJECT_AVOIANCE_FACTOR_SIDES)
        front_object = min(msg.ranges[351:359] + msg.ranges[0:10])
        self.object_avoidance_front = 1 - 3 ** -front_object



    def find_better_ball(self):
        if (self.nearest_blue != None) and (self.holding.item_colour != "BLUE"):
            self.colour_filter = "BLUE"
            self.state = State.HEADING_TO_BALL
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)
        elif (self.nearest_green != None) and (self.holding.item_colour == "RED"):
            self.colour_filter = "GREEN"
            self.state = State.HEADING_TO_BALL
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)

    def heading_for_ball_decision(self):
        if self.colour_filter == None:
            if self.holding.holding_item:
                return True
            elif self.nearest_item == None:
                self.time_since_goal_seen += 1
                if self.time_since_goal_seen > LAST_SEEN_GOAL_TIMER:
                    self.time_since_goal_seen = 0
                    return True
        else:
            if self.holding.item_colour == self.colour_filter:
                return True
            elif (self.colour_filter == "BLUE") and (self.nearest_blue == None):
                self.time_since_goal_seen += 1
                if self.time_since_goal_seen > LAST_SEEN_GOAL_TIMER:
                    self.time_since_goal_seen = 0
                    return True
            elif (self.colour_filter == "GREEN") and (self.nearest_green == None):
                self.time_since_goal_seen += 1
                if self.time_since_goal_seen > 5:
                    self.time_since_goal_seen = 0
                    return True
        return False
                    
    def look_for_ball(self):
        if self.nearest_item == None:
            msg = Twist()
            msg.angular.z = TURNING_SPEED * self.turn_direction
            self.cmd_vel_publisher.publish(msg)
        else:
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)
            self.state = State.HEADING_TO_BALL
            self.delay = True

    def look_for_spawn(self):
        if not self.homeMessage.visible:
            msg = Twist()
            msg.angular.z = TURNING_SPEED * self.turn_direction
            self.cmd_vel_publisher.publish(msg)
            self.find_better_ball()
        else:
            msg = Twist()
            msg.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg)
            self.state = State.HEADING_TO_SPAWN
            self.delay = True

    def head_to_ball(self):
        not_reached_goal = self.heading_for_ball_decision()
        if not_reached_goal:
            self.colour_filter = None
            msg = Twist()
            self.cmd_vel_publisher.publish(msg)
            self.state = State.LOOKING_FOR_SPAWN
            self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
            self.find_better_ball()
            self.delay = True
        else:
            nearest = self.nearest_item
            match self.colour_filter:
                case "BLUE":
                    nearest = self.nearest_blue
                case "GREEN":
                    nearest = self.nearest_green
            if (nearest != None):
                estimated_distance = 69.0 * float(nearest.diameter) ** -0.89

                msg = Twist()
                msg.linear.x = (FORWARD_SPEED + 0.25 * estimated_distance) * self.object_avoidance_front
                msg.angular.z = nearest.x / 320.0 + self.object_avoidance_sides

                self.cmd_vel_publisher.publish(msg)
            else:
                msg = Twist()
                msg.linear.x = FORWARD_SPEED * self.object_avoidance_front
                self.cmd_vel_publisher.publish(msg)
    
    def head_to_spawn(self):
        if self.holding.holding_item:
            if (self.homeMessage.visible):
                msg = Twist()
                msg.linear.x = FORWARD_SPEED * self.object_avoidance_front
                msg.angular.z = self.homeMessage.x / 320.0 + self.object_avoidance_sides

                self.cmd_vel_publisher.publish(msg)
            else:
                msg = Twist()
                msg.angular.z = TURNING_SPEED * self.turn_direction
                self.cmd_vel_publisher.publish(msg)
        else:
            self.colour_filter = None
            self.state = State.LOOKING_FOR_BALL
            self.turn_direction = random.choice([TURN_LEFT, TURN_RIGHT])
            self.delay = True
    
    def control_loop(self):
        ####  difference in yaw = item.x * -0.261  ######

        self.get_logger().info(f"State: {self.state}")

        # if (self.delay):
        #     self.delayTimer += 1
        #     if self.delayTimer > 3:
        #         self.delay = False
        #         self.delayTimer = 0
        #     return
        
        match self.state:
            case State.LOOKING_FOR_BALL:
                self.look_for_ball()
            case State.LOOKING_FOR_SPAWN:
                self.look_for_spawn()
            case State.HEADING_TO_BALL:
                self.get_logger().info(f"Colour: {self.colour_filter}")
                self.head_to_ball()
            case State.HEADING_TO_SPAWN:
                self.head_to_spawn()
            
    
    

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