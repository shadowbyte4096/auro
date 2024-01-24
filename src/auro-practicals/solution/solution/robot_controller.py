import sys
import math
from enum import IntEnum

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from visualization_msgs.msg import Marker

from assessment_interfaces.msg import ItemHolder, ItemHolders
from solution_interfaces.msg import NearestItemTypes, Item
from geometry_msgs.msg import PoseStamped, Point
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

class GoalState(IntEnum):
    GO_HOME = 0
    CONTINUE = 1
    GO_TO_NEAREST = 2
    GO_TO_RED = 3
    GO_TO_GREEN = 4
    GO_TO_BLUE = 5

class Colour(IntEnum):
    NONE = 0
    RED = 1
    GREEN = 2
    BLUE = 3

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

        self.pos_x = self.initial_x
        self.pos_y = self.initial_y
        self.yaw = self.initial_yaw

        self.current_nav_goal = PoseStamped()

        self.navigator = BasicNavigator()

        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = self.initial_x
        self.initial_pose.pose.position.y = self.initial_y
        self.initial_pose.pose.orientation.z = self.initial_yaw
        self.navigator.setInitialPose(self.initial_pose)

        self.nearest_blue = Item()
        self.nearest_green = Item()
        self.nearest_item = Item()

        self.holding = Colour.NONE

        self.goal_state = GoalState.GO_HOME
        self.continue_timeout = self.get_clock().now()
        self.goal_set_time = self.get_clock().now()
        self.last_highest_colour_seen = Colour.NONE
        self.last_colour_held = Colour.NONE

        self.iterations_since_goal = 0

        self.navigator.waitUntilNav2Active()

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

        self.item_publisher = self.create_publisher(Marker, 'goal', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def nearest_items_callback(self, msg):
        self.nearest_item = msg.nearest
        self.nearest_blue = msg.blue
        self.nearest_green = msg.green

        self.add_marker(msg.red, Colour.RED) #visulise on rviz
        self.add_marker(msg.green, Colour.GREEN)
        self.add_marker(msg.blue, Colour.BLUE)
    
    def item_holder_callback(self, msg):
        holders = msg.data
        self.holding = Colour.NONE
        for holder in holders:
            if holder.robot_id != self.robot_name:
                continue
            elif not holder.holding_item:
                continue
            old = self.holding
            self.holding = self.colour_id_to_enum(holder.item_colour)
            if (old != self.holding):
                self.find_new_target()
    
    def odom_callback(self, msg):
        (roll, pitch, yaw) = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                    msg.pose.pose.orientation.y,
                                                    msg.pose.pose.orientation.z,
                                                    msg.pose.pose.orientation.w])
        
        self.pos_x = msg.pose.pose.position.x + self.initial_x
        self.pos_y = msg.pose.pose.position.y + self.initial_y
        
        self.yaw = -math.degrees(yaw)
        if self.yaw < 0:
            self.yaw += 360 #normalise to 0-360 to act as a bearing
    
    def add_marker(self, ball, colour):
        if not ball.visible:
            return
        r = 0.0
        g = 0.0
        b = 0.0
        match colour:
            case Colour.RED:
                r = 1.0
            case Colour.GREEN:
                g = 1.0
            case Colour.BLUE:
                b = 1.0
        position = self.find_ball_position(ball).pose.position
        marker = self.point_to_marker(position, r, g, b)
        self.item_publisher.publish(marker)
    
    def point_to_marker(self, point, r = 1.0, g = 1.0, b = 0.0):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.robot_name
        marker.id = self.get_clock().now().nanoseconds // 10000000 #kind of random id ig
        marker.type = 2
        marker.action = 0
        marker.pose.position = point
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.lifetime = Duration(seconds=1).to_msg()
        return marker




    def control_loop(self):
        highest_seen = self.highest_colour_seen()

        if highest_seen == self.last_highest_colour_seen:
            if self.holding == self.last_colour_held: #only set new goal when state changes
                time_since_goal_set = self.get_clock().now() - self.goal_set_time
                if (time_since_goal_set < 1): #reset goal every 1 second when state hasnt changed
                    return

        self.goal_set_time = self.get_clock().now()
        self.last_highest_colour_seen = highest_seen
        self.last_colour_held = self.holding

        goal = self.find_new_target() #what the robot should be doing
        self.enact_goal(goal) #how the robot should do it
    
    def find_new_target(self):
        highest_seen = self.last_highest_colour_seen
        if (self.holding == Colour.BLUE):
            return GoalState.GO_HOME
        
        elif (highest_seen <= self.holding):
            return GoalState.CONTINUE

        elif (self.holding == Colour.NONE):
            return GoalState.GO_TO_NEAREST
        
        elif (highest_seen == Colour.BLUE):
            return GoalState.GO_TO_BLUE
        
        elif (highest_seen == Colour.GREEN):
            return GoalState.GO_TO_GREEN
        
        else: # should never happen as all cases should have been covered so log
            self.get_logger().info(f"UNCOVERED STATE")
            return GoalState.GO_HOME

    def enact_goal(self, goal):
        pose = PoseStamped()
        match goal:
            case GoalState.GO_HOME:
                pose = self.initial_pose
            case GoalState.CONTINUE:
                time_on_continue = self.get_clock().now() - self.continue_timeout
                if (time_on_continue > 3): #continue on path unless its been over 3 seconds
                    #self.get_logger().info(f"GOING HOME")
                    pose = self.initial_pose
                    self.navigator.goToPose(pose)
                #self.get_logger().info(f"FINISHED CONTINUING")
                return
            case GoalState.GO_TO_NEAREST:
                ball = self.colour_enum_to_ball(Colour.NONE)
                pose = self.find_ball_position(ball)
            case GoalState.GO_TO_RED:
                self.get_logger().info(f"WANT TO FIND RED??") # should never happen under this implementation so log
                ball = self.colour_enum_to_ball(Colour.RED)
                pose = self.find_ball_position(ball)
            case GoalState.GO_TO_GREEN:
                ball = self.colour_enum_to_ball(Colour.GREEN)
                pose = self.find_ball_position(ball)
            case GoalState.GO_TO_BLUE:
                ball = self.colour_enum_to_ball(Colour.BLUE)
                pose = self.find_ball_position(ball)
        self.continue_timeout = self.get_clock().now()
        self.navigator.goToPose(pose)
    
    def find_ball_position(self, ball):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        if ball.visible:
            estimated_distance = (69.0 * (float(ball.diameter) ** -0.89)) + 0.1 #aims a little further then nessessary
            ball_x_mult = (0.003 * estimated_distance) + 0.085 #since camera is mounted near front mult changes with distance
            angle_diff = -ball_x_mult * ball.x
            estimated_angle = self.yaw + angle_diff
            
            x = estimated_distance * math.cos(math.radians(estimated_angle)) #x is positive towards 0 degrees?!
            y = estimated_distance * -math.sin(math.radians(estimated_angle)) #y is positive towards 90 degrees?!

            goal_pose.pose.position.x = x + self.pos_x
            goal_pose.pose.position.y = y + self.pos_y
            goal_pose.pose.orientation.w = estimated_angle
        else:
            self.get_logger().info(f"FINDING INVISIBLE BALL??") # should never happen so log
            goal_pose.pose.position.x = self.pos_x + 1
            goal_pose.pose.position.y = self.pos_y + 1
            goal_pose.pose.orientation.w = self.yaw
        return goal_pose




    def colour_id_to_enum(self, colour):
        match colour:
            case "BLUE":
                return Colour.BLUE
            case "GREEN":
                return Colour.GREEN
            case "RED":
                return Colour.RED
        return Colour.NONE
    
    def highest_colour_seen(self):
        if self.nearest_blue.visible:
            return Colour.BLUE
        if self.nearest_green.visible:
            return Colour.GREEN
        if self.nearest_item.visible:
            return Colour.RED
        return Colour.NONE

    def colour_enum_to_ball(self, colour):
        match colour:
            case Colour.BLUE:
                return self.nearest_blue
            case Colour.GREEN:
                return self.nearest_green
            case _:
                return self.nearest_item
            



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