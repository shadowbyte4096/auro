import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import ItemList
from solution_interfaces.msg import CoordinatorInfo, RobotInfo, RobotPoint, RobotPoseStamped

class RobotCoordinator(Node):

    def __init__(self):
        super().__init__('robot_coordinator')

        self.all_homes = []
        self.targets = []
        self.positions = []

        self.robot1_position_subscriber = self.create_subscription(
            RobotPoint,
            'robot1/position',
            self.robot_position_callback,
            10
        )

        self.robot2_position_subscriber = self.create_subscription(
            RobotPoint,
            'robot2/position',
            self.robot_position_callback,
            10
        )

        self.robot3_position_subscriber = self.create_subscription(
            RobotPoint,
            'robot3/position',
            self.robot_position_callback,
            10
        )

        self.robot1_subscriber = self.create_subscription(
            RobotInfo,
            'robot1/coordination',
            self.robot_callback,
            10
        )

        self.robot2_subscriber = self.create_subscription(
            RobotInfo,
            'robot2/coordination',
            self.robot_callback,
            10
        )

        self.robot3_subscriber = self.create_subscription(
            RobotInfo,
            'robot3/coordination',
            self.robot_callback,
            10
        )

        self.home_and_targets_publisher = self.create_publisher(CoordinatorInfo, 'coordination', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        for x in range(10):
            self.get_logger().info(f"FINSIHED STARTING ROBOT COORDINATOR")
    
    def robot_position_callback(self, msg): #updated more frequently
        robot = msg.robot_id
        self.positions = [msg if robot == t.robot_id else t for t in self.positions]
        if msg not in self.positions:
            self.positions.append(msg)

    def robot_callback(self, msg):
        home = msg.initial_home
        if home not in self.all_homes:
            self.all_homes.append(home)

        target_msg = msg.current_target
        robot = target_msg.robot_id
        self.targets = [target_msg if robot == t.robot_id else t for t in self.targets]
        if target_msg not in self.targets:
            self.targets.append(target_msg)

        if robot == "robot2":   
            t = target_msg.point
            self.get_logger().info(f"RECEIVING: ({t.x},{t.y})")

        position_msg = msg.current_position
        robot = position_msg.robot_id
        self.positions = [position_msg if robot == t.robot_id else t for t in self.positions]
        if position_msg not in self.positions:
            self.positions.append(position_msg)

    def control_loop(self):

        coordination_msg = CoordinatorInfo()
        coordination_msg.targets = self.targets

        targets = [t.point for t in self.targets]

        tstr = [f"({t.x},{t.y})" for t in targets]
        #self.get_logger().info("TARGETS"+", ".join(tstr))

        homes = [h.pose.position for h in self.all_homes]
        hstr = [f"({h.x},{h.y})" for h in homes]
        #self.get_logger().info("HOMES"+", ".join(hstr))

        #available_homes = [h for h in self.all_homes if h.pose.position not in targets]
        def close(h, targets):
            for t in targets:
                if abs(h.y - t.y) < 0.1:
                    if abs(h.x - t.x) < 0.1:
                        #self.get_logger().info(f"AAAAHHHHHHH: ({h.x},{h.y}) : ({t.x},{t.y})")
                        return True
            return False
        available_homes = [h for h in self.all_homes if not close(h.pose.position, targets)]
        #self.get_logger().info(f"homes: {len(available_homes)}")

        close_robots = self.find_close_robots()
        robot_homes, available_homes = self.find_suitable_home(available_homes, close_robots)
        coordination_msg.avoidance_positions = robot_homes
        coordination_msg.available_homes = available_homes

        self.home_and_targets_publisher.publish(coordination_msg)
        
    def find_close_robots(self):
        close_robots = []
        for robot in self.positions:
            for other in self.positions:
                if robot == other:
                    continue
                elif is_near(robot.point, other.point) and robot not in close_robots:
                    close_robots.append(robot)
        return close_robots 
    
    def find_suitable_home(self, available_homes, close_robots):
        robot_homes = []
        for robot in close_robots:
            robot_pose = RobotPoseStamped()
            robot_pose.robot_id = robot.robot_id

            others = [other for other in close_robots if other != robot]
            robot_angles = [calculate_bearing(robot.point, other.point) for other in others]
            avg_robot_angle = sum(robot_angles) / len(robot_angles)

            greatest_diff = -1.0
            for home in available_homes:
                home_angle = calculate_bearing(robot.point, home.pose.position)
                diff = abs(home_angle - avg_robot_angle)
                if diff > greatest_diff:
                    greatest_diff = diff
                    robot_pose.pose = home
            
            robot_homes.append(robot_pose)

            available_homes = [home for home in available_homes if home != robot_pose.pose]
        return robot_homes, available_homes
                
            
    def destroy_node(self):
        self.get_logger().info(f"Stopping")
        super().destroy_node()

def calculate_bearing(robot, point):
    dx = point.x - robot.x
    dy = point.y - robot.y

    bearing = math.atan2(dy, dx)
    bearing = math.degrees(bearing)

    if bearing < 0:
        bearing += 360
    
    return bearing

def is_near(point_a, point_b):
    diffX = point_a.x - point_b.x
    diffY = point_a.y - point_b.y
    distance = math.sqrt(diffX ** 2 + diffY ** 2)
    if (distance < 1):
        return True
    return False

def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = RobotCoordinator()

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
        