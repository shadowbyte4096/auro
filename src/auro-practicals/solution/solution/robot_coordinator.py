import sys
import math

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import ItemList
from solution_interfaces.msg import HomesAndTargets, RobotPoint

class RobotCoordinator(Node):

    def __init__(self):
        super().__init__('item_sensor_filters')

        self.all_homes = []
        self.targets = []
        self.positions = []

        self.robot1_subscriber = self.create_subscription(
            HomesAndTargets,
            'robot1/coordination',
            self.robot_callback,
            10
        )

        self.robot2_subscriber = self.create_subscription(
            HomesAndTargets,
            'robot2/coordination',
            self.robot_callback,
            10
        )

        self.robot3_subscriber = self.create_subscription(
            HomesAndTargets,
            'robot3/coordination',
            self.robot_callback,
            10
        )

        self.home_and_targets_publisher = self.create_publisher(HomesAndTargets, '/coordination', 10)

        self.timer_period = 0.1 # 100 milliseconds = 10 Hz
        self.timer = self.create_timer(self.timer_period, self.control_loop)
    
    def robot_callback(self, msg):
        if len(msg.homes) != 1:
            self.get_logger().info(f"HOME NOT SET PROPERLY") #shouldnt happen so log
            return
        home = msg.homes[0]
        if home not in self.all_homes:
            self.all_homes.append(home)
        
        if len(msg.targets) != 1:
            self.get_logger().info(f"TARGET NOT SET PROPERLY") #shouldnt happen so log
            return
        target_msg = msg.targets[0]
        robot = target_msg.robot_id
        self.targets = [target_msg if robot == t.robot_id else t for t in self.targets]
        if target_msg not in self.targets:
            self.targets.append(target_msg)
        
        if len(msg.positions) != 1:
            self.get_logger().info(f"POSIION NOT SET PROPERLY") #shouldnt happen so log
            return
        position_msg = msg.positions[0]
        robot = position_msg.robot_id
        self.positions = [position_msg if robot == t.robot_id else t for t in self.positions]
        if position_msg not in self.positions:
            self.positions.append(position_msg)

    def control_loop(self):

        targets = [t.point for t in self.targets]
        available_homes = [h for h in self.all_homes if h.pose.position not in targets]
        close_robots = self.find_close_robots()
        for robot in close_robots:

        ###########################################################################
            #make robots go home
        ###########################################################################
            

        homes_and_targets = HomesAndTargets()
        homes_and_targets.homes = available_homes
        homes_and_targets.targets = self.targets
        homes_and_targets.positions = self.positions

        self.home_and_targets_publisher.publish(homes_and_targets)
        
    def find_close_robots(self):
        close_robots = []
        for robot in self.positions:
            for other in self.positions:
                if robot == other:
                    continue
                elif is_near(robot.point, other.point):
                    close_robots.append(robot)
        return close_robots 
    
    

    def destroy_node(self):
        self.get_logger().info(f"Stopping")
        super().destroy_node()

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
        