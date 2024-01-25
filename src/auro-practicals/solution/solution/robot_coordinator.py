import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import ItemList
from solution_interfaces.msg import HomesAndTargets, RobotTarget

class RobotCoordinator(Node):

    def __init__(self):
        super().__init__('item_sensor_filters')

        self.all_homes = []
        self.targets = []
        self.target_history = []

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

    def control_loop(self):

        targets = [t.target for t in self.targets]
        available_homes = [h for h in self.all_homes if h.pose.position not in targets]

        homes_and_targets = HomesAndTargets()
        homes_and_targets.homes = available_homes
        homes_and_targets.targets = self.targets

        self.home_and_targets_publisher.publish(homes_and_targets)
        

    def destroy_node(self):
        self.get_logger().info(f"Stopping")
        super().destroy_node()


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
        