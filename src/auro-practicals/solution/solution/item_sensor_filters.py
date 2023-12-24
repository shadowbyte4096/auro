import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import Item, ItemList

class item_sensor_filters(Node):

    def __init__(self):
        super().__init__('item_sensor_filters')

        self.item_sensor_subsciber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10
        )

        self.nearest_items_publisher = self.create_publisher(ItemList, 'items/nearest', 10)
        self.nearest_blue_publisher = self.create_publisher(ItemList, 'items/blue', 10)

    def item_callback(self, items):
        return
        