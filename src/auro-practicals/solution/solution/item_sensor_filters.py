import sys

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.executors import ExternalShutdownException

from assessment_interfaces.msg import ItemList
from solution_interfaces.msg import Item, NearestItemTypes

class ItemSensorFilter(Node):

    def __init__(self):
        super().__init__('item_sensor_filters')

        self.get_logger().info(f"STARTING ITEM FILTER")

        self.item_sensor_subsciber = self.create_subscription(
            ItemList,
            'items',
            self.item_callback,
            10
        )

        self.nearest_iteams_publisher = self.create_publisher(NearestItemTypes, 'items/near', 10)
    
    def item_callback(self, items):

        def replace_nearest(nearest):
            if nearest == None:
                return item
            elif item.diameter > nearest.diameter:
                return item
            else:
                return nearest

        def translate_item(to_translate):
            item = Item()
            if to_translate == None:
                item.visible = False
                item.diameter = 0
                item.x = 0
            else:
                item.visible = True
                item.diameter = to_translate.diameter
                item.x = to_translate.x
            return item
        
        nearest_item = None
        nearest_blue = None
        nearest_green = None
        nearest_red = None
        for item in items.data:
            nearest_item = replace_nearest(nearest_item)
            match (item.colour):
                case "BLUE":
                    nearest_blue = replace_nearest(nearest_blue)
                    continue
                case "GREEN":
                    nearest_green = replace_nearest(nearest_green)
                    continue
                case "RED":
                    nearest_red = replace_nearest(nearest_red)
        
        nearest_item = translate_item(nearest_item)
        nearest_blue = translate_item(nearest_blue)
        nearest_green = translate_item(nearest_green)
        nearest_red = translate_item(nearest_red)

        nearest_items = NearestItemTypes()
        nearest_items.nearest = nearest_item
        nearest_items.blue = nearest_blue
        nearest_items.green = nearest_green
        nearest_items.red = nearest_red
        self.nearest_iteams_publisher.publish(nearest_items)

    def destroy_node(self):
        self.get_logger().info(f"Stopping")
        super().destroy_node()


def main(args=None):

    rclpy.init(args = args, signal_handler_options = SignalHandlerOptions.NO)

    node = ItemSensorFilter()

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
        