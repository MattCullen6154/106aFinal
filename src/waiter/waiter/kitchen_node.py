import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class KitchenReceiver(Node):
    def __init__(self):
        super().__init__('kitchen_receiver')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'orders',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        item_num = msg.data[0]
        qty = msg.data[1]
        if item_num == 5:
            return
        else:
            if item_num == 1:
                item = "Water"
            elif item_num == 2:
                item = "Burger"
            elif item_num == 3:
                item = "Fries"
            elif item_num == 4:
                item = "Shake"
            self.get_logger().info(f'ORDER RECEIVED: {qty} {item.lower()}')

def main(args=None):
    rclpy.init(args=args)
    node = KitchenReceiver()
    rclpy.spin(node)
    rclpy.shutdown()