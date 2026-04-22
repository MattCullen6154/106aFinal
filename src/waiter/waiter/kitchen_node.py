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
        item = msg.data[0]
        qty = msg.data[1]
        self.get_logger().info(f'ORDER RECEIVED: Item #{item}, Quantity: {qty}')

def main(args=None):
    rclpy.init(args=args)
    node = KitchenReceiver()
    rclpy.spin(node)
    rclpy.shutdown()