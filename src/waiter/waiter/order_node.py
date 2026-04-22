import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import threading

GOLD = "\033[1;33m"
BLUE = "\033[1;34m"
CYAN = "\033[1;36m"
GREEN = "\033[1;32m"
RESET = "\033[0m"
BOLD = "\033[1m"


def print_menu():
    print(f"{GOLD}╔══════════════════════════════════════════╗{RESET}")
    print(f"{GOLD}║{RESET} {BOLD}         THE ROS 2 BISTRO               {RESET} {GOLD}║{RESET}")
    print(f"{GOLD}╠══════════════════════════════════════════╣{RESET}")
    print(f"{GOLD}║{RESET} {CYAN}“Hi! I will be your waiter today.”  {RESET}     {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  Please select an option below:          {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}                                          {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(1){RESET} 💧 Water                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(2){RESET} 🍔 Burger                           {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(3){RESET} 🍟 Fries                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(4){RESET} 🥤 Shake                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(5){RESET} ❌ NOTHING                          {GOLD}║{RESET}")
    print(f"{GOLD}╚══════════════════════════════════════════╝{RESET}")

class Order(Node):
    def __init__(self):
        super().__init__('order_node')
        self.publisher_ = self.create_publisher(Int32MultiArray, 'orders', 10)

    def publish_text(self, item, qty):
        msg = Int32MultiArray()
        msg.data = [item, qty]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Order()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        os.system('cls' if os.name == 'nt' else 'clear')
        print_menu()
        item = int(input(f"{GREEN}Enter the number for the menu item you want: {RESET}"))
        if item == 5:
            print(f"\n{BOLD}{CYAN}Waiter:{RESET} \"Thank you for visiting! I hope you enjoyed your meal. Goodbye.\"\n")
            qty = 0
        else:
            qty = int(input(f"\n{BOLD}{CYAN}Waiter:{RESET} \"Excellent choice! How many\": "))
            print(f"\n{BOLD}{CYAN}Waiter:{RESET} \"Great! I'll get that for you right away.\"\n")
        node.publish_text(item, qty)
        time.sleep(3)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
