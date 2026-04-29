#!/usr/bin/env python3

import os
import time
import math
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

GOLD, BLUE, CYAN, GREEN = "\033[1;33m", "\033[1;34m", "\033[1;36m", "\033[1;32m"
RESET, BOLD = "\033[0m", "\033[1m"

def print_menu():
    """Prints the CLI Menu Interface."""
    os.system('clear' if os.name != 'nt' else 'cls')
    print(f"{GOLD}╔══════════════════════════════════════════╗{RESET}")
    print(f"{GOLD}║{RESET} {BOLD}          THE TOASTED TURTLE            {RESET} {GOLD}║{RESET}")
    print(f"{GOLD}╠══════════════════════════════════════════╣{RESET}")
    print(f"{GOLD}║{RESET} {CYAN}“Hi! I will be your waiter today.”  {RESET}     {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  Please select an option below:          {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}                                          {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(1){RESET} 💧 Water                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(2){RESET} 🍔 Burger                           {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(3){RESET} 🍟 Fries                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(4){RESET} 🥤 Shake                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(5){RESET} ❌ NOTHING (EXIT)                   {GOLD}║{RESET}")
    print(f"{GOLD}╚══════════════════════════════════════════╝{RESET}")

class WaiterExecutive(Node):
    def __init__(self):
        super().__init__('waiter_executive')
        self.order_pub = self.create_publisher(Int32MultiArray, 'orders', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10) # TODO: Is this the right place to publish?
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.state = "TAKING_ORDER"
        self.current_pose = None
        self.arrival_time = None
        self.locations = {
            "kitchen": {"x": 4.364, "y": 1.124},
            "user":    {"x": -0.179, "y": -3.728}
        } # TODO: Use YAML here??
        
    def odom_callback(self, msg):
        """Updates robot's current position from Odometry."""
        self.current_pose = msg.pose.pose.position

    def send_goal(self, loc_name):
        """Publishes a goal pose based on waypoint name."""
        coords = self.locations[loc_name]
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = coords["x"]
        msg.pose.position.y = coords["y"]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

    def arrived(self, loc_name, threshold=0.3): # TODO: I don't think we need this function once the controller is built
        """Calculates Euclidean distance to target to determine arrival."""
        if self.current_pose is None:
            return False
        
        target = self.locations[loc_name]
        dist = math.sqrt(
            (self.current_pose.x - target["x"])**2 + 
            (self.current_pose.y - target["y"])**2
        )
        return dist < threshold

    # Main State Machine
    def run_robot(self):
        """Main control loop for the waiter robot."""
        while rclpy.ok():
            if self.state == "TAKING_ORDER":
                print_menu()
                try:
                    choice = input(f"{GREEN}Enter menu item number: {RESET}")
                    item = int(choice)
                    
                    if item == 5:
                        print(f"\n{BOLD}{CYAN}Waiter:{RESET} \"Goodbye! Come back soon.\"\n")
                        break
                    
                    qty = int(input(f"{BOLD}{CYAN}Waiter:{RESET} \"Excellent choice! How many?\": "))
                    print(f"{BOLD}{CYAN}Waiter:{RESET} \"Great! I'll get that for you right away.\"\n")
                    
                    # Publish order
                    order_msg = Int32MultiArray(data=[item, qty])
                    self.order_pub.publish(order_msg)

                    self.send_goal("kitchen")
                    self.state = "GOING_TO_KITCHEN"

                    print(f"{BOLD}[STATUS]{RESET} Moving to Kitchen...")

                except ValueError:
                    print(f"{GOLD}Invalid input. Please enter a number.{RESET}")
                    time.sleep(1)

            elif self.state == "GOING_TO_KITCHEN":
                if self.arrived("kitchen"):
                    self.arrival_time = time.time()
                    self.state = "WAITING_FOR_FOOD"
                    print(f"{BOLD}[STATUS]{RESET} At Kitchen. Loading food...")

            elif self.state == "WAITING_FOR_FOOD":
                if time.time() - self.arrival_time >= 10.0:
                    self.send_goal("user")
                    self.state = "DELIVERING"
                    print(f"{BOLD}[STATUS]{RESET} Food loaded. Delivering to user...")

            elif self.state == "DELIVERING":
                if self.arrived("user"):
                    print(f"{BOLD}[STATUS]{RESET} Order Delivered!")
                    time.sleep(3)
                    self.state = "TAKING_ORDER"
            
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = WaiterExecutive()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_robot()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()