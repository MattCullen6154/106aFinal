#!/usr/bin/env python3

import os
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String


GOLD = "\033[1;33m"
BLUE = "\033[1;34m"
CYAN = "\033[1;36m"
GREEN = "\033[1;32m"
RESET = "\033[0m"
BOLD = "\033[1m"


def print_menu():
    os.system("clear" if os.name != "nt" else "cls")
    print(f"{GOLD}╔══════════════════════════════════════════╗{RESET}")
    print(f"{GOLD}║{RESET} {BOLD}          THE TOASTED TURTLE            {RESET} {GOLD}║{RESET}")
    print(f"{GOLD}╠══════════════════════════════════════════╣{RESET}")
    print(f"{GOLD}║{RESET} {CYAN}“Hi! I will be your waiter today.”  {RESET}     {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  Please select an option below:          {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}                                          {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(1){RESET} Water                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(2){RESET} Burger                           {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(3){RESET} Fries                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(4){RESET} Shake                            {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  {BLUE}(5){RESET} Nothing / exit                   {GOLD}║{RESET}")
    print(f"{GOLD}╚══════════════════════════════════════════╝{RESET}")


class WaiterExecutive(Node):
    def __init__(self):
        super().__init__("waiter_executive")
        self.order_pub = self.create_publisher(Int32MultiArray, "orders", 10)
        self.nav_goal_pub = self.create_publisher(String, "/nav_goal_waypoint", 10)
        self.create_subscription(String, "/nav_status", self.nav_status_callback, 10)

        self.nav_status = "idle"
        self.current_target = None
        self.state = "TAKING_ORDER"
        self.pending_pickup = None

    def nav_status_callback(self, msg):
        self.nav_status = msg.data

    def send_nav_goal(self, waypoint_name):
        self.current_target = waypoint_name
        self.nav_status = "requested"
        self.nav_goal_pub.publish(String(data=waypoint_name))
        print(f"{BOLD}[NAV]{RESET} Sent goal: {waypoint_name}")

    def navigation_arrived(self):
        return self.nav_status == "arrived"

    def take_order(self):
        print_menu()
        try:
            item = int(input(f"{GREEN}Enter menu item number: {RESET}"))
            if item == 5:
                print(f"\n{BOLD}{CYAN}Waiter:{RESET} \"Goodbye! Come back soon.\"\n")
                return False
            if item not in (1, 2, 3, 4):
                print(f"{GOLD}Invalid item. Please enter 1-5.{RESET}")
                time.sleep(1.0)
                return True

            qty = int(input(f"{BOLD}{CYAN}Waiter:{RESET} \"Excellent choice! How many?\": "))
            print(f"{BOLD}{CYAN}Waiter:{RESET} \"Great! I'll get that for you right away.\"")

            self.order_pub.publish(Int32MultiArray(data=[item, qty]))
            self.pending_pickup = "water_station" if item == 1 else "kitchen"
            self.send_nav_goal(self.pending_pickup)
            self.state = "GOING_TO_PICKUP"
            return True
        except ValueError:
            print(f"{GOLD}Invalid input. Please enter a number.{RESET}")
            time.sleep(1.0)
            return True

    def run_robot(self):
        while rclpy.ok():
            if self.state == "TAKING_ORDER":
                if not self.take_order():
                    break

            elif self.state == "GOING_TO_PICKUP":
                if self.navigation_arrived():
                    print(f"{BOLD}[STATUS]{RESET} Arrived at {self.pending_pickup}. Loading...")
                    self.state = "WAITING_AT_PICKUP"
                    self.pickup_start_time = time.time()

            elif self.state == "WAITING_AT_PICKUP":
                if time.time() - self.pickup_start_time >= 5.0:
                    self.send_nav_goal("table")
                    self.state = "GOING_TO_TABLE"
                    print(f"{BOLD}[STATUS]{RESET} Delivering to table...")

            elif self.state == "GOING_TO_TABLE":
                if self.navigation_arrived():
                    print(f"{BOLD}[STATUS]{RESET} Order delivered.")
                    time.sleep(3.0)
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


if __name__ == "__main__":
    main()
