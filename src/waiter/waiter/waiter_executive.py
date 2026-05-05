#!/usr/bin/env python3

import math
import os
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
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
    print(f"{GOLD}║{RESET}  Build an order below:                   {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}                                          {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  Food: burger, fries, shake, or none     {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  Water: yes or no                        {GOLD}║{RESET}")
    print(f"{GOLD}║{RESET}  Type exit to quit                       {GOLD}║{RESET}")
    print(f"{GOLD}╚══════════════════════════════════════════╝{RESET}")


class WaiterExecutive(Node):
    def __init__(self):
        super().__init__("waiter_executive")
        self.order_pub = self.create_publisher(Int32MultiArray, "orders", 10)
        self.nav_goal_pub = self.create_publisher(String, "/nav_goal_waypoint", 10)
        self.create_subscription(String, "/nav_status", self.nav_status_callback, 10)

        self.nav_status = "idle"
        self.current_target = None
        self.current_location = "kitchen"
        self.state = "TAKING_ORDER"
        self.route_queue = []
        self.pickup_stops = set()
        self.waypoints = self.load_waypoints()

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
        food_text = input(f"{GREEN}Welcome to the Toasted Turtle, what will you be having to eat today?: {RESET}").strip().lower()
        if food_text in ("exit", "quit", "5"):
            print(f"\n{BOLD}{CYAN}Waiter:{RESET} \"Goodbye! Come back soon.\"\n")
            return False

        water_text = input(f"{GREEN}Water? [y/n]: {RESET}").strip().lower()
        if water_text in ("exit", "quit"):
            print(f"\n{BOLD}{CYAN}Waiter:{RESET} \"Goodbye! Come back soon.\"\n")
            return False

        food_item = self.parse_food_item(food_text)
        wants_water = water_text in ("y", "yes", "water", "1")
        if food_item is None and not wants_water:
            print(f"{GOLD}No items selected. Please order food, water, or exit.{RESET}")
            time.sleep(1.0)
            return True

        if food_item is not None:
            qty = self.prompt_quantity(food_item)
            if qty is None:
                return True
            self.order_pub.publish(Int32MultiArray(data=[food_item, qty]))

        pickup_stops = []
        if food_item is not None:
            pickup_stops.append("kitchen")
        if wants_water:
            pickup_stops.append("water_station")

        pickup_stops = self.order_pickup_stops(pickup_stops)
        self.pickup_stops = set(pickup_stops)
        self.route_queue = pickup_stops + ["table", "kitchen"]

        print(f"{BOLD}{CYAN}Waiter:{RESET} \"Great! I'll get that for you right away.\"")
        print(f"{BOLD}[ROUTE]{RESET} {' -> '.join(self.route_queue)}")
        self.send_next_goal()
        return True

    def prompt_quantity(self, item):
        try:
            return int(input(f"{BOLD}{CYAN}Waiter:{RESET} \"How many {self.item_name(item)}s would you like?\": "))
        except ValueError:
            print(f"{GOLD}Invalid quantity. Please enter a number.{RESET}")
            time.sleep(1.0)
            return None

    @staticmethod
    def parse_food_item(food_text):
        food_items = {
            "none": None,
            "no": None,
            "": None,
            "burger": 2,
            "2": 2,
            "fries": 3,
            "3": 3,
            "shake": 4,
            "4": 4,
        }
        return food_items.get(food_text)

    @staticmethod
    def item_name(item):
        return {2: "burger", 3: "fries", 4: "shake"}.get(item, "item")

    def order_pickup_stops(self, pickup_stops):
        if len(pickup_stops) < 2:
            return pickup_stops

        candidates = [
            pickup_stops,
            list(reversed(pickup_stops)),
        ]
        return min(candidates, key=self.route_length_to_table)

    def route_length_to_table(self, pickup_stops):
        route = [self.current_location] + pickup_stops + ["table"]
        return sum(
            self.waypoint_distance(route[index], route[index + 1])
            for index in range(len(route) - 1)
        )

    def waypoint_distance(self, start, goal):
        if start not in self.waypoints or goal not in self.waypoints:
            return 0.0
        start_x, start_y = self.waypoints[start]
        goal_x, goal_y = self.waypoints[goal]
        return math.hypot(goal_x - start_x, goal_y - start_y)

    def send_next_goal(self):
        if not self.route_queue:
            self.state = "TAKING_ORDER"
            return

        next_goal = self.route_queue.pop(0)
        self.send_nav_goal(next_goal)
        self.state = "GOING_TO_WAYPOINT"

    def run_robot(self):
        while rclpy.ok():
            if self.state == "TAKING_ORDER":
                if not self.take_order():
                    break

            elif self.state == "GOING_TO_WAYPOINT":
                if self.navigation_arrived():
                    self.current_location = self.current_target
                    self.handle_arrival()

            elif self.state == "WAITING":
                if time.time() - self.wait_start_time >= self.wait_duration:
                    self.send_next_goal()

            time.sleep(0.1)

    def handle_arrival(self):
        if self.current_location in self.pickup_stops:
            print(f"{BOLD}[STATUS]{RESET} Arrived at {self.current_location}. Loading...")
            self.pickup_stops.remove(self.current_location)
            self.wait_at_stop(5.0)
        elif self.current_location == "table":
            print(f"{BOLD}[STATUS]{RESET} Order delivered. Returning to kitchen...")
            self.wait_at_stop(3.0)
        elif self.current_location == "kitchen" and not self.route_queue:
            print(f"{BOLD}[STATUS]{RESET} Returned to kitchen. Ready for next order.")
            time.sleep(1.0)
            self.state = "TAKING_ORDER"
        else:
            self.send_next_goal()

    def wait_at_stop(self, duration):
        self.wait_start_time = time.time()
        self.wait_duration = duration
        self.state = "WAITING"

    @staticmethod
    def load_waypoints():
        try:
            package_share = Path(get_package_share_directory("restaurant_mapping"))
            waypoint_path = package_share / "config" / "waypoints.yaml"
        except Exception:
            return {}

        waypoints = {}
        current_name = None
        current_entry = {}
        with waypoint_path.open("r", encoding="utf-8") as handle:
            for raw_line in handle:
                stripped = raw_line.strip()
                if not stripped or stripped.startswith("#") or stripped == "waypoints:":
                    continue

                indent = len(raw_line) - len(raw_line.lstrip(" "))
                if indent == 2 and stripped.endswith(":"):
                    if current_name is not None:
                        waypoints[current_name] = (
                            float(current_entry["x"]),
                            float(current_entry["y"]),
                        )
                    current_name = stripped[:-1]
                    current_entry = {}
                elif indent == 4 and ":" in stripped:
                    key, value = stripped.split(":", 1)
                    current_entry[key.strip()] = value.strip()

        if current_name is not None:
            waypoints[current_name] = (
                float(current_entry["x"]),
                float(current_entry["y"]),
            )
        return waypoints


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
