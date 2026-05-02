#!/usr/bin/env python3
import rclpy

from .occupancy_grid_2d import OccupancyGrid2d


def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGrid2d()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
