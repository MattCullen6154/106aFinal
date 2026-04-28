import argparse
from pathlib import Path

from .planner import astar, build_planner, path_to_world, simplify_path, snap_to_nearest_free, world_to_coarse_cell


def main():
    parser = argparse.ArgumentParser(description="Plan a coarse A* path between named map waypoints.")
    parser.add_argument("start", help="Start waypoint name")
    parser.add_argument("goal", help="Goal waypoint name")
    parser.add_argument(
        "--map-yaml",
        default=str(Path(__file__).resolve().parent.parent / "maps" / "restaurant_map.yaml"),
        help="Path to the saved ROS map YAML",
    )
    parser.add_argument(
        "--waypoints",
        default=str(Path(__file__).resolve().parent.parent / "config" / "waypoints.yaml"),
        help="Path to the waypoint YAML",
    )
    parser.add_argument("--block-size", type=int, default=4, help="Number of fine map cells per coarse planning cell")
    parser.add_argument(
        "--inflation-radius",
        type=float,
        default=0.15,
        help="Obstacle inflation radius in meters",
    )
    args = parser.parse_args()

    grid_map, coarse_map, waypoints = build_planner(
        args.map_yaml,
        args.waypoints,
        block_size=args.block_size,
        inflation_radius_m=args.inflation_radius,
    )

    if args.start not in waypoints or args.goal not in waypoints:
        available = ", ".join(sorted(waypoints))
        raise SystemExit(f"Unknown waypoint. Available choices: {available}")

    start_wp = waypoints[args.start]
    goal_wp = waypoints[args.goal]

    start_cell = world_to_coarse_cell(start_wp.x, start_wp.y, grid_map, coarse_map.block_size)
    goal_cell = world_to_coarse_cell(goal_wp.x, goal_wp.y, grid_map, coarse_map.block_size)
    start_cell = snap_to_nearest_free(start_cell, coarse_map)
    goal_cell = snap_to_nearest_free(goal_cell, coarse_map)

    path = astar(coarse_map, start_cell, goal_cell)
    if not path:
        raise SystemExit("No path found between the requested waypoints")

    path = simplify_path(path)
    world_path = path_to_world(path, grid_map, coarse_map.block_size)

    print(f"Start waypoint: {args.start} -> coarse cell {start_cell}")
    print(f"Goal waypoint:  {args.goal} -> coarse cell {goal_cell}")
    print("World-frame path:")
    for index, (x_coord, y_coord) in enumerate(world_path):
        print(f"  {index:02d}: x={x_coord:.3f}, y={y_coord:.3f}")
