from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float = 0.0


def load_waypoints(yaml_path):
    waypoints = {}
    current_name = None
    current_entry = {}

    with Path(yaml_path).open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            if not raw_line.strip() or raw_line.lstrip().startswith("#"):
                continue

            indent = len(raw_line) - len(raw_line.lstrip(" "))
            stripped = raw_line.strip()
            if stripped == "waypoints:":
                continue

            if indent == 2 and stripped.endswith(":"):
                if current_name is not None:
                    waypoints[current_name] = make_waypoint(current_name, current_entry)
                current_name = stripped[:-1]
                current_entry = {}
                continue

            if indent == 4 and ":" in stripped:
                key, value = stripped.split(":", 1)
                current_entry[key.strip()] = value.strip()

    if current_name is not None:
        waypoints[current_name] = make_waypoint(current_name, current_entry)
    return waypoints


def make_waypoint(name, entry):
    return Waypoint(
        name=name,
        x=float(entry["x"]),
        y=float(entry["y"]),
        yaw=float(entry.get("yaw", 0.0)),
    )
