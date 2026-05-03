from dataclasses import dataclass
from pathlib import Path
import ast

import numpy as np


@dataclass(frozen=True)
class MapMetadata:
    yaml_path: Path
    image_path: Path
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    width: int
    height: int
    occupied_thresh: float
    free_thresh: float
    negate: int


@dataclass(frozen=True)
class LoadedMap:
    metadata: MapMetadata
    occupancy: np.ndarray


def load_map(yaml_path):
    metadata = load_map_metadata(yaml_path)
    image = read_pgm_image(metadata.image_path)
    if metadata.negate:
        image = 255 - image

    image_float = image.astype(np.float32) / 255.0
    occupancy = np.full(image.shape, -1, dtype=np.int8)
    occupied_cutoff = 1.0 - metadata.occupied_thresh

    occupancy[image_float >= metadata.free_thresh] = 0
    occupancy[image_float <= occupied_cutoff] = 100

    # ROS OccupancyGrid data starts at the map origin, so flip the PGM rows.
    return LoadedMap(metadata=metadata, occupancy=np.flipud(occupancy))


def load_map_metadata(yaml_path):
    yaml_path = Path(yaml_path)
    data = read_simple_yaml(yaml_path)

    image_path = (yaml_path.parent / data["image"]).resolve()
    width, height = read_pgm_size(image_path)
    origin_x, origin_y, origin_yaw = data["origin"]

    return MapMetadata(
        yaml_path=yaml_path.resolve(),
        image_path=image_path,
        resolution=float(data["resolution"]),
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        origin_yaw=float(origin_yaw),
        width=width,
        height=height,
        occupied_thresh=float(data["occupied_thresh"]),
        free_thresh=float(data["free_thresh"]),
        negate=int(data.get("negate", 0)),
    )


def read_pgm_size(image_path):
    with Path(image_path).open("rb") as handle:
        magic = _next_non_comment_line(handle)
        if magic != b"P5":
            raise ValueError(f"Expected binary PGM/P5 map, got {magic!r}")

        width, height = [int(value) for value in _next_non_comment_line(handle).split()]
        return width, height


def read_pgm_image(image_path):
    with Path(image_path).open("rb") as handle:
        magic = _next_non_comment_line(handle)
        if magic != b"P5":
            raise ValueError(f"Expected binary PGM/P5 map, got {magic!r}")

        width, height = [int(value) for value in _next_non_comment_line(handle).split()]
        maxval = int(_next_non_comment_line(handle))
        if maxval > 255:
            raise ValueError(f"Unsupported PGM max value {maxval}")

        image = np.frombuffer(handle.read(width * height), dtype=np.uint8)
        if image.size != width * height:
            raise ValueError(f"PGM payload size does not match {width}x{height}")
        return image.reshape((height, width))


def read_simple_yaml(yaml_path):
    data = {}
    with Path(yaml_path).open("r", encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("#") or ":" not in stripped:
                continue

            key, value = stripped.split(":", 1)
            data[key.strip()] = parse_yaml_value(value.strip())
    return data


def parse_yaml_value(value):
    if value == "":
        return ""
    if value.startswith("["):
        return ast.literal_eval(value)
    if value in ("true", "false"):
        return value == "true"
    try:
        return int(value)
    except ValueError:
        pass
    try:
        return float(value)
    except ValueError:
        return value


def _next_non_comment_line(handle):
    while True:
        line = handle.readline()
        if not line:
            raise ValueError("Unexpected end of file while reading PGM")

        stripped = line.strip()
        if stripped and not stripped.startswith(b"#"):
            return stripped
