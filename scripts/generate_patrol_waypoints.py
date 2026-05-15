#!/usr/bin/env python3
from collections import deque
import math
import os
import sys

import rospy


ROS_NODE_READY = False


def _coerce_param_value(value):
    lowered = value.lower()
    if lowered in ("true", "false"):
        return lowered == "true"
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value


def _cli_private_params():
    params = {}
    for arg in sys.argv[1:]:
        if ":=" not in arg:
            continue
        name, value = arg.split(":=", 1)
        name = name.lstrip("_")
        params[name] = _coerce_param_value(value)
    return params


def _get_param(name, default):
    if ROS_NODE_READY:
        return rospy.get_param("~" + name, default)
    return _cli_private_params().get(name, default)


def _loginfo(message, *args):
    if ROS_NODE_READY:
        rospy.loginfo(message, *args)
    else:
        print(message % args if args else message)


def _read_map_yaml(path):
    values = {}
    with open(path, "r", encoding="utf-8") as yaml_file:
        for raw_line in yaml_file:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip()

    if "image" not in values or "resolution" not in values or "origin" not in values:
        raise ValueError("map yaml must contain image, resolution, and origin")

    image = values["image"].strip("\"'")
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(path), image)

    origin_text = values["origin"].strip()
    origin_text = origin_text.strip("[]")
    origin = [float(part.strip()) for part in origin_text.split(",")]
    if len(origin) < 2:
        raise ValueError("map origin must contain at least x and y")

    return image, float(values["resolution"]), origin


def _read_pgm(path):
    with open(path, "rb") as pgm_file:
        magic = pgm_file.readline().strip()
        if magic != b"P5":
            raise ValueError("only binary P5 PGM maps are supported")

        tokens = []
        while len(tokens) < 3:
            line = pgm_file.readline()
            if not line:
                raise ValueError("unexpected end of PGM header")
            line = line.split(b"#", 1)[0].strip()
            if line:
                tokens.extend(line.split())

        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        if max_value > 255:
            raise ValueError("only 8-bit PGM maps are supported")

        pixels = pgm_file.read(width * height)
        if len(pixels) != width * height:
            raise ValueError("PGM pixel data size does not match width/height")

    return width, height, pixels


def _to_index(x, y, width):
    return y * width + x


def _world_to_grid(wx, wy, origin_x, origin_y, resolution):
    gx = int(math.floor((wx - origin_x) / resolution))
    gy = int(math.floor((wy - origin_y) / resolution))
    return gx, gy


def _grid_to_world(gx, gy, origin_x, origin_y, resolution):
    return origin_x + (gx + 0.5) * resolution, origin_y + (gy + 0.5) * resolution


def _is_safe_cell(gx, gy, width, height, free_cells, clearance_cells):
    if gx < clearance_cells or gy < clearance_cells:
        return False
    if gx >= width - clearance_cells or gy >= height - clearance_cells:
        return False

    for ny in range(gy - clearance_cells, gy + clearance_cells + 1):
        for nx in range(gx - clearance_cells, gx + clearance_cells + 1):
            if not free_cells[_to_index(nx, ny, width)]:
                return False
    return True


def _find_nearest_safe_seed(start_gx, start_gy, width, height, safe_cells):
    if 0 <= start_gx < width and 0 <= start_gy < height:
        if safe_cells[_to_index(start_gx, start_gy, width)]:
            return start_gx, start_gy

    visited = set()
    queue = deque([(start_gx, start_gy)])
    visited.add((start_gx, start_gy))
    while queue:
        gx, gy = queue.popleft()
        for nx, ny in ((gx + 1, gy), (gx - 1, gy), (gx, gy + 1), (gx, gy - 1)):
            if (nx, ny) in visited:
                continue
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            if safe_cells[_to_index(nx, ny, width)]:
                return nx, ny
            visited.add((nx, ny))
            queue.append((nx, ny))
    return None


def _reachable_safe_cells(seed, width, height, safe_cells):
    reachable = [False] * (width * height)
    queue = deque([seed])
    reachable[_to_index(seed[0], seed[1], width)] = True

    while queue:
        gx, gy = queue.popleft()
        for nx, ny in ((gx + 1, gy), (gx - 1, gy), (gx, gy + 1), (gx, gy - 1)):
            if nx < 0 or ny < 0 or nx >= width or ny >= height:
                continue
            idx = _to_index(nx, ny, width)
            if reachable[idx] or not safe_cells[idx]:
                continue
            reachable[idx] = True
            queue.append((nx, ny))

    return reachable


def _sample_candidates(width, height, reachable, stride_cells):
    candidates = []
    for gy in range(0, height, stride_cells):
        for gx in range(0, width, stride_cells):
            best = None
            best_distance = None
            for ny in range(gy, min(height, gy + stride_cells)):
                for nx in range(gx, min(width, gx + stride_cells)):
                    if not reachable[_to_index(nx, ny, width)]:
                        continue
                    center_x = gx + (stride_cells * 0.5)
                    center_y = gy + (stride_cells * 0.5)
                    distance = math.hypot(nx - center_x, ny - center_y)
                    if best_distance is None or distance < best_distance:
                        best_distance = distance
                        best = (nx, ny)
            if best is not None:
                candidates.append(best)
    return candidates


def _select_spread_points(candidates, start, max_count, min_spacing_cells):
    if not candidates or max_count <= 0:
        return []

    selected = []
    remaining = [
        point for point in candidates
        if math.hypot(point[0] - start[0], point[1] - start[1]) >= min_spacing_cells
    ]

    while remaining and len(selected) < max_count:
        if not selected:
            best = max(remaining, key=lambda point: math.hypot(point[0] - start[0], point[1] - start[1]))
        else:
            best = max(
                remaining,
                key=lambda point: min(
                    math.hypot(point[0] - chosen[0], point[1] - chosen[1])
                    for chosen in selected
                ),
            )

        selected.append(best)
        remaining = [
            point for point in remaining
            if math.hypot(point[0] - best[0], point[1] - best[1]) >= min_spacing_cells
        ]

    return selected


def _order_nearest_neighbor(points, start):
    ordered = []
    remaining = list(points)
    current = start
    while remaining:
        next_point = min(
            remaining,
            key=lambda point: math.hypot(point[0] - current[0], point[1] - current[1]),
        )
        ordered.append(next_point)
        remaining.remove(next_point)
        current = next_point
    return ordered


def _points_to_waypoints(points, origin_x, origin_y, resolution, wait_sec):
    waypoints = []
    for index, point in enumerate(points):
        wx, wy = _grid_to_world(point[0], point[1], origin_x, origin_y, resolution)
        if index + 1 < len(points):
            nx, ny = _grid_to_world(points[index + 1][0], points[index + 1][1], origin_x, origin_y, resolution)
            yaw = math.atan2(ny - wy, nx - wx)
        elif waypoints:
            previous = waypoints[-1]
            yaw = math.atan2(wy - previous["y"], wx - previous["x"])
        else:
            yaw = 0.0
        waypoints.append({"x": wx, "y": wy, "yaw": yaw, "wait_sec": wait_sec})
    return waypoints


def _write_waypoints(path, waypoints):
    output_dir = os.path.dirname(path)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)

    with open(path, "w", encoding="utf-8") as output_file:
        output_file.write("# Generated by generate_patrol_waypoints.py\n")
        output_file.write("waypoints:\n")
        for waypoint in waypoints:
            output_file.write(
                "  - {x: %.3f, y: %.3f, yaw: %.3f, wait_sec: %.1f}\n"
                % (
                    waypoint["x"],
                    waypoint["y"],
                    waypoint["yaw"],
                    waypoint["wait_sec"],
                )
            )


def generate_waypoints():
    package_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    map_file = _get_param("map_file", os.path.join(package_dir, "maps", "patrol_map.yaml"))
    output_file = _get_param(
        "output_file", os.path.join(package_dir, "maps", "generated_patrol_waypoints.yaml")
    )
    max_waypoints = int(_get_param("max_waypoints", 6))
    sample_spacing = float(_get_param("sample_spacing", 1.8))
    min_waypoint_spacing = float(_get_param("min_waypoint_spacing", 1.6))
    obstacle_clearance = float(_get_param("obstacle_clearance", 0.35))
    wait_sec = float(_get_param("wait_sec", 3.0))
    start_x = float(_get_param("start_x", 0.0))
    start_y = float(_get_param("start_y", 0.85))
    free_pixel_threshold = int(_get_param("free_pixel_threshold", 250))

    map_file = os.path.expanduser(map_file)
    output_file = os.path.expanduser(output_file)
    image_file, resolution, origin = _read_map_yaml(map_file)
    width, height, pixels = _read_pgm(image_file)
    origin_x, origin_y = origin[0], origin[1]

    free_cells = []
    for gy in range(height):
        image_y = height - 1 - gy
        for gx in range(width):
            free_cells.append(pixels[_to_index(gx, image_y, width)] >= free_pixel_threshold)
    clearance_cells = max(1, int(math.ceil(obstacle_clearance / resolution)))
    safe_cells = [
        _is_safe_cell(gx, gy, width, height, free_cells, clearance_cells)
        for gy in range(height)
        for gx in range(width)
    ]

    start_gx, start_gy = _world_to_grid(start_x, start_y, origin_x, origin_y, resolution)
    seed = _find_nearest_safe_seed(start_gx, start_gy, width, height, safe_cells)
    if seed is None:
        raise RuntimeError("No safe reachable seed cell found near start/home position")

    reachable = _reachable_safe_cells(seed, width, height, safe_cells)
    stride_cells = max(1, int(round(sample_spacing / resolution)))
    min_spacing_cells = max(1, int(round(min_waypoint_spacing / resolution)))
    candidates = _sample_candidates(width, height, reachable, stride_cells)
    selected = _select_spread_points(candidates, seed, max_waypoints, min_spacing_cells)
    ordered = _order_nearest_neighbor(selected, seed)
    waypoints = _points_to_waypoints(ordered, origin_x, origin_y, resolution, wait_sec)

    if not waypoints:
        raise RuntimeError("No patrol waypoints generated from map")

    _write_waypoints(output_file, waypoints)
    _loginfo(
        "Generated %d patrol waypoints from %s -> %s",
        len(waypoints),
        map_file,
        output_file,
    )


def main():
    global ROS_NODE_READY
    try:
        rospy.init_node("generate_patrol_waypoints")
        ROS_NODE_READY = True
    except Exception as exc:
        print("Running without ROS master: %s" % exc)
    generate_waypoints()


if __name__ == "__main__":
    main()
