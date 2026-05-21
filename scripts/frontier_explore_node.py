#!/usr/bin/env python3
from collections import deque
import math

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler


def yaw_to_quaternion(yaw):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class FrontierExploreNode:
    def __init__(self):
        self.map_msg = None
        self.tf_listener = tf.TransformListener()
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.blacklist = []
        self.frontier_blacklist = []
        self.blocked_frontier_sectors = []
        self.current_goal = None
        self.current_frontier = None
        self.goal_failure_counts = {}
        self.last_goal_time = rospy.Time(0)
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.last_frontier_seen_time = rospy.Time(0)
        self.start_time = rospy.Time.now()
        self.last_progress_time = rospy.Time.now()
        self.best_free_cell_count = 0
        self.suppressed_frontier_events = 0
        self.exploration_complete_published = False
        self.patrol_paused = False

        # 같은 goal에 너무 오래 묶이지 않게 해서, 막힌 구역이면 빨리 다른 frontier를 보게 한다.
        self.goal_timeout = rospy.Duration(rospy.get_param("~goal_timeout_sec", 8.0))
        # 로봇 바로 주변의 frontier는 의미 없는 미세 이동이 되기 쉬워서 최소 거리를 둔다.
        self.min_goal_distance = rospy.get_param("~min_goal_distance", 0.4)
        # 0보다 크면 너무 먼 frontier goal은 제외해서 move_base 실패 가능성을 줄인다.
        self.max_goal_distance = rospy.get_param("~max_goal_distance", 0.0)
        # 한 번 실패한 지점 주변은 잠시 제외해서 같은 실패를 반복하지 않게 한다.
        self.blacklist_radius = rospy.get_param("~blacklist_radius", 0.3)
        self.frontier_blacklist_radius = rospy.get_param("~frontier_blacklist_radius", 0.5)
        self.blocked_sector_enabled = rospy.get_param("~blocked_sector_enabled", True)
        self.blocked_sector_width = rospy.get_param("~blocked_sector_width", 0.9)
        self.blocked_sector_depth = rospy.get_param("~blocked_sector_depth", 3.0)
        self.blocked_sector_backtrack = rospy.get_param("~blocked_sector_backtrack", 0.3)
        self.blacklist_ttl = rospy.Duration(rospy.get_param("~blacklist_ttl_sec", 60.0))
        self.blacklist_max_size = rospy.get_param("~blacklist_max_size", 30)
        # frontier 주변 여유 공간을 조금 확인해서 지나치게 벽에 붙은 goal은 피한다.
        self.frontier_clearance_cells = rospy.get_param("~frontier_clearance_cells", 2)
        self.viewpoint_clearance_cells = rospy.get_param("~viewpoint_clearance_cells", 5)
        self.viewpoint_require_known_space = rospy.get_param("~viewpoint_require_known_space", False)
        self.occupied_threshold = rospy.get_param("~occupied_threshold", 40)
        # 너무 작은 frontier는 문틈 노이즈나 벽 가장자리일 가능성이 높아서 제외한다.
        self.min_frontier_cluster_size = rospy.get_param("~min_frontier_cluster_size", 8)
        # 넓고 정보량이 큰 frontier를 선호하되, 먼 goal과 장애물 근접 후보는 감점한다.
        self.frontier_size_weight = rospy.get_param("~frontier_size_weight", 1.0)
        self.information_radius_cells = rospy.get_param("~information_radius_cells", 4)
        self.obstacle_penalty_radius_cells = rospy.get_param("~obstacle_penalty_radius_cells", 3)
        self.information_gain_weight = rospy.get_param("~information_gain_weight", 1.0)
        self.distance_weight = rospy.get_param("~distance_weight", 0.25)
        self.obstacle_penalty_weight = rospy.get_param("~obstacle_penalty_weight", 0.25)
        self.max_score_distance = rospy.get_param("~max_score_distance", 4.0)
        self.prefer_nearest_frontier = rospy.get_param("~prefer_nearest_frontier", True)
        self.nearest_frontier_score_tolerance = rospy.get_param(
            "~nearest_frontier_score_tolerance", 3.0
        )
        self.local_frontier_radius = rospy.get_param("~local_frontier_radius", 1.2)
        self.local_frontier_bonus = rospy.get_param("~local_frontier_bonus", 20.0)
        self.viewpoint_min_distance_cells = rospy.get_param("~viewpoint_min_distance_cells", 4)
        self.viewpoint_max_distance_cells = rospy.get_param("~viewpoint_max_distance_cells", 9)
        self.max_goal_failures_before_blacklist = rospy.get_param(
            "~max_goal_failures_before_blacklist", 1
        )
        self.suppress_reached_frontier = rospy.get_param("~suppress_reached_frontier", True)
        self.completion_spin_enabled = rospy.get_param("~completion_spin_enabled", True)
        self.completion_spin_attempts = rospy.get_param("~completion_spin_attempts", 2)
        self.completion_spin_angular_speed = rospy.get_param(
            "~completion_spin_angular_speed", 0.45
        )
        self.completion_spin_count = 0
        self.minimum_mapping_time = rospy.Duration(
            rospy.get_param("~minimum_mapping_time_sec", 120.0)
        )
        self.progress_timeout = rospy.Duration(
            rospy.get_param("~progress_timeout_sec", 60.0)
        )
        self.free_cell_growth_threshold = rospy.get_param("~free_cell_growth_threshold", 25)
        self.min_suppressed_frontiers_for_completion = rospy.get_param(
            "~min_suppressed_frontiers_for_completion", 3
        )
        self.force_completion_after_repeated_failures = rospy.get_param(
            "~force_completion_after_repeated_failures", True
        )
        self.exploration_complete_wait = rospy.Duration(
            rospy.get_param("~exploration_complete_wait_sec", 15.0)
        )

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.exploration_complete_pub = rospy.Publisher(
            "/exploration_complete", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback)
        rospy.Subscriber("/patrol_pause", Bool, self._patrol_pause_callback, queue_size=1)

        rospy.loginfo("Waiting for move_base action server for frontier exploration...")
        if not self.move_base.wait_for_server(rospy.Duration(20.0)):
            raise rospy.ROSInitException("move_base action server is not available")
        self.last_frontier_seen_time = rospy.Time.now()

    def _map_callback(self, msg):
        self.map_msg = msg
        free_cells = sum(1 for value in msg.data if value == 0)
        if free_cells >= self.best_free_cell_count + self.free_cell_growth_threshold:
            self.best_free_cell_count = free_cells
            self.last_progress_time = rospy.Time.now()
            self.completion_spin_count = 0
            rospy.loginfo(
                "Exploration map progress: free_cells=%d threshold=%d",
                free_cells,
                self.free_cell_growth_threshold,
            )

    def _patrol_pause_callback(self, msg):
        paused = bool(msg.data)
        if paused == self.patrol_paused:
            return

        self.patrol_paused = paused
        if paused:
            rospy.logwarn("Frontier exploration paused by fire response.")
            self.move_base.cancel_goal()
            self.current_goal = None
            self.current_frontier = None
            self.cmd_vel_pub.publish(Twist())
        else:
            rospy.loginfo("Frontier exploration pause cleared.")

    def _get_robot_pose(self):
        self.tf_listener.waitForTransform("map", "base_footprint", rospy.Time(0), rospy.Duration(1.0))
        translation, rotation = self.tf_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        yaw = tf.transformations.euler_from_quaternion(rotation)[2]
        return translation[0], translation[1], yaw

    def _to_index(self, x, y, width):
        return y * width + x

    def _grid_to_world(self, gx, gy, origin_x, origin_y, resolution):
        world_x = origin_x + (gx + 0.5) * resolution
        world_y = origin_y + (gy + 0.5) * resolution
        return world_x, world_y

    def _is_blacklisted(self, wx, wy):
        self._prune_blacklist()
        for bx, by, _stamp in self.blacklist:
            if math.hypot(wx - bx, wy - by) <= self.blacklist_radius:
                return True
        return False

    def _is_frontier_blacklisted(self, wx, wy):
        self._prune_blacklist()
        for bx, by, _stamp in self.frontier_blacklist:
            if math.hypot(wx - bx, wy - by) <= self.frontier_blacklist_radius:
                return True

        for sector in self.blocked_frontier_sectors:
            sx, sy, dx, dy, start_dist, end_dist, half_width, _stamp = sector
            rel_x = wx - sx
            rel_y = wy - sy
            forward = (rel_x * dx) + (rel_y * dy)
            if forward < start_dist or forward > end_dist:
                continue
            sideways = abs((rel_x * -dy) + (rel_y * dx))
            if sideways <= half_width:
                return True
        return False

    def _prune_blacklist(self):
        if not self.blacklist and not self.frontier_blacklist and not self.blocked_frontier_sectors:
            return

        now = rospy.Time.now()
        if self.blacklist_ttl.to_sec() > 0.0:
            self.blacklist = [
                entry for entry in self.blacklist
                if now - entry[2] <= self.blacklist_ttl
            ]
            self.frontier_blacklist = [
                entry for entry in self.frontier_blacklist
                if now - entry[2] <= self.blacklist_ttl
            ]
            self.blocked_frontier_sectors = [
                entry for entry in self.blocked_frontier_sectors
                if now - entry[7] <= self.blacklist_ttl
            ]

        if self.blacklist_max_size > 0 and len(self.blacklist) > self.blacklist_max_size:
            self.blacklist = self.blacklist[-self.blacklist_max_size:]
        if self.blacklist_max_size > 0 and len(self.frontier_blacklist) > self.blacklist_max_size:
            self.frontier_blacklist = self.frontier_blacklist[-self.blacklist_max_size:]
        if (
            self.blacklist_max_size > 0
            and len(self.blocked_frontier_sectors) > self.blacklist_max_size
        ):
            self.blocked_frontier_sectors = self.blocked_frontier_sectors[-self.blacklist_max_size:]

    def _add_to_blacklist(self, goal):
        if goal is None:
            return

        self.blacklist.append((goal[0], goal[1], rospy.Time.now()))
        self._prune_blacklist()
        rospy.loginfo(
            "Frontier blacklist updated: size=%d ttl=%.1fs max_size=%d",
            len(self.blacklist),
            self.blacklist_ttl.to_sec(),
            self.blacklist_max_size,
        )

    def _add_to_frontier_blacklist(self, frontier, reason):
        if frontier is None:
            return

        self.frontier_blacklist.append((frontier[0], frontier[1], rospy.Time.now()))
        self.suppressed_frontier_events += 1
        self._prune_blacklist()
        rospy.loginfo(
            "Frontier area suppressed after %s: size=%d radius=%.2f ttl=%.1fs",
            reason,
            len(self.frontier_blacklist),
            self.frontier_blacklist_radius,
            self.blacklist_ttl.to_sec(),
        )

    def _add_blocked_sector(self, frontier, reason):
        if not self.blocked_sector_enabled or frontier is None:
            return

        dx = frontier[0] - self.current_robot_x
        dy = frontier[1] - self.current_robot_y
        distance = math.hypot(dx, dy)
        if distance <= 0.05:
            return

        dx /= distance
        dy /= distance
        start_dist = max(0.0, distance - self.blocked_sector_backtrack)
        end_dist = distance + self.blocked_sector_depth
        half_width = max(0.05, self.blocked_sector_width * 0.5)
        self.blocked_frontier_sectors.append(
            (
                self.current_robot_x,
                self.current_robot_y,
                dx,
                dy,
                start_dist,
                end_dist,
                half_width,
                rospy.Time.now(),
            )
        )
        self._prune_blacklist()
        rospy.loginfo(
            "Blocked frontier sector after %s: sectors=%d width=%.2fm depth=%.2fm",
            reason,
            len(self.blocked_frontier_sectors),
            self.blocked_sector_width,
            self.blocked_sector_depth,
        )

    def _failure_key(self):
        target = self.current_frontier if self.current_frontier is not None else self.current_goal
        if target is None:
            return None
        return (round(target[0], 1), round(target[1], 1))

    def _record_goal_failure(self):
        key = self._failure_key()
        if key is None:
            return 0

        failures = self.goal_failure_counts.get(key, 0) + 1
        self.goal_failure_counts[key] = failures
        return failures

    def _has_free_clearance(self, gx, gy, width, height, data):
        # frontier 주변에 장애물이 너무 가까우면, move_base가 접근하기 어렵다고 보고 제외한다.
        radius = self.frontier_clearance_cells
        for ny in range(max(0, gy - radius), min(height, gy + radius + 1)):
            for nx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                idx = self._to_index(nx, ny, width)
                if data[idx] > self.occupied_threshold:
                    return False
        return True

    def _is_safe_free_cell(self, gx, gy, width, height, data):
        if gx <= 0 or gy <= 0 or gx >= width - 1 or gy >= height - 1:
            return False

        idx = self._to_index(gx, gy, width)
        if data[idx] != 0:
            return False

        for ny in range(
            max(0, gy - self.viewpoint_clearance_cells),
            min(height, gy + self.viewpoint_clearance_cells + 1),
        ):
            for nx in range(
                max(0, gx - self.viewpoint_clearance_cells),
                min(width, gx + self.viewpoint_clearance_cells + 1),
            ):
                nidx = self._to_index(nx, ny, width)
                if data[nidx] > self.occupied_threshold:
                    return False
                if self.viewpoint_require_known_space and data[nidx] < 0:
                    return False

        return True

    def _unknown_direction(self, gx, gy, width, height, data):
        dx = 0.0
        dy = 0.0
        count = 0
        for ny in range(max(1, gy - 1), min(height - 1, gy + 2)):
            for nx in range(max(1, gx - 1), min(width - 1, gx + 2)):
                if nx == gx and ny == gy:
                    continue
                idx = self._to_index(nx, ny, width)
                if data[idx] == -1:
                    dx += nx - gx
                    dy += ny - gy
                    count += 1

        if count == 0:
            return None

        length = math.hypot(dx, dy)
        if length <= 0.0:
            return None
        return dx / length, dy / length

    def _find_viewpoint_for_frontier(self, gx, gy, width, height, data):
        unknown_dir = self._unknown_direction(gx, gy, width, height, data)
        if unknown_dir is None:
            return None

        ux, uy = unknown_dir
        best_cell = None
        best_distance = None

        for distance_cells in range(
            self.viewpoint_min_distance_cells,
            self.viewpoint_max_distance_cells + 1,
        ):
            projected_x = int(round(gx - (ux * distance_cells)))
            projected_y = int(round(gy - (uy * distance_cells)))

            for search_radius in range(0, 5):
                for cy in range(projected_y - search_radius, projected_y + search_radius + 1):
                    for cx in range(projected_x - search_radius, projected_x + search_radius + 1):
                        if not self._is_safe_free_cell(cx, cy, width, height, data):
                            continue

                        distance = math.hypot(cx - projected_x, cy - projected_y)
                        if best_distance is None or distance < best_distance:
                            best_distance = distance
                            best_cell = (cx, cy)

                if best_cell is not None:
                    return best_cell[0], best_cell[1], ux, uy

        return None

    def _count_cells_around(self, gx, gy, width, height, data, radius, predicate):
        count = 0
        for ny in range(max(0, gy - radius), min(height, gy + radius + 1)):
            for nx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                idx = self._to_index(nx, ny, width)
                if predicate(data[idx]):
                    count += 1
        return count

    def _is_frontier_cell(self, gx, gy, width, data):
        idx = self._to_index(gx, gy, width)
        if data[idx] != 0:
            return False

        for ny in range(gy - 1, gy + 2):
            for nx in range(gx - 1, gx + 2):
                if nx == gx and ny == gy:
                    continue
                nidx = self._to_index(nx, ny, width)
                if data[nidx] == -1:
                    return True
        return False

    def _build_frontier_clusters(self, width, height, data):
        frontier_cells = set()
        for gy in range(1, height - 1):
            for gx in range(1, width - 1):
                if self._is_frontier_cell(gx, gy, width, data):
                    frontier_cells.add((gx, gy))

        clusters = []
        visited = set()
        for start in frontier_cells:
            if start in visited:
                continue

            queue = deque([start])
            visited.add(start)
            cluster = []

            while queue:
                gx, gy = queue.popleft()
                cluster.append((gx, gy))

                for ny in range(gy - 1, gy + 2):
                    for nx in range(gx - 1, gx + 2):
                        neighbor = (nx, ny)
                        if neighbor in visited or neighbor not in frontier_cells:
                            continue
                        visited.add(neighbor)
                        queue.append(neighbor)

            if len(cluster) >= self.min_frontier_cluster_size:
                clusters.append(cluster)

        return clusters

    def _pick_cluster_goal(self, cluster, width, height, data, origin_x, origin_y, resolution):
        center_x = sum(gx for gx, _ in cluster) / float(len(cluster))
        center_y = sum(gy for _, gy in cluster) / float(len(cluster))

        best_goal = None
        best_distance = None
        for gx, gy in cluster:
            viewpoint = self._find_viewpoint_for_frontier(gx, gy, width, height, data)
            if viewpoint is None:
                continue
            goal_gx, goal_gy, unknown_dx, unknown_dy = viewpoint

            frontier_wx, frontier_wy = self._grid_to_world(gx, gy, origin_x, origin_y, resolution)
            if self._is_frontier_blacklisted(frontier_wx, frontier_wy):
                continue

            wx, wy = self._grid_to_world(goal_gx, goal_gy, origin_x, origin_y, resolution)
            if self._is_blacklisted(wx, wy):
                continue

            distance = math.hypot(gx - center_x, gy - center_y)
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_goal = (
                    goal_gx,
                    goal_gy,
                    gx,
                    gy,
                    unknown_dx,
                    unknown_dy,
                    frontier_wx,
                    frontier_wy,
                )

        if best_goal is None:
            return None

        (
            goal_gx,
            goal_gy,
            frontier_gx,
            frontier_gy,
            unknown_dx,
            unknown_dy,
            frontier_wx,
            frontier_wy,
        ) = best_goal
        wx, wy = self._grid_to_world(goal_gx, goal_gy, origin_x, origin_y, resolution)
        goal_yaw = math.atan2(unknown_dy, unknown_dx)
        return wx, wy, frontier_gx, frontier_gy, goal_yaw, frontier_wx, frontier_wy

    def _score_frontier(self, gx, gy, width, height, data, distance, cluster_size, resolution):
        information_gain = self._count_cells_around(
            gx,
            gy,
            width,
            height,
            data,
            self.information_radius_cells,
            lambda value: value == -1,
        )
        obstacle_penalty = self._count_cells_around(
            gx,
            gy,
            width,
            height,
            data,
            self.obstacle_penalty_radius_cells,
            lambda value: value > self.occupied_threshold,
        )

        cluster_width = cluster_size * resolution
        distance_score = min(distance, self.max_score_distance)
        local_bonus = self.local_frontier_bonus if distance <= self.local_frontier_radius else 0.0
        normalized_information = min(information_gain, 20) / 20.0
        normalized_obstacle_penalty = min(obstacle_penalty, 20) / 20.0
        score = (
            (self.frontier_size_weight * cluster_width)
            + (self.information_gain_weight * normalized_information)
            + local_bonus
            - (self.distance_weight * distance_score)
            - (self.obstacle_penalty_weight * normalized_obstacle_penalty)
        )
        return score, information_gain, obstacle_penalty

    def _find_frontier_goal(self, robot_x, robot_y):
        if self.map_msg is None:
            return None

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        resolution = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        data = list(self.map_msg.data)

        best_goal = None
        best_score = None
        clusters = self._build_frontier_clusters(width, height, data)

        for cluster in clusters:
            goal = self._pick_cluster_goal(cluster, width, height, data, origin_x, origin_y, resolution)
            if goal is None:
                continue

            wx, wy, gx, gy, goal_yaw, frontier_wx, frontier_wy = goal
            distance = math.hypot(wx - robot_x, wy - robot_y)
            if distance < self.min_goal_distance:
                continue
            if self.max_goal_distance > 0.0 and distance > self.max_goal_distance:
                continue

            score, information_gain, obstacle_penalty = self._score_frontier(
                gx,
                gy,
                width,
                height,
                data,
                distance,
                len(cluster),
                resolution,
            )
            should_select = best_score is None or score > best_score
            if self.prefer_nearest_frontier and best_goal is not None:
                best_distance = best_goal[3]
                score_gap = best_score - score
                should_select = (
                    distance < best_distance
                    and score_gap <= self.nearest_frontier_score_tolerance
                ) or score > best_score

            if should_select:
                best_score = score
                best_goal = (
                    wx,
                    wy,
                    score,
                    distance,
                    information_gain,
                    obstacle_penalty,
                    len(cluster),
                    goal_yaw,
                    frontier_wx,
                    frontier_wy,
                )

        rospy.loginfo_throttle(
            5.0,
            "Frontier candidates: clusters=%d selected=%s",
            len(clusters),
            "yes" if best_goal is not None else "no",
        )

        return best_goal

    def _send_goal(self, goal_x, goal_y, robot_yaw, goal_info=None):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal_yaw = math.atan2(goal_y - self.current_robot_y, goal_x - self.current_robot_x)
        if goal_info is not None and "goal_yaw" in goal_info:
            goal_yaw = goal_info["goal_yaw"]
        goal.target_pose.pose.orientation = yaw_to_quaternion(goal_yaw)
        self.move_base.send_goal(goal)
        self.current_goal = (goal_x, goal_y)
        self.current_frontier = None
        if goal_info is not None and "frontier_x" in goal_info and "frontier_y" in goal_info:
            self.current_frontier = (goal_info["frontier_x"], goal_info["frontier_y"])
        self.last_goal_time = rospy.Time.now()
        if goal_info is None:
            rospy.loginfo("Frontier goal sent: x=%.2f y=%.2f", goal_x, goal_y)
        else:
            rospy.loginfo(
                "Frontier goal sent: x=%.2f y=%.2f score=%.2f distance=%.2f information=%d obstacle_penalty=%d cluster_size=%d",
                goal_x,
                goal_y,
                goal_info["score"],
                goal_info["distance"],
                goal_info["information_gain"],
                goal_info["obstacle_penalty"],
                goal_info["cluster_size"],
            )

    def _mark_frontier_seen(self):
        self.last_frontier_seen_time = rospy.Time.now()
        self.completion_spin_count = 0
        if self.exploration_complete_published:
            self.exploration_complete_pub.publish(Bool(data=False))
            self.exploration_complete_published = False

    def _spin_in_place_for_scan(self):
        if not self.completion_spin_enabled:
            return False

        angular_speed = abs(self.completion_spin_angular_speed)
        if angular_speed <= 0.0:
            return False

        rospy.loginfo(
            "No reachable frontier. Rotating in place to rescan before completing exploration."
        )
        duration = (2.0 * math.pi) / angular_speed
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(10)
        cmd = Twist()
        cmd.angular.z = angular_speed

        while (
            not rospy.is_shutdown()
            and not self.patrol_paused
            and rospy.Time.now() < end_time
        ):
            self.cmd_vel_pub.publish(cmd)
            rate.sleep()

        self.cmd_vel_pub.publish(Twist())
        self.completion_spin_count += 1
        self.last_frontier_seen_time = rospy.Time.now()
        return True

    def _completion_conditions_met(self, require_frontier_wait=True):
        now = rospy.Time.now()
        if now - self.start_time < self.minimum_mapping_time:
            rospy.loginfo_throttle(
                10.0,
                "Exploration completion blocked: minimum mapping time %.1fs not reached.",
                self.minimum_mapping_time.to_sec(),
            )
            return False

        if (
            require_frontier_wait
            and now - self.last_frontier_seen_time < self.exploration_complete_wait
        ):
            return False

        no_progress = now - self.last_progress_time >= self.progress_timeout
        enough_suppressed = (
            self.suppressed_frontier_events >= self.min_suppressed_frontiers_for_completion
        )

        if no_progress and enough_suppressed:
            return True

        rospy.loginfo_throttle(
            10.0,
            "Exploration completion blocked: no_progress=%s suppressed=%d/%d.",
            "yes" if no_progress else "no",
            self.suppressed_frontier_events,
            self.min_suppressed_frontiers_for_completion,
        )
        return False

    def _publish_exploration_complete(self, reason):
        if self.exploration_complete_published:
            return

        rospy.loginfo("Exploration complete: %s", reason)
        self.move_base.cancel_goal()
        self.exploration_complete_pub.publish(Bool(data=True))
        self.exploration_complete_published = True

    def _maybe_force_completion_after_repeated_failures(self):
        if (
            not self.force_completion_after_repeated_failures
            or self.exploration_complete_published
        ):
            return

        if self._completion_conditions_met(require_frontier_wait=False):
            self._publish_exploration_complete(
                "map progress stopped and repeated frontier failures were suppressed"
            )

    def _maybe_publish_exploration_complete(self):
        if self.exploration_complete_published:
            return

        if self.completion_spin_count < self.completion_spin_attempts:
            if self._spin_in_place_for_scan():
                return

        if not self._completion_conditions_met(require_frontier_wait=True):
            self.last_frontier_seen_time = rospy.Time.now()
            return

        self._publish_exploration_complete(
            "no reachable frontier for %.1f sec" % self.exploration_complete_wait.to_sec()
        )

    def _handle_goal_state(self):
        if self.current_goal is None:
            return

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Frontier goal reached.")
            self.goal_failure_counts.pop(self._failure_key(), None)
            if self.suppress_reached_frontier:
                self._add_to_frontier_blacklist(self.current_frontier, "successful observation")
            self.current_goal = None
            self.current_frontier = None
            return

        if state in (GoalStatus.ABORTED, GoalStatus.REJECTED):
            failures = self._record_goal_failure()
            if failures >= self.max_goal_failures_before_blacklist:
                rospy.logwarn("Frontier area failed %d times, blacklisting it.", failures)
                self._add_to_frontier_blacklist(self.current_frontier, "failed navigation")
                self._add_blocked_sector(self.current_frontier, "failed navigation")
                self._add_to_blacklist(self.current_goal)
                self._maybe_force_completion_after_repeated_failures()
            else:
                rospy.logwarn(
                    "Frontier goal failed with state %d. Retrying alternatives before blacklist.",
                    state,
                )
            self.current_goal = None
            self.current_frontier = None
            return

        # move_base가 너무 오래 같은 goal에 매달리면,
        # 일단 그 구역은 포기하고 다른 frontier를 고르게 한다.
        if rospy.Time.now() - self.last_goal_time > self.goal_timeout:
            failures = self._record_goal_failure()
            rospy.logwarn("Frontier goal timed out after %.1f sec.", self.goal_timeout.to_sec())
            self.move_base.cancel_goal()
            if failures >= self.max_goal_failures_before_blacklist:
                rospy.logwarn("Frontier area timed out %d times, blacklisting it.", failures)
                self._add_to_frontier_blacklist(self.current_frontier, "navigation timeout")
                self._add_blocked_sector(self.current_frontier, "navigation timeout")
                self._add_to_blacklist(self.current_goal)
                self._maybe_force_completion_after_repeated_failures()
            self.current_goal = None
            self.current_frontier = None

    def run(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            if self.map_msg is None:
                rate.sleep()
                continue

            try:
                robot_x, robot_y, robot_yaw = self._get_robot_pose()
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                rate.sleep()
                continue
            self.current_robot_x = robot_x
            self.current_robot_y = robot_y

            if self.patrol_paused:
                if self.current_goal is not None:
                    self.move_base.cancel_goal()
                    self.current_goal = None
                    self.current_frontier = None
                self.cmd_vel_pub.publish(Twist())
                rate.sleep()
                continue

            self._handle_goal_state()

            if self.exploration_complete_published:
                rate.sleep()
                continue

            if self.current_goal is None:
                frontier_goal = self._find_frontier_goal(robot_x, robot_y)
                if frontier_goal is not None:
                    self._mark_frontier_seen()
                    goal_info = {
                        "score": frontier_goal[2],
                        "distance": frontier_goal[3],
                        "information_gain": frontier_goal[4],
                        "obstacle_penalty": frontier_goal[5],
                        "cluster_size": frontier_goal[6],
                        "goal_yaw": frontier_goal[7],
                        "frontier_x": frontier_goal[8],
                        "frontier_y": frontier_goal[9],
                    }
                    self._send_goal(frontier_goal[0], frontier_goal[1], robot_yaw, goal_info)
                else:
                    rospy.loginfo_throttle(5.0, "No reachable frontier found right now.")
                    self._maybe_publish_exploration_complete()

            rate.sleep()


def main():
    rospy.init_node("frontier_explore_node")
    FrontierExploreNode().run()


if __name__ == "__main__":
    main()
