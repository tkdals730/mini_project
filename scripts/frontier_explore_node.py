#!/usr/bin/env python3
from collections import deque
import math

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
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
        self.current_goal = None
        self.last_goal_time = rospy.Time(0)
        self.current_robot_x = 0.0
        self.current_robot_y = 0.0
        self.last_frontier_seen_time = rospy.Time(0)
        self.exploration_complete_published = False

        # 같은 goal에 너무 오래 묶이지 않게 해서, 막힌 구역이면 빨리 다른 frontier를 보게 한다.
        self.goal_timeout = rospy.Duration(rospy.get_param("~goal_timeout_sec", 8.0))
        # 로봇 바로 주변의 frontier는 의미 없는 미세 이동이 되기 쉬워서 최소 거리를 둔다.
        self.min_goal_distance = rospy.get_param("~min_goal_distance", 0.4)
        # 한 번 실패한 지점 주변은 잠시 제외해서 같은 실패를 반복하지 않게 한다.
        self.blacklist_radius = rospy.get_param("~blacklist_radius", 0.3)
        # frontier 주변 여유 공간을 조금 확인해서 지나치게 벽에 붙은 goal은 피한다.
        self.frontier_clearance_cells = rospy.get_param("~frontier_clearance_cells", 2)
        # 너무 작은 frontier는 문틈 노이즈나 벽 가장자리일 가능성이 높아서 제외한다.
        self.min_frontier_cluster_size = rospy.get_param("~min_frontier_cluster_size", 8)
        # 가까운 후보만 고르면 한 방 안에서 맴돌기 쉬워서, 넓은 frontier를 더 높게 본다.
        self.frontier_size_weight = rospy.get_param("~frontier_size_weight", 1.0)
        self.frontier_distance_weight = rospy.get_param("~frontier_distance_weight", 0.25)
        self.max_score_distance = rospy.get_param("~max_score_distance", 4.0)
        self.exploration_complete_wait = rospy.Duration(
            rospy.get_param("~exploration_complete_wait_sec", 15.0)
        )

        self.exploration_complete_pub = rospy.Publisher(
            "/exploration_complete", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback)

        rospy.loginfo("Waiting for move_base action server for frontier exploration...")
        if not self.move_base.wait_for_server(rospy.Duration(20.0)):
            raise rospy.ROSInitException("move_base action server is not available")
        self.last_frontier_seen_time = rospy.Time.now()

    def _map_callback(self, msg):
        self.map_msg = msg

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
        for bx, by in self.blacklist:
            if math.hypot(wx - bx, wy - by) <= self.blacklist_radius:
                return True
        return False

    def _has_free_clearance(self, gx, gy, width, height, data):
        # frontier 주변에 장애물이 너무 가까우면, move_base가 접근하기 어렵다고 보고 제외한다.
        radius = self.frontier_clearance_cells
        for ny in range(max(0, gy - radius), min(height, gy + radius + 1)):
            for nx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                idx = self._to_index(nx, ny, width)
                if data[idx] > 40:
                    return False
        return True

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

        best_cell = None
        best_distance = None
        for gx, gy in cluster:
            if not self._has_free_clearance(gx, gy, width, height, data):
                continue
            wx, wy = self._grid_to_world(gx, gy, origin_x, origin_y, resolution)
            if self._is_blacklisted(wx, wy):
                continue

            distance = math.hypot(gx - center_x, gy - center_y)
            if best_distance is None or distance < best_distance:
                best_distance = distance
                best_cell = (gx, gy)

        if best_cell is None:
            return None

        return self._grid_to_world(best_cell[0], best_cell[1], origin_x, origin_y, resolution)

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

            wx, wy = goal
            distance = math.hypot(wx - robot_x, wy - robot_y)
            if distance < self.min_goal_distance:
                continue

            cluster_width = len(cluster) * resolution
            distance_score = min(distance, self.max_score_distance)
            score = (self.frontier_size_weight * cluster_width) + (
                self.frontier_distance_weight * distance_score
            )
            if best_score is None or score > best_score:
                best_score = score
                best_goal = (wx, wy)

        rospy.loginfo_throttle(
            5.0,
            "Frontier candidates: clusters=%d selected=%s",
            len(clusters),
            "yes" if best_goal is not None else "no",
        )

        return best_goal

    def _send_goal(self, goal_x, goal_y, robot_yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal_yaw = math.atan2(goal_y - self.current_robot_y, goal_x - self.current_robot_x)
        goal.target_pose.pose.orientation = yaw_to_quaternion(goal_yaw)
        self.move_base.send_goal(goal)
        self.current_goal = (goal_x, goal_y)
        self.last_goal_time = rospy.Time.now()
        rospy.loginfo("Frontier goal sent: x=%.2f y=%.2f", goal_x, goal_y)

    def _mark_frontier_seen(self):
        self.last_frontier_seen_time = rospy.Time.now()
        if self.exploration_complete_published:
            self.exploration_complete_pub.publish(Bool(data=False))
            self.exploration_complete_published = False

    def _maybe_publish_exploration_complete(self):
        if self.exploration_complete_published:
            return

        if rospy.Time.now() - self.last_frontier_seen_time < self.exploration_complete_wait:
            return

        rospy.loginfo("Exploration complete: no reachable frontier for %.1f sec.",
                      self.exploration_complete_wait.to_sec())
        self.exploration_complete_pub.publish(Bool(data=True))
        self.exploration_complete_published = True

    def _handle_goal_state(self):
        if self.current_goal is None:
            return

        state = self.move_base.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("Frontier goal reached.")
            self.current_goal = None
            return

        if state in (GoalStatus.ABORTED, GoalStatus.REJECTED):
            rospy.logwarn("Frontier goal failed, blacklisting area.")
            self.blacklist.append(self.current_goal)
            self.current_goal = None
            return

        # move_base가 너무 오래 같은 goal에 매달리면,
        # 일단 그 구역은 포기하고 다른 frontier를 고르게 한다.
        if rospy.Time.now() - self.last_goal_time > self.goal_timeout:
            rospy.logwarn("Frontier goal timed out, blacklisting area.")
            self.move_base.cancel_goal()
            self.blacklist.append(self.current_goal)
            self.current_goal = None

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

            self._handle_goal_state()

            if self.current_goal is None:
                frontier_goal = self._find_frontier_goal(robot_x, robot_y)
                if frontier_goal is not None:
                    self._mark_frontier_seen()
                    # 목표 자세는 현재 로봇 heading을 대략 유지하고,
                    # 세부 접근은 move_base가 처리하도록 둔다.
                    self._send_goal(frontier_goal[0], frontier_goal[1], robot_yaw)
                else:
                    rospy.loginfo_throttle(5.0, "No reachable frontier found right now.")
                    self._maybe_publish_exploration_complete()

            rate.sleep()


def main():
    rospy.init_node("frontier_explore_node")
    FrontierExploreNode().run()


if __name__ == "__main__":
    main()
