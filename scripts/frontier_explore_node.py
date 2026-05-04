#!/usr/bin/env python3
import math

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
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

        # 같은 goal에 너무 오래 묶이지 않게 해서, 막힌 구역이면 빨리 다른 frontier를 보게 한다.
        self.goal_timeout = rospy.Duration(rospy.get_param("~goal_timeout_sec", 8.0))
        # 로봇 바로 주변의 frontier는 의미 없는 미세 이동이 되기 쉬워서 최소 거리를 둔다.
        self.min_goal_distance = rospy.get_param("~min_goal_distance", 0.4)
        # 한 번 실패한 지점 주변은 잠시 제외해서 같은 실패를 반복하지 않게 한다.
        self.blacklist_radius = rospy.get_param("~blacklist_radius", 0.3)
        self.blacklist_ttl = rospy.Duration(rospy.get_param("~blacklist_ttl_sec", 60.0))
        self.blacklist_max_size = rospy.get_param("~blacklist_max_size", 30)
        # frontier 주변 여유 공간을 조금 확인해서 지나치게 벽에 붙은 goal은 피한다.
        self.frontier_clearance_cells = rospy.get_param("~frontier_clearance_cells", 2)
        self.occupied_threshold = rospy.get_param("~occupied_threshold", 40)
        self.information_radius_cells = rospy.get_param("~information_radius_cells", 4)
        self.obstacle_penalty_radius_cells = rospy.get_param("~obstacle_penalty_radius_cells", 3)
        self.information_gain_weight = rospy.get_param("~information_gain_weight", 1.0)
        self.distance_weight = rospy.get_param("~distance_weight", 1.0)
        self.obstacle_penalty_weight = rospy.get_param("~obstacle_penalty_weight", 0.25)

        rospy.Subscriber("/map", OccupancyGrid, self._map_callback)

        rospy.loginfo("Waiting for move_base action server for frontier exploration...")
        if not self.move_base.wait_for_server(rospy.Duration(20.0)):
            raise rospy.ROSInitException("move_base action server is not available")

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
        self._prune_blacklist()
        for bx, by, _stamp in self.blacklist:
            if math.hypot(wx - bx, wy - by) <= self.blacklist_radius:
                return True
        return False

    def _prune_blacklist(self):
        if not self.blacklist:
            return

        now = rospy.Time.now()
        if self.blacklist_ttl.to_sec() > 0.0:
            self.blacklist = [
                entry for entry in self.blacklist
                if now - entry[2] <= self.blacklist_ttl
            ]

        if self.blacklist_max_size > 0 and len(self.blacklist) > self.blacklist_max_size:
            self.blacklist = self.blacklist[-self.blacklist_max_size:]

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

    def _has_free_clearance(self, gx, gy, width, height, data):
        # frontier 주변에 장애물이 너무 가까우면, move_base가 접근하기 어렵다고 보고 제외한다.
        radius = self.frontier_clearance_cells
        for ny in range(max(0, gy - radius), min(height, gy + radius + 1)):
            for nx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                idx = self._to_index(nx, ny, width)
                if data[idx] > self.occupied_threshold:
                    return False
        return True

    def _count_cells_around(self, gx, gy, width, height, data, radius, predicate):
        count = 0
        for ny in range(max(0, gy - radius), min(height, gy + radius + 1)):
            for nx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                idx = self._to_index(nx, ny, width)
                if predicate(data[idx]):
                    count += 1
        return count

    def _score_frontier(self, gx, gy, width, height, data, distance):
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

        score = (
            (self.distance_weight * distance)
            + (self.obstacle_penalty_weight * obstacle_penalty)
            - (self.information_gain_weight * information_gain)
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

        for gy in range(1, height - 1):
            for gx in range(1, width - 1):
                idx = self._to_index(gx, gy, width)
                if data[idx] != 0:
                    continue

                # frontier는 미탐색 셀(-1)과 맞닿아 있는 자유 공간 셀(0)로 본다.
                has_unknown_neighbor = False
                for ny in range(gy - 1, gy + 2):
                    for nx in range(gx - 1, gx + 2):
                        nidx = self._to_index(nx, ny, width)
                        if data[nidx] == -1:
                            has_unknown_neighbor = True
                            break
                    if has_unknown_neighbor:
                        break

                if not has_unknown_neighbor:
                    continue

                if not self._has_free_clearance(gx, gy, width, height, data):
                    continue

                wx, wy = self._grid_to_world(gx, gy, origin_x, origin_y, resolution)
                distance = math.hypot(wx - robot_x, wy - robot_y)
                if distance < self.min_goal_distance:
                    continue

                if self._is_blacklisted(wx, wy):
                    continue

                score, information_gain, obstacle_penalty = self._score_frontier(
                    gx,
                    gy,
                    width,
                    height,
                    data,
                    distance,
                )
                if best_score is None or score < best_score:
                    best_score = score
                    best_goal = (wx, wy, score, distance, information_gain, obstacle_penalty)

        return best_goal

    def _send_goal(self, goal_x, goal_y, robot_yaw, goal_info=None):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation = yaw_to_quaternion(robot_yaw)
        self.move_base.send_goal(goal)
        self.current_goal = (goal_x, goal_y)
        self.last_goal_time = rospy.Time.now()
        if goal_info is None:
            rospy.loginfo("Frontier goal sent: x=%.2f y=%.2f", goal_x, goal_y)
        else:
            rospy.loginfo(
                "Frontier goal sent: x=%.2f y=%.2f score=%.2f distance=%.2f information=%d obstacle_penalty=%d",
                goal_x,
                goal_y,
                goal_info["score"],
                goal_info["distance"],
                goal_info["information_gain"],
                goal_info["obstacle_penalty"],
            )

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
            self._add_to_blacklist(self.current_goal)
            self.current_goal = None
            return

        # move_base가 너무 오래 같은 goal에 매달리면,
        # 일단 그 구역은 포기하고 다른 frontier를 고르게 한다.
        if rospy.Time.now() - self.last_goal_time > self.goal_timeout:
            rospy.logwarn("Frontier goal timed out, blacklisting area.")
            self.move_base.cancel_goal()
            self._add_to_blacklist(self.current_goal)
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

            self._handle_goal_state()

            if self.current_goal is None:
                frontier_goal = self._find_frontier_goal(robot_x, robot_y)
                if frontier_goal is not None:
                    # 목표 자세는 현재 로봇 heading을 대략 유지하고,
                    # 세부 접근은 move_base가 처리하도록 둔다.
                    goal_info = {
                        "score": frontier_goal[2],
                        "distance": frontier_goal[3],
                        "information_gain": frontier_goal[4],
                        "obstacle_penalty": frontier_goal[5],
                    }
                    self._send_goal(frontier_goal[0], frontier_goal[1], robot_yaw, goal_info)
                else:
                    rospy.loginfo_throttle(5.0, "No reachable frontier found right now.")

            rate.sleep()


def main():
    rospy.init_node("frontier_explore_node")
    FrontierExploreNode().run()


if __name__ == "__main__":
    main()
