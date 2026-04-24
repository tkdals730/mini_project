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

        self.goal_timeout = rospy.Duration(rospy.get_param("~goal_timeout_sec", 15.0))
        self.min_goal_distance = rospy.get_param("~min_goal_distance", 0.7)
        self.blacklist_radius = rospy.get_param("~blacklist_radius", 0.6)
        self.frontier_clearance_cells = rospy.get_param("~frontier_clearance_cells", 2)

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
        for bx, by in self.blacklist:
            if math.hypot(wx - bx, wy - by) <= self.blacklist_radius:
                return True
        return False

    def _has_free_clearance(self, gx, gy, width, height, data):
        radius = self.frontier_clearance_cells
        for ny in range(max(0, gy - radius), min(height, gy + radius + 1)):
            for nx in range(max(0, gx - radius), min(width, gx + radius + 1)):
                idx = self._to_index(nx, ny, width)
                if data[idx] > 40:
                    return False
        return True

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

                score = distance
                if best_score is None or score < best_score:
                    best_score = score
                    best_goal = (wx, wy)

        return best_goal

    def _send_goal(self, goal_x, goal_y, robot_yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation = yaw_to_quaternion(robot_yaw)
        self.move_base.send_goal(goal)
        self.current_goal = (goal_x, goal_y)
        self.last_goal_time = rospy.Time.now()
        rospy.loginfo("Frontier goal sent: x=%.2f y=%.2f", goal_x, goal_y)

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

            self._handle_goal_state()

            if self.current_goal is None:
                frontier_goal = self._find_frontier_goal(robot_x, robot_y)
                if frontier_goal is not None:
                    self._send_goal(frontier_goal[0], frontier_goal[1], robot_yaw)
                else:
                    rospy.loginfo_throttle(5.0, "No reachable frontier found right now.")

            rate.sleep()


def main():
    rospy.init_node("frontier_explore_node")
    FrontierExploreNode().run()


if __name__ == "__main__":
    main()
