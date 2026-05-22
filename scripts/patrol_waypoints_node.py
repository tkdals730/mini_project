#!/usr/bin/env python3

import actionlib
import os
import rospy
import time
import yaml
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_yaw(yaw):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class PatrolWaypointsNode:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.initial_pose_received = False
        self.patrol_paused = False
        self.stop_requested = False
        self.pause_generation = 0
        self.marker_pub = rospy.Publisher(
            "/patrol_waypoints/markers", MarkerArray, queue_size=1, latch=True
        )
        self.finished_pub = rospy.Publisher("/patrol_finished", Bool, queue_size=1)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_pose_callback)
        rospy.Subscriber("/patrol_pause", Bool, self._patrol_pause_callback, queue_size=1)
        rospy.Subscriber(
            "/patrol_stop_requested", Bool, self._stop_requested_callback, queue_size=1
        )

        self.loop_enabled = rospy.get_param("~patrol_loop", True)
        self.return_home_enabled = rospy.get_param("~return_home", True)
        self.wait_after_goal_sec = rospy.get_param("~wait_after_goal_sec", 5.0)
        self.goal_timeout = rospy.Duration(rospy.get_param("~goal_timeout_sec", 75.0))
        self.home_goal_timeout = rospy.Duration(
            rospy.get_param("~home_goal_timeout_sec", self.goal_timeout.to_sec())
        )
        # 위치 초기화 직후 goal을 보내면 경로가 흔들릴 수 있어서 대기 시간을 둔다.
        self.initial_wait_sec = rospy.get_param("~initial_wait_sec", 2.0)
        self.waypoints_file = rospy.get_param("~waypoints_file", "")
        self.waypoints_file_wait_sec = rospy.get_param("~waypoints_file_wait_sec", 3.0)
        self.waypoints_file_require_fresh = rospy.get_param(
            "~waypoints_file_require_fresh", False
        )
        self.node_start_wall_time = time.time()
        self.waypoints = self._load_waypoints_from_file(self.waypoints_file)
        self.home_approach_waypoint = rospy.get_param("~home_approach_waypoint", {})
        self.home_waypoint = rospy.get_param("~home_waypoint", {})

        if not self.waypoints:
            rospy.logerr(
                "No generated patrol waypoints available. Check waypoint generation before starting patrol."
            )
            raise rospy.ROSInitException("waypoints parameter is empty")

        if self.return_home_enabled and not self._has_home_waypoint():
            rospy.logwarn("~return_home is enabled but ~home_waypoint is empty. Skipping home return.")

        self._publish_waypoint_markers()

        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(20.0)):
            raise rospy.ROSInitException("move_base action server is not available")

        rospy.loginfo("Waiting for AMCL pose before starting patrol...")
        wait_rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.initial_pose_received:
            wait_rate.sleep()

        # 첫 goal을 보내기 전에 AMCL 위치추정이 잠깐 안정될 시간을 둔다.
        rospy.sleep(self.initial_wait_sec)

    def _amcl_pose_callback(self, _msg):
        self.initial_pose_received = True

    def _patrol_pause_callback(self, msg):
        paused = bool(msg.data)
        if paused != self.patrol_paused:
            self.patrol_paused = paused
            if paused:
                self.pause_generation += 1
                rospy.logwarn("Patrol paused.")
            else:
                rospy.loginfo("Patrol pause cleared. Continuing patrol.")

    def _stop_requested_callback(self, msg):
        requested = bool(msg.data)
        if requested != self.stop_requested:
            self.stop_requested = requested
            if requested:
                rospy.loginfo(
                    "Scheduled patrol stop requested. Current patrol cycle will finish before returning home."
                )

    def _load_waypoints_from_file(self, path):
        if not path:
            rospy.logerr("~waypoints_file is empty. Generated waypoints are required.")
            return []

        expanded_path = os.path.expanduser(path)
        wait_until = time.time() + self.waypoints_file_wait_sec
        while (
            not rospy.is_shutdown()
            and self.waypoints_file_wait_sec > 0.0
            and not self._waypoints_file_ready(expanded_path)
            and time.time() < wait_until
        ):
            rospy.sleep(0.1)

        if not os.path.exists(expanded_path):
            rospy.logerr(
                "Generated waypoint file not found: %s.",
                expanded_path,
            )
            return []

        if self.waypoints_file_require_fresh and not self._waypoints_file_ready(expanded_path):
            rospy.logerr(
                "Generated waypoint file was not refreshed in time: %s.",
                expanded_path,
            )
            return []

        with open(expanded_path, "r", encoding="utf-8") as waypoint_file:
            data = yaml.safe_load(waypoint_file) or {}

        if isinstance(data, dict):
            waypoints = data.get("waypoints", [])
        else:
            waypoints = data

        if not isinstance(waypoints, list):
            rospy.logerr(
                "Waypoint file %s does not contain a waypoint list.",
                expanded_path,
            )
            return []

        valid_waypoints = []
        for index, waypoint in enumerate(waypoints, start=1):
            if not isinstance(waypoint, dict) or "x" not in waypoint or "y" not in waypoint:
                rospy.logwarn("Skipping invalid waypoint %d from %s.", index, expanded_path)
                continue
            valid_waypoints.append(
                {
                    "x": float(waypoint.get("x", 0.0)),
                    "y": float(waypoint.get("y", 0.0)),
                    "yaw": float(waypoint.get("yaw", 0.0)),
                    "wait_sec": float(waypoint.get("wait_sec", self.wait_after_goal_sec)),
                }
            )

        if valid_waypoints:
            rospy.loginfo(
                "Loaded %d generated patrol waypoints from %s.",
                len(valid_waypoints),
                expanded_path,
            )
        return valid_waypoints

    def _waypoints_file_ready(self, path):
        if not os.path.exists(path):
            return False
        if not self.waypoints_file_require_fresh:
            return True

        # roslaunch starts nodes in parallel. Allow a small wall-time margin so a
        # generator that finishes just before this node initializes still counts.
        return os.path.getmtime(path) >= self.node_start_wall_time - 2.0

    def _build_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.get("x", 0.0)
        goal.target_pose.pose.position.y = waypoint.get("y", 0.0)
        goal.target_pose.pose.orientation = quaternion_from_yaw(waypoint.get("yaw", 0.0))
        return goal

    def _has_home_waypoint(self):
        return isinstance(self.home_waypoint, dict) and bool(self.home_waypoint)

    def _has_home_approach_waypoint(self):
        return isinstance(self.home_approach_waypoint, dict) and bool(self.home_approach_waypoint)

    def _publish_waypoint_markers(self):
        markers = MarkerArray()
        stamp = rospy.Time.now()

        route = Marker()
        route.header.frame_id = "map"
        route.header.stamp = stamp
        route.ns = "patrol_route"
        route.id = 0
        route.type = Marker.LINE_STRIP
        route.action = Marker.ADD
        route.pose.orientation.w = 1.0
        route.scale.x = 0.04
        route.color.r = 0.1
        route.color.g = 0.45
        route.color.b = 1.0
        route.color.a = 0.9

        display_waypoints = list(self.waypoints)
        has_home_return = self.return_home_enabled and self._has_home_waypoint()
        has_home_approach = has_home_return and self._has_home_approach_waypoint()
        home_approach_index = None
        home_index = None
        if has_home_return:
            if has_home_approach:
                display_waypoints.append(self.home_approach_waypoint)
                home_approach_index = len(display_waypoints)
            display_waypoints.append(self.home_waypoint)
            home_index = len(display_waypoints)

        for index, waypoint in enumerate(display_waypoints, start=1):
            x = waypoint.get("x", 0.0)
            y = waypoint.get("y", 0.0)
            route.points.append(Point(x=x, y=y, z=0.03))
            is_home_approach = has_home_approach and index == home_approach_index
            is_home = has_home_return and index == home_index

            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = stamp
            if is_home:
                sphere.ns = "patrol_home"
            elif is_home_approach:
                sphere.ns = "patrol_home_entry"
            else:
                sphere.ns = "patrol_waypoints"
            sphere.id = index
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = x
            sphere.pose.position.y = y
            sphere.pose.position.z = 0.08
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.28
            sphere.scale.y = 0.28
            sphere.scale.z = 0.08
            sphere.color.r = 1.0 if is_home else 0.05
            sphere.color.g = 0.55 if is_home_approach else 0.8
            sphere.color.b = 0.05 if is_home else 0.35
            sphere.color.a = 0.95
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            if is_home:
                label.ns = "patrol_home_label"
            elif is_home_approach:
                label.ns = "patrol_home_entry_label"
            else:
                label.ns = "patrol_waypoint_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.55 if is_home else 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.24 if is_home else 0.28
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            if is_home:
                label.text = "HOME"
            elif is_home_approach:
                label.text = "ENTRY"
            else:
                label.text = "WP%d" % index
            markers.markers.append(label)

        if self.loop_enabled and route.points and not has_home_return:
            route.points.append(route.points[0])
        markers.markers.append(route)
        self.marker_pub.publish(markers)

    def _send_goal_and_wait(self, waypoint, label, timeout=None):
        if timeout is None:
            timeout = self.goal_timeout
        goal = self._build_goal(waypoint)
        while not rospy.is_shutdown():
            self._wait_while_paused()
            pause_generation_at_send = self.pause_generation
            rospy.loginfo(
                "Sending %s: x=%.2f y=%.2f yaw=%.2f",
                label,
                waypoint.get("x", 0.0),
                waypoint.get("y", 0.0),
                waypoint.get("yaw", 0.0),
            )
            self.client.send_goal(goal)
            deadline = rospy.Time.now() + timeout

            while not rospy.is_shutdown():
                if self.patrol_paused:
                    self.client.cancel_goal()
                    rospy.logwarn("%s canceled while patrol pause is active.", label)
                    self.client.wait_for_result(rospy.Duration(2.0))
                    break

                if self.client.wait_for_result(rospy.Duration(0.2)):
                    state = self.client.get_state()
                    if self._was_interrupted_by_pause(state, pause_generation_at_send):
                        rospy.logwarn(
                            "%s was canceled during patrol pause. It will be retried after pause clears.",
                            label,
                        )
                        break
                    return state

                if rospy.Time.now() >= deadline:
                    self.client.cancel_goal()
                    rospy.logwarn("%s timed out after %.1f sec.", label, timeout.to_sec())
                    return GoalStatus.ABORTED

            self._wait_while_paused()
            rospy.loginfo("Retrying %s after patrol pause.", label)

        return GoalStatus.PREEMPTED

    def _was_interrupted_by_pause(self, state, pause_generation_at_send):
        if state not in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
            return False
        return self.patrol_paused or self.pause_generation != pause_generation_at_send

    def _wait_while_paused(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown() and self.patrol_paused:
            rate.sleep()

    def _wait_at_waypoint(self, waypoint, label):
        wait_sec = waypoint.get("wait_sec", self.wait_after_goal_sec)
        if wait_sec > 0.0:
            rospy.loginfo("%s reached. Waiting %.1f seconds for inspection.", label, wait_sec)
            end_time = rospy.Time.now() + rospy.Duration(wait_sec)
            rate = rospy.Rate(10)
            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                if self.patrol_paused:
                    self._wait_while_paused()
                else:
                    rate.sleep()

    def _return_home(self, cycle_index, reason):
        if not self.return_home_enabled or not self._has_home_waypoint():
            return

        rospy.loginfo("Returning home after patrol cycle %d: %s", cycle_index, reason)
        home_entry_reached = True
        if self._has_home_approach_waypoint():
            state = self._send_goal_and_wait(
                self.home_approach_waypoint,
                "home entry waypoint",
                self.home_goal_timeout,
            )
            if state != GoalStatus.SUCCEEDED:
                rospy.logwarn("Home entry failed with state %d.", state)
                home_entry_reached = False
            else:
                self._wait_at_waypoint(self.home_approach_waypoint, "Home entry")

        state = self._send_goal_and_wait(
            self.home_waypoint,
            "home waypoint",
            self.home_goal_timeout,
        )
        if state == GoalStatus.SUCCEEDED:
            self._wait_at_waypoint(self.home_waypoint, "Home waypoint")
            if home_entry_reached:
                rospy.loginfo("Home return completed after patrol cycle %d.", cycle_index)
            else:
                rospy.loginfo("Home waypoint reached after skipping failed entry waypoint.")
        else:
            rospy.logwarn("Home return failed with state %d.", state)

    def run(self):
        waypoint_count = len(self.waypoints)
        cycle_index = 0
        self.finished_pub.publish(Bool(data=False))

        while not rospy.is_shutdown():
            cycle_index += 1
            rospy.loginfo("Starting patrol cycle %d with %d waypoints.", cycle_index, waypoint_count)

            for index, waypoint in enumerate(self.waypoints, start=1):
                label = "waypoint %d/%d" % (index, waypoint_count)
                state = self._send_goal_and_wait(waypoint, label)

                # 순찰 루프를 단순하게 유지하기 위해, goal 하나가 실패하면
                # 복잡한 복구 로직 대신 다음 사이클에서 다시 시도한다.
                if state != GoalStatus.SUCCEEDED:
                    rospy.logwarn(
                        "Waypoint %d failed with state %d. Returning home before stopping or retrying.",
                        index,
                        state,
                    )
                    self._return_home(cycle_index, "waypoint %d failed" % index)
                    self.finished_pub.publish(Bool(data=True))
                    break

                self._wait_at_waypoint(waypoint, "Waypoint %d" % index)
            else:
                rospy.loginfo("Patrol cycle %d completed.", cycle_index)
                self._return_home(cycle_index, "patrol cycle completed")
                self.finished_pub.publish(Bool(data=True))
                if self.stop_requested:
                    rospy.loginfo("Scheduled stop request completed after patrol cycle %d.", cycle_index)
                    return

            if not self.loop_enabled:
                rospy.loginfo("Patrol loop disabled, stopping after one cycle.")
                return

            if self.stop_requested:
                rospy.loginfo("Scheduled stop request accepted between patrol cycles.")
                return

            rospy.sleep(1.0)


def main():
    rospy.init_node("patrol_waypoints_node")
    try:
        PatrolWaypointsNode().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
