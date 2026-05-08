#!/usr/bin/env python3

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_from_yaw(yaw):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class PatrolWaypointsNode:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.initial_pose_received = False
        self.marker_pub = rospy.Publisher(
            "/patrol_waypoints/markers", MarkerArray, queue_size=1, latch=True
        )

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_pose_callback)

        self.loop_enabled = rospy.get_param("~patrol_loop", True)
        self.wait_after_goal_sec = rospy.get_param("~wait_after_goal_sec", 5.0)
        # 위치 초기화 직후 goal을 보내면 경로가 흔들릴 수 있어서 대기 시간을 둔다.
        self.initial_wait_sec = rospy.get_param("~initial_wait_sec", 2.0)
        self.waypoints = rospy.get_param("~waypoints", [])

        if not self.waypoints:
            rospy.logerr("No waypoints configured. Set ~waypoints before starting patrol.")
            raise rospy.ROSInitException("waypoints parameter is empty")

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

    def _build_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.get("x", 0.0)
        goal.target_pose.pose.position.y = waypoint.get("y", 0.0)
        goal.target_pose.pose.orientation = quaternion_from_yaw(waypoint.get("yaw", 0.0))
        return goal

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

        for index, waypoint in enumerate(self.waypoints, start=1):
            x = waypoint.get("x", 0.0)
            y = waypoint.get("y", 0.0)
            route.points.append(Point(x=x, y=y, z=0.03))

            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = stamp
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
            sphere.color.r = 0.05
            sphere.color.g = 0.8
            sphere.color.b = 0.35
            sphere.color.a = 0.95
            markers.markers.append(sphere)

            label = Marker()
            label.header.frame_id = "map"
            label.header.stamp = stamp
            label.ns = "patrol_waypoint_labels"
            label.id = index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = x
            label.pose.position.y = y
            label.pose.position.z = 0.35
            label.pose.orientation.w = 1.0
            label.scale.z = 0.28
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 1.0
            label.text = "WP%d" % index
            markers.markers.append(label)

        if self.loop_enabled and route.points:
            route.points.append(route.points[0])
        markers.markers.append(route)
        self.marker_pub.publish(markers)

    def run(self):
        waypoint_count = len(self.waypoints)
        cycle_index = 0

        while not rospy.is_shutdown():
            cycle_index += 1
            rospy.loginfo("Starting patrol cycle %d with %d waypoints.", cycle_index, waypoint_count)

            for index, waypoint in enumerate(self.waypoints, start=1):
                goal = self._build_goal(waypoint)
                rospy.loginfo(
                    "Sending waypoint %d/%d: x=%.2f y=%.2f yaw=%.2f",
                    index,
                    waypoint_count,
                    waypoint.get("x", 0.0),
                    waypoint.get("y", 0.0),
                    waypoint.get("yaw", 0.0),
                )
                self.client.send_goal(goal)
                self.client.wait_for_result()
                state = self.client.get_state()

                # 순찰 루프를 단순하게 유지하기 위해, goal 하나가 실패하면
                # 복잡한 복구 로직 대신 다음 사이클에서 다시 시도한다.
                if state != GoalStatus.SUCCEEDED:
                    rospy.logwarn("Waypoint %d failed with state %d. Retrying on next cycle.", index, state)
                    break

                wait_sec = waypoint.get("wait_sec", self.wait_after_goal_sec)
                if wait_sec > 0.0:
                    rospy.loginfo("Waypoint %d reached. Waiting %.1f seconds for inspection.", index, wait_sec)
                    rospy.sleep(wait_sec)
            else:
                rospy.loginfo("Patrol cycle %d completed.", cycle_index)

            if not self.loop_enabled:
                rospy.loginfo("Patrol loop disabled, stopping after one cycle.")
                return

            rospy.sleep(1.0)


def main():
    rospy.init_node("patrol_waypoints_node")
    PatrolWaypointsNode().run()


if __name__ == "__main__":
    main()
