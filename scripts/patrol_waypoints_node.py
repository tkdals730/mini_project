#!/usr/bin/env python3

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler


def quaternion_from_yaw(yaw):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


class PatrolWaypointsNode:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.initial_pose_received = False

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_pose_callback)

        self.loop_enabled = rospy.get_param("~patrol_loop", True)
        self.wait_after_goal_sec = rospy.get_param("~wait_after_goal_sec", 5.0)
        self.initial_wait_sec = rospy.get_param("~initial_wait_sec", 2.0)
        self.waypoints = rospy.get_param("~waypoints", [])

        if not self.waypoints:
            rospy.logerr("No waypoints configured. Set ~waypoints before starting patrol.")
            raise rospy.ROSInitException("waypoints parameter is empty")

        rospy.loginfo("Waiting for move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(20.0)):
            raise rospy.ROSInitException("move_base action server is not available")

        rospy.loginfo("Waiting for AMCL pose before starting patrol...")
        wait_rate = rospy.Rate(5)
        while not rospy.is_shutdown() and not self.initial_pose_received:
            wait_rate.sleep()

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
