#!/usr/bin/env python3
import math
import os

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def _yaw_to_quaternion(yaw):
    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
    return qx, qy, qz, qw


class AutoMapSaverNode:
    def __init__(self):
        self.map_msg = None
        self.saved = False
        self.returned_home = False

        self.map_output = rospy.get_param("~map_output", "maps/patrol_map")
        self.occupied_thresh = rospy.get_param("~occupied_thresh", 0.65)
        self.free_thresh = rospy.get_param("~free_thresh", 0.196)
        self.min_map_updates = rospy.get_param("~min_map_updates", 3)
        self.map_updates = 0
        self.return_home_after_save = rospy.get_param("~return_home_after_save", True)
        self.home_approach_waypoint = rospy.get_param("~home_approach_waypoint", {})
        self.home_waypoint = rospy.get_param("~home_waypoint", {})
        self.home_goal_timeout_sec = rospy.get_param("~home_goal_timeout_sec", 45.0)
        self.completion_escape_enabled = rospy.get_param("~completion_escape_enabled", True)
        self.completion_escape_reverse_sec = rospy.get_param("~completion_escape_reverse_sec", 2.5)
        self.completion_escape_reverse_speed = rospy.get_param("~completion_escape_reverse_speed", 0.08)
        self.completion_escape_cmd_vel_topic = rospy.get_param(
            "~completion_escape_cmd_vel_topic", "/cmd_vel_nav"
        )
        self.clear_costmaps_before_home = rospy.get_param("~clear_costmaps_before_home", True)

        self.move_base = None
        self.cmd_vel_pub = None
        self.clear_costmaps = None
        if self.return_home_after_save:
            self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
            self.cmd_vel_pub = rospy.Publisher(
                self.completion_escape_cmd_vel_topic, Twist, queue_size=1
            )
            self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)

        rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber("/exploration_complete", Bool, self._complete_callback, queue_size=1)

        rospy.loginfo("Auto map saver ready: output=%s", self.map_output)

    def _map_callback(self, msg):
        self.map_msg = msg
        self.map_updates += 1

    def _complete_callback(self, msg):
        if not msg.data or self.saved:
            return

        if self.map_msg is None:
            rospy.logwarn("Exploration complete, but no /map has been received yet.")
            return

        if self.map_updates < self.min_map_updates:
            rospy.logwarn(
                "Exploration complete, but only %d map updates received. Waiting for more.",
                self.map_updates,
            )
            return

        self._save_map(self.map_msg)
        self.saved = True
        self._return_home_after_save()

    def _save_map(self, msg):
        output_base = os.path.expanduser(self.map_output)
        output_dir = os.path.dirname(output_base)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        pgm_path = output_base + ".pgm"
        yaml_path = output_base + ".yaml"
        image_name = os.path.basename(pgm_path)

        width = msg.info.width
        height = msg.info.height
        data = msg.data

        with open(pgm_path, "wb") as pgm_file:
            pgm_file.write(("P5\n# CREATOR: auto_map_saver_node.py\n%d %d\n255\n" % (width, height)).encode("ascii"))
            for y in range(height - 1, -1, -1):
                for x in range(width):
                    value = data[y * width + x]
                    if value < 0:
                        pixel = 205
                    elif value >= int(self.occupied_thresh * 100.0):
                        pixel = 0
                    elif value <= int(self.free_thresh * 100.0):
                        pixel = 254
                    else:
                        pixel = 205
                    pgm_file.write(bytes([pixel]))

        origin = msg.info.origin
        quat = origin.orientation
        yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
        if math.isnan(yaw):
            yaw = 0.0

        with open(yaml_path, "w", encoding="utf-8") as yaml_file:
            yaml_file.write("image: %s\n" % image_name)
            yaml_file.write("resolution: %.12g\n" % msg.info.resolution)
            yaml_file.write(
                "origin: [%.12g, %.12g, %.12g]\n"
                % (origin.position.x, origin.position.y, yaw)
            )
            yaml_file.write("negate: 0\n")
            yaml_file.write("occupied_thresh: %.12g\n" % self.occupied_thresh)
            yaml_file.write("free_thresh: %.12g\n" % self.free_thresh)

        rospy.loginfo("Saved map automatically: %s and %s", yaml_path, pgm_path)

    def _has_waypoint(self, waypoint):
        return isinstance(waypoint, dict) and bool(waypoint)

    def _build_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint.get("x", 0.0)
        goal.target_pose.pose.position.y = waypoint.get("y", 0.0)
        qx, qy, qz, qw = _yaw_to_quaternion(waypoint.get("yaw", 0.0))
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw
        return goal

    def _send_home_goal(self, waypoint, label):
        if self.move_base is None:
            return False

        rospy.loginfo(
            "Sending %s after map save: x=%.2f y=%.2f yaw=%.2f",
            label,
            waypoint.get("x", 0.0),
            waypoint.get("y", 0.0),
            waypoint.get("yaw", 0.0),
        )
        self.move_base.send_goal(self._build_goal(waypoint))
        finished = self.move_base.wait_for_result(rospy.Duration(self.home_goal_timeout_sec))
        if not finished:
            self.move_base.cancel_goal()
            rospy.logwarn("%s timed out after %.1f sec.", label, self.home_goal_timeout_sec)
            return False

        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("%s failed with state %d.", label, state)
            return False
        return True

    def _publish_stop(self):
        if self.cmd_vel_pub is not None:
            self.cmd_vel_pub.publish(Twist())

    def _clear_move_base_costmaps(self):
        if not self.clear_costmaps_before_home or self.clear_costmaps is None:
            return

        try:
            rospy.wait_for_service("/move_base/clear_costmaps", timeout=2.0)
            self.clear_costmaps()
            rospy.loginfo("Cleared move_base costmaps before home return.")
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Could not clear move_base costmaps before home return: %s", exc)

    def _perform_completion_escape(self):
        if (
            not self.completion_escape_enabled
            or self.cmd_vel_pub is None
            or self.completion_escape_reverse_sec <= 0.0
            or self.completion_escape_reverse_speed <= 0.0
        ):
            return

        if self.move_base is not None:
            self.move_base.cancel_all_goals()

        rospy.loginfo(
            "Backing away from exploration endpoint before home return: %.1fs at %.2fm/s",
            self.completion_escape_reverse_sec,
            self.completion_escape_reverse_speed,
        )
        reverse = Twist()
        reverse.linear.x = -abs(self.completion_escape_reverse_speed)
        end_time = rospy.Time.now() + rospy.Duration(self.completion_escape_reverse_sec)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(reverse)
            rate.sleep()

        self._publish_stop()
        rospy.sleep(0.2)

    def _return_home_after_save(self):
        if self.returned_home or not self.return_home_after_save:
            return
        if not self._has_waypoint(self.home_waypoint):
            rospy.logwarn("Map saved, but ~home_waypoint is empty. Skipping return home.")
            return

        rospy.loginfo("Map saved. Returning robot to home position.")
        if self.move_base is not None and not self.move_base.wait_for_server(rospy.Duration(10.0)):
            rospy.logwarn("move_base action server is not available. Skipping return home.")
            return

        self._perform_completion_escape()
        self._clear_move_base_costmaps()

        if self._has_waypoint(self.home_approach_waypoint):
            if not self._send_home_goal(self.home_approach_waypoint, "home entry waypoint"):
                return

        if self._send_home_goal(self.home_waypoint, "home waypoint"):
            self.returned_home = True
            rospy.loginfo("Robot returned home after map save.")


def main():
    rospy.init_node("auto_map_saver_node")
    AutoMapSaverNode()
    rospy.spin()


if __name__ == "__main__":
    main()
