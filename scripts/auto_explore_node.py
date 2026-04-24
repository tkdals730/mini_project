#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


SECTORS = {
    "front": 999.0,
    "front_left": 999.0,
    "left": 999.0,
    "front_right": 999.0,
    "right": 999.0,
}


def clamp(value, low, high):
    return max(low, min(high, value))


def _sector_min(msg, center_rad, half_width_rad):
    values = []
    for index, measured_range in enumerate(msg.ranges):
        if not (msg.range_min < measured_range < msg.range_max):
            continue

        angle = msg.angle_min + (index * msg.angle_increment)
        diff = math.atan2(
            math.sin(angle - center_rad),
            math.cos(angle - center_rad),
        )

        if abs(diff) <= half_width_rad:
            values.append(measured_range)

    return min(values) if values else msg.range_max


def scan_callback(msg):
    global SECTORS

    SECTORS["front"] = _sector_min(msg, 0.0, half_width_rad=0.28)
    SECTORS["front_left"] = _sector_min(msg, math.pi / 4.0, half_width_rad=0.26)
    SECTORS["left"] = _sector_min(msg, math.pi / 2.0, half_width_rad=0.30)
    SECTORS["front_right"] = _sector_min(msg, -math.pi / 4.0, half_width_rad=0.26)
    SECTORS["right"] = _sector_min(msg, -math.pi / 2.0, half_width_rad=0.30)


class AutoExploreController:
    def __init__(self):
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, scan_callback)

        self.mode = "follow_wall"
        self.mode_until = rospy.Time(0)
        self.last_branch_turn_time = rospy.Time(0)
        self.prefer_left_at_intersection = True

        self.front_danger = 0.32
        self.front_blocked = 0.60
        self.side_opening = 1.10
        self.target_right = 0.48
        self.wall_detect = 1.15
        self.base_speed = 0.13
        self.kp_right = 1.7
        self.branch_cooldown = rospy.Duration(4.0)

    def _start_mode(self, mode_name, duration_sec):
        self.mode = mode_name
        self.mode_until = rospy.Time.now() + rospy.Duration(duration_sec)

    def _turn_direction_from_space(self):
        return 1.0 if SECTORS["left"] >= SECTORS["right"] else -1.0

    def _should_take_left_branch(self):
        return (
            SECTORS["front"] > 0.85
            and SECTORS["front_left"] > self.side_opening
            and SECTORS["left"] > self.side_opening
            and SECTORS["front_left"] > (SECTORS["front_right"] + 0.25)
        )

    def _should_take_right_branch(self):
        return (
            SECTORS["front"] > 0.85
            and SECTORS["front_right"] > self.side_opening
            and SECTORS["right"] > self.side_opening
            and SECTORS["front_right"] > (SECTORS["front_left"] + 0.25)
        )

    def _branch_ready(self):
        return rospy.Time.now() - self.last_branch_turn_time > self.branch_cooldown

    def _maybe_enter_branch(self):
        if not self._branch_ready():
            return False

        left_open = self._should_take_left_branch()
        right_open = self._should_take_right_branch()

        if not left_open and not right_open:
            return False

        if left_open and right_open:
            turn_left = self.prefer_left_at_intersection
            self.prefer_left_at_intersection = not self.prefer_left_at_intersection
        else:
            turn_left = left_open

        self.last_branch_turn_time = rospy.Time.now()
        self._start_mode("branch_left" if turn_left else "branch_right", 1.8)
        return True

    def _command_for_active_mode(self):
        now = rospy.Time.now()
        if now >= self.mode_until:
            self.mode = "follow_wall"

        cmd = Twist()

        if self.mode == "branch_left":
            cmd.linear.x = 0.07
            cmd.angular.z = 0.8
            return cmd

        if self.mode == "branch_right":
            cmd.linear.x = 0.07
            cmd.angular.z = -0.8
            return cmd

        if self.mode == "recover_turn":
            cmd.linear.x = -0.04
            cmd.angular.z = 0.9 * self._turn_direction_from_space()
            return cmd

        return None

    def step(self):
        active_cmd = self._command_for_active_mode()
        if active_cmd is not None:
            self.publisher.publish(active_cmd)
            return

        cmd = Twist()
        front = SECTORS["front"]
        left = SECTORS["left"]
        right = SECTORS["right"]
        front_left = SECTORS["front_left"]
        front_right = SECTORS["front_right"]

        if front < self.front_danger:
            self._start_mode("recover_turn", 1.2)
            cmd.linear.x = -0.05
            cmd.angular.z = 0.9 * self._turn_direction_from_space()

        elif front < self.front_blocked:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.75 * self._turn_direction_from_space()

        elif self._maybe_enter_branch():
            cmd.linear.x = 0.07
            cmd.angular.z = 0.8 if self.mode == "branch_left" else -0.8

        else:
            cmd.linear.x = self.base_speed

            if right < self.wall_detect:
                error = self.target_right - right
                cmd.angular.z = clamp(self.kp_right * error, -0.65, 0.65)
            else:
                # Right wall was lost, so gently curve right to keep exploring new corridors.
                cmd.angular.z = -0.32

            # If the left side opens much more than the current path, bias slightly left
            # so the robot does not keep skipping branch entrances.
            if front_left > (front_right + 0.45) and left > self.side_opening:
                cmd.angular.z += 0.22

        self.publisher.publish(cmd)


def main():
    rospy.init_node("auto_explore_node")
    controller = AutoExploreController()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        controller.step()
        rate.sleep()


if __name__ == "__main__":
    main()
