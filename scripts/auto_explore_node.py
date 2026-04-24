#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

front = 999.0
left = 999.0
right = 999.0


def clamp(value, low, high):
    return max(low, min(high, value))


def _sector_min(msg, center_rad, half_width_rad=0.30):
    vals = []
    for i, r in enumerate(msg.ranges):
        if not (msg.range_min < r < msg.range_max):
            continue

        angle = msg.angle_min + (i * msg.angle_increment)

        diff = math.atan2(
            math.sin(angle - center_rad),
            math.cos(angle - center_rad)
        )

        if abs(diff) <= half_width_rad:
            vals.append(r)

    return min(vals) if vals else 999.0


def scan_callback(msg):
    global front, left, right

    front = _sector_min(msg, 0.0, half_width_rad=0.35)
    left = _sector_min(msg, math.pi / 2.0, half_width_rad=0.35)
    right = _sector_min(msg, -math.pi / 2.0, half_width_rad=0.35)


def main():
    rospy.init_node("auto_explore_node")

    rospy.Subscriber("/scan", LaserScan, scan_callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)

    FRONT_DANGER = 0.30
    FRONT_BLOCKED = 0.55
    TARGET_RIGHT = 0.50
    WALL_DETECT = 1.20
    BASE_SPEED = 0.13
    KP_RIGHT = 1.6

    while not rospy.is_shutdown():
        cmd = Twist()

        if front < FRONT_DANGER:
            cmd.linear.x = -0.10
            cmd.angular.z = 0.9 if left > right else -0.9

        elif front < FRONT_BLOCKED:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.8 if left > right else -0.8

        else:
            cmd.linear.x = BASE_SPEED

            if right < WALL_DETECT:
                error = TARGET_RIGHT - right
                cmd.angular.z = clamp(KP_RIGHT * error, -0.6, 0.6)
            else:
                cmd.angular.z = 0.0

        pub.publish(cmd)
        rate.sleep()


if __name__ == "__main__":
    main()
