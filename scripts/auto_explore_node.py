#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

front = 999.0
left = 999.0
right = 999.0


def _sector_min(msg, center_rad, half_width_rad=0.30):
    vals = []
    for i, r in enumerate(msg.ranges):
        if not (msg.range_min < r < msg.range_max):
            continue
        angle = msg.angle_min + (i * msg.angle_increment)

        # -pi~pi 범위로 정규화해서 섹터 비교
        diff = math.atan2(math.sin(angle - center_rad), math.cos(angle - center_rad))
        if abs(diff) <= half_width_rad:
            vals.append(r)

    return min(vals) if vals else 999.0

def scan_callback(msg):
    global front, left, right

    # LaserScan 각도 기준으로 전/좌/우 거리 계산
    # 전방: 0rad, 좌측: +90deg, 우측: -90deg
    front = _sector_min(msg, 0.0, half_width_rad=0.35)
    left = _sector_min(msg, math.pi / 2.0, half_width_rad=0.35)
    right = _sector_min(msg, -math.pi / 2.0, half_width_rad=0.35)

def main():
    rospy.init_node("auto_explore_node")

    rospy.Subscriber("/scan", LaserScan, scan_callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)

    SAFE_FRONT = 0.8
    DANGER_FRONT = 0.45
    SAFE_SIDE = 0.35

    while not rospy.is_shutdown():
        cmd = Twist()


        if front < DANGER_FRONT:
            cmd.linear.x = -0.12
            cmd.angular.z = 0.8 if left > right else -0.8

        elif left < SAFE_SIDE:
            cmd.linear.x = 0.03
            cmd.angular.z = -0.6

        elif right < SAFE_SIDE:
            cmd.linear.x = 0.03
            cmd.angular.z = 0.6

        elif front < SAFE_FRONT:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.6 if left > right else -0.6

        else:
            cmd.linear.x = 0.12
            cmd.angular.z = 0.15 if left > right else -0.15

        pub.publish(cmd)
        rate.sleep()


if __name__ == "__main__":
    main()