#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

front = 999
left = 999
right = 999

def scan_callback(msg):
    global front, left, right

    ranges = msg.ranges
    n = len(ranges)

    # 구간 나누기
    front_vals = ranges[n//2 - 20 : n//2 + 20]
    left_vals  = ranges[int(n*0.75) : int(n*0.9)]
    right_vals = ranges[int(n*0.1)  : int(n*0.25)]

    def get_min(vals):
        vals = [v for v in vals if v > 0.05 and v < 10.0]
        return min(vals) if vals else 999

    front = get_min(front_vals)
    left  = get_min(left_vals)
    right = get_min(right_vals)

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