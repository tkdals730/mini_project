#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

front_dist = 999.0

def scan_callback(msg):
    global front_dist

    # 라이다 정면 근처 거리만 사용
    ranges = list(msg.ranges)
    center = len(ranges) // 2
    front_values = ranges[center-15:center+15]

    valid = [r for r in front_values if r > 0.05 and r < 10.0]

    if valid:
        front_dist = min(valid)
    else:
        front_dist = 999.0

def main():
    rospy.init_node("auto_explore_node")

    rospy.Subscriber("/scan", LaserScan, scan_callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        cmd = Twist()

        if front_dist > 0.7:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5

        pub.publish(cmd)
        rate.sleep()

if __name__ == "__main__":
    main()