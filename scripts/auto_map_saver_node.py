#!/usr/bin/env python3
import math
import os

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


class AutoMapSaverNode:
    def __init__(self):
        self.map_msg = None
        self.saved = False

        self.map_output = rospy.get_param("~map_output", "maps/patrol_map")
        self.occupied_thresh = rospy.get_param("~occupied_thresh", 0.65)
        self.free_thresh = rospy.get_param("~free_thresh", 0.196)
        self.min_map_updates = rospy.get_param("~min_map_updates", 3)
        self.map_updates = 0

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


def main():
    rospy.init_node("auto_map_saver_node")
    AutoMapSaverNode()
    rospy.spin()


if __name__ == "__main__":
    main()
