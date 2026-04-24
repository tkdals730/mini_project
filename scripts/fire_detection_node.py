#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class FireDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.detected_streak = 0
        self.fire_detected = False

        self.min_area_ratio = rospy.get_param("~min_area_ratio", 0.015)
        self.min_frames = rospy.get_param("~min_frames", 3)
        self.use_debug_image = rospy.get_param("~use_debug_image", True)

        self.detection_pub = rospy.Publisher("/fire_detected", Bool, queue_size=10)
        self.debug_pub = rospy.Publisher("/fire_detection/debug_image", Image, queue_size=1)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self._image_callback, queue_size=1)

    def _build_fire_mask(self, hsv_image):
        lower_red_1 = (0, 120, 120)
        upper_red_1 = (10, 255, 255)
        lower_red_2 = (170, 120, 120)
        upper_red_2 = (180, 255, 255)
        lower_orange = (8, 90, 140)
        upper_orange = (30, 255, 255)

        red_mask_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        red_mask_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        orange_mask = cv2.inRange(hsv_image, lower_orange, upper_orange)

        mask = cv2.bitwise_or(red_mask_1, red_mask_2)
        mask = cv2.bitwise_or(mask, orange_mask)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def _image_callback(self, msg):
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "Failed to convert camera image: %s", exc)
            return

        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        mask = self._build_fire_mask(hsv_image)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        frame_area = float(bgr_image.shape[0] * bgr_image.shape[1])
        largest_area = 0.0
        best_rect = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                best_rect = cv2.boundingRect(contour)

        area_ratio = largest_area / frame_area if frame_area > 0 else 0.0
        frame_detected = area_ratio >= self.min_area_ratio

        if frame_detected:
            self.detected_streak += 1
        else:
            self.detected_streak = 0

        new_state = self.detected_streak >= self.min_frames
        if new_state != self.fire_detected:
            self.fire_detected = new_state
            if self.fire_detected:
                rospy.logwarn("FIRE DETECTED: area_ratio=%.3f streak=%d", area_ratio, self.detected_streak)
            else:
                rospy.loginfo("Fire detection cleared.")

        self.detection_pub.publish(Bool(data=self.fire_detected))

        if self.use_debug_image:
            debug_image = bgr_image.copy()
            if best_rect is not None:
                x, y, w, h = best_rect
                color = (0, 0, 255) if self.fire_detected else (0, 255, 255)
                cv2.rectangle(debug_image, (x, y), (x + w, y + h), color, 2)
            label = "FIRE DETECTED" if self.fire_detected else "No Fire"
            cv2.putText(
                debug_image,
                f"{label} area={area_ratio:.3f} streak={self.detected_streak}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255) if self.fire_detected else (0, 255, 0),
                2,
            )
            try:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8"))
            except CvBridgeError as exc:
                rospy.logwarn_throttle(5.0, "Failed to publish fire debug image: %s", exc)


def main():
    rospy.init_node("fire_detection_node")
    FireDetectionNode()
    rospy.spin()


if __name__ == "__main__":
    main()
