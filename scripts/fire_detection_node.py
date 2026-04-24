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
        self.last_alert_time = rospy.Time(0)

        # 테스트 월드의 fire_target은 색이 뚜렷해서,
        # 우선은 단순한 threshold 기반으로 감지한다.
        self.min_area_ratio = rospy.get_param("~min_area_ratio", 0.015)
        # 짧은 노이즈나 반짝임을 줄이기 위해 연속 프레임 기준을 둔다.
        self.min_frames = rospy.get_param("~min_frames", 3)
        # 화재가 계속 보이는 동안 경고 로그를 이 주기로 반복한다.
        self.alert_interval = rospy.Duration(rospy.get_param("~alert_interval_sec", 2.0))
        self.use_debug_image = rospy.get_param("~use_debug_image", True)

        self.detection_pub = rospy.Publisher("/fire_detected", Bool, queue_size=10)
        self.debug_pub = rospy.Publisher("/fire_detection/debug_image", Image, queue_size=1)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self._image_callback, queue_size=1)

    def _build_fire_mask(self, hsv_image):
        # 불 오브젝트의 발광색과 비슷한 빨강/주황 계열을 함께 마스킹한다.
        # 실제 화재 일반화 모델이 아니라, 현재 Gazebo 오브젝트를 안정적으로 잡기 위한 기준이다.
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

        # 작은 노이즈보다 가장 큰 후보 영역을 기준으로 판단한다.
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > largest_area:
                largest_area = area
                best_rect = cv2.boundingRect(contour)

        area_ratio = largest_area / frame_area if frame_area > 0 else 0.0
        # 화면 대비 일정 비율 이상 보여야 실제 화재 후보로 인정한다.
        frame_detected = area_ratio >= self.min_area_ratio

        if frame_detected:
            self.detected_streak += 1
        else:
            self.detected_streak = 0

        # 한 프레임짜리 오탐을 줄이기 위해 연속 프레임 기준을 둔다.
        new_state = self.detected_streak >= self.min_frames
        if new_state != self.fire_detected:
            self.fire_detected = new_state
            if self.fire_detected:
                self.last_alert_time = rospy.Time.now()
                rospy.logwarn("FIRE DETECTED: area_ratio=%.3f streak=%d", area_ratio, self.detected_streak)
            else:
                rospy.loginfo("Fire detection cleared.")

        if self.fire_detected and rospy.Time.now() - self.last_alert_time >= self.alert_interval:
            self.last_alert_time = rospy.Time.now()
            rospy.logwarn("FIRE STILL DETECTED: area_ratio=%.3f streak=%d", area_ratio, self.detected_streak)

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
