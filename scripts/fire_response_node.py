#!/usr/bin/env python3

from collections import deque

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class FireResponseNode:
    def __init__(self):
        self.stop_window = rospy.Duration(rospy.get_param("~stop_window_sec", 1.5))
        self.stop_required = rospy.Duration(rospy.get_param("~stop_required_sec", 0.25))
        self.alert_window = rospy.Duration(rospy.get_param("~alert_window_sec", 5.0))
        self.alert_required = rospy.Duration(rospy.get_param("~alert_required_sec", 2.5))
        self.clear_window = rospy.Duration(rospy.get_param("~clear_window_sec", 5.0))
        self.clear_max = rospy.Duration(rospy.get_param("~clear_max_sec", 0.2))
        self.stop_window_sec = self.stop_window.to_sec()
        self.stop_required_sec = self.stop_required.to_sec()
        self.alert_window_sec = self.alert_window.to_sec()
        self.alert_required_sec = self.alert_required.to_sec()
        self.clear_window_sec = self.clear_window.to_sec()
        self.clear_max_sec = self.clear_max.to_sec()
        self.update_rate_hz = rospy.get_param("~update_rate_hz", 10.0)
        self.alert_repeat_interval = rospy.Duration(
            rospy.get_param("~alert_repeat_interval_sec", 3.0)
        )

        self.history = deque()
        self.max_window = max(self.stop_window, self.alert_window, self.clear_window)
        self.state = "normal"
        self.last_alert_time = rospy.Time(0)
        self.last_cancel_time = rospy.Time(0)
        self.last_pause = None
        self.last_alert_active = None

        self.pause_pub = rospy.Publisher("/patrol_pause", Bool, queue_size=1, latch=True)
        self.alert_active_pub = rospy.Publisher(
            "/fire_response/alert_active", Bool, queue_size=1, latch=True
        )
        self.state_pub = rospy.Publisher(
            "/fire_response/state", String, queue_size=1, latch=True
        )
        self.alert_pub = rospy.Publisher("/patrol_alert", String, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.move_base_cancel_pub = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)

        rospy.Subscriber("/fire_detected", Bool, self._fire_detected_callback, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.update_rate_hz), self._timer_callback)

        self._publish_state(False, False)
        rospy.loginfo(
            "Fire response ready: stop %.1fs/%.1fs, alert %.1fs/%.1fs, clear %.1fs/%.1fs",
            self.stop_required.to_sec(),
            self.stop_window.to_sec(),
            self.alert_required.to_sec(),
            self.alert_window.to_sec(),
            self.clear_max.to_sec(),
            self.clear_window.to_sec(),
        )

    def _fire_detected_callback(self, msg):
        now = rospy.Time.now().to_sec()
        self.history.append((now, bool(msg.data)))
        self._prune_history(now)

    def _prune_history(self, now):
        keep_window = self.max_window.to_sec() + 1.0
        cutoff = max(0.0, now - keep_window)
        while self.history and self.history[0][0] < cutoff:
            self.history.popleft()

    def _detected_duration(self, window, now):
        if not self.history:
            return 0.0

        start = max(0.0, now - window)
        total = 0.0
        samples = list(self.history)

        for index, (stamp, detected) in enumerate(samples):
            next_stamp = samples[index + 1][0] if index + 1 < len(samples) else now
            segment_start = max(stamp, start)
            segment_end = min(next_stamp, now)
            if detected and segment_end > segment_start:
                total += segment_end - segment_start

        return total

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        now_sec = now.to_sec()
        self._prune_history(now_sec)

        stop_seen = self._detected_duration(self.stop_window_sec, now_sec)
        alert_seen = self._detected_duration(self.alert_window_sec, now_sec)
        clear_seen = self._detected_duration(self.clear_window_sec, now_sec)

        if self.state == "normal":
            if stop_seen >= self.stop_required_sec:
                self.state = "confirming"
                rospy.logwarn(
                    "Possible fire detected. Pausing patrol for confirmation "
                    "(%.2fs in %.2fs).",
                    stop_seen,
                    self.stop_window_sec,
                )
                self._publish_state(True, False)
                self.cmd_vel_pub.publish(Twist())
                self._cancel_move_base(now)

        elif self.state == "confirming":
            if alert_seen >= self.alert_required_sec:
                self.state = "alert"
                self.last_alert_time = now
                rospy.logerr(
                    "FIRE ALERT: %.2fs detected in %.2fs.",
                    alert_seen,
                    self.alert_window_sec,
                )
                self.alert_pub.publish("화재 발생!!!")
            elif clear_seen <= self.clear_max_sec:
                self.state = "normal"
                rospy.loginfo("Fire candidate cleared. Resuming patrol.")
                self.alert_pub.publish("화재 의심 해제")

        elif self.state == "alert":
            if now - self.last_alert_time >= self.alert_repeat_interval:
                self.last_alert_time = now
                self.alert_pub.publish("화재 발생!!!")

            if clear_seen <= self.clear_max_sec:
                self.state = "normal"
                rospy.loginfo("Fire alert cleared. Resuming patrol.")
                self.alert_pub.publish("화재 경보 해제")

        paused = self.state in ("confirming", "alert")
        alert_active = self.state == "alert"
        if paused:
            self.cmd_vel_pub.publish(Twist())
            if now - self.last_cancel_time >= rospy.Duration(1.0):
                self._cancel_move_base(now)
        self._publish_state(paused, alert_active)

    def _cancel_move_base(self, now):
        self.last_cancel_time = now
        self.move_base_cancel_pub.publish(GoalID())

    def _publish_state(self, paused, alert_active):
        self.pause_pub.publish(Bool(data=paused))
        self.alert_active_pub.publish(Bool(data=alert_active))
        self.last_pause = paused
        self.last_alert_active = alert_active
        self.state_pub.publish(String(data=self.state))


def main():
    rospy.init_node("fire_response_node")
    FireResponseNode()
    rospy.spin()


if __name__ == "__main__":
    main()
