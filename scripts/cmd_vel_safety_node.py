#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class CmdVelSafetyNode:
    def __init__(self):
        self.paused = False
        self.last_cmd = None
        self.last_cmd_time = rospy.Time(0)
        self.model_pose = None
        self.last_model_brake_time = rospy.Time(0)

        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_nav")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.pause_topic = rospy.get_param("~pause_topic", "/patrol_pause")
        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 30.0)
        self.stale_timeout = rospy.Duration(rospy.get_param("~stale_timeout_sec", 0.5))
        self.hard_brake_enabled = rospy.get_param("~hard_brake_enabled", True)
        self.hard_brake_model = rospy.get_param("~hard_brake_model", "turtlebot3_waffle_pi")
        self.hard_brake_rate_hz = rospy.get_param("~hard_brake_rate_hz", 10.0)
        self.hard_brake_interval = rospy.Duration(1.0 / self.hard_brake_rate_hz)

        self.cmd_pub = rospy.Publisher(self.output_topic, Twist, queue_size=1)
        rospy.Subscriber(self.input_topic, Twist, self._cmd_callback, queue_size=1)
        rospy.Subscriber(self.pause_topic, Bool, self._pause_callback, queue_size=1)
        self.set_model_state = None
        if self.hard_brake_enabled:
            rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1)
            self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._timer_callback
        )

        rospy.loginfo(
            "cmd_vel safety gate ready: %s -> %s, pause=%s",
            self.input_topic,
            self.output_topic,
            self.pause_topic,
        )

    def _model_states_callback(self, msg):
        try:
            index = msg.name.index(self.hard_brake_model)
        except ValueError:
            return
        self.model_pose = msg.pose[index]

    def _cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()
        if not self.paused:
            self.cmd_pub.publish(msg)

    def _pause_callback(self, msg):
        paused = bool(msg.data)
        if paused and not self.paused:
            rospy.logwarn("cmd_vel safety gate locked by patrol pause.")
            self.cmd_pub.publish(Twist())
        elif not paused and self.paused:
            rospy.loginfo("cmd_vel safety gate unlocked.")
        self.paused = paused

    def _timer_callback(self, _event):
        if self.paused:
            self.cmd_pub.publish(Twist())
            self._hard_brake_robot()
            return

        if self.last_cmd is None:
            return

        if rospy.Time.now() - self.last_cmd_time > self.stale_timeout:
            self.cmd_pub.publish(Twist())
            self.last_cmd = None

    def _hard_brake_robot(self):
        if (
            not self.hard_brake_enabled
            or self.set_model_state is None
            or self.model_pose is None
        ):
            return

        now = rospy.Time.now()
        if now - self.last_model_brake_time < self.hard_brake_interval:
            return

        state = ModelState()
        state.model_name = self.hard_brake_model
        state.pose = self.model_pose
        state.twist = Twist()
        state.reference_frame = "world"
        try:
            self.set_model_state(state)
            self.last_model_brake_time = now
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(5.0, "Failed to hard-brake Gazebo model: %s", exc)


def main():
    rospy.init_node("cmd_vel_safety_node")
    CmdVelSafetyNode()
    rospy.spin()


if __name__ == "__main__":
    main()
