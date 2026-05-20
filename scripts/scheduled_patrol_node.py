#!/usr/bin/env python3
import datetime
import shlex

import roslaunch
import rospy
from std_msgs.msg import Bool


def _param_as_launch_value(name):
    value = rospy.get_param("~" + name)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _parse_time(value, param_name):
    try:
        parts = [int(part) for part in str(value).strip().split(":")]
    except ValueError:
        parts = []

    if len(parts) != 2:
        raise rospy.ROSInitException(
            "%s must use HH:MM format, for example 22:00." % param_name
        )

    hour, minute = parts
    if not (0 <= hour <= 23 and 0 <= minute <= 59):
        raise rospy.ROSInitException(
            "%s is out of range. Use HH:MM in 24-hour local time." % param_name
        )

    return datetime.time(hour=hour, minute=minute)


def _time_to_minutes(value):
    return value.hour * 60 + value.minute


def _parse_launch_args(value):
    args = shlex.split(str(value).strip()) if str(value).strip() else []
    values = {}
    for arg in args:
        if ":=" in arg:
            name, val = arg.split(":=", 1)
            values[name] = val
    return args, values


def _with_launch_arg(args, name, value):
    prefix = name + ":="
    return [arg for arg in args if not arg.startswith(prefix)] + [prefix + str(value)]


class ScheduledPatrolNode:
    def __init__(self):
        self.start_time = _parse_time(rospy.get_param("~start_time", "22:00"), "start_time")
        self.end_time = _parse_time(rospy.get_param("~end_time", "06:00"), "end_time")
        self.check_period_sec = rospy.get_param("~check_period_sec", 5.0)
        if self.check_period_sec <= 0.0:
            rospy.logwarn("check_period_sec must be positive. Falling back to 5.0 seconds.")
            self.check_period_sec = 5.0
        self.pause_before_shutdown_sec = rospy.get_param("~pause_before_shutdown_sec", 1.0)
        self.graceful_stop_timeout_sec = rospy.get_param("~graceful_stop_timeout_sec", 600.0)
        self.publish_pause_on_idle = rospy.get_param("~publish_pause_on_idle", True)
        max_run_minutes = rospy.get_param(
            "~max_run_minutes", rospy.get_param("~run_duration_minutes", 0.0)
        )
        self.max_run_duration = datetime.timedelta(
            minutes=max(0.0, max_run_minutes)
        )
        self.rest_after_run = datetime.timedelta(
            minutes=max(0.0, rospy.get_param("~rest_minutes_after_run", 0.0))
        )
        self.preload_gazebo = rospy.get_param("~preload_gazebo", False)
        self.patrol_cli_args, self.patrol_arg_values = _parse_launch_args(
            rospy.get_param("~patrol_launch_args", "")
        )

        self.parent = None
        self.gazebo_parent = None
        self.uuid = None
        self.gazebo_uuid = None
        self.run_started_at = None
        self.next_allowed_start = None
        self.stop_requested = False
        self.stop_requested_at = None
        self.patrol_finished = False
        self.pause_pub = rospy.Publisher("/patrol_pause", Bool, queue_size=1, latch=True)
        self.stop_request_pub = rospy.Publisher(
            "/patrol_stop_requested", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/patrol_finished", Bool, self._patrol_finished_callback, queue_size=1)

        rospy.on_shutdown(self.shutdown)
        if self.preload_gazebo:
            self.start_gazebo()

        rospy.loginfo(
            "Scheduled patrol ready: start=%s end=%s max_run=%.1fmin rest_after_run=%.1fmin preload_gazebo=%s check_period=%.1fs",
            self.start_time.strftime("%H:%M"),
            self.end_time.strftime("%H:%M"),
            self.max_run_duration.total_seconds() / 60.0,
            self.rest_after_run.total_seconds() / 60.0,
            self.preload_gazebo,
            self.check_period_sec,
        )

    def _should_patrol_now(self, now_time):
        start = _time_to_minutes(self.start_time)
        end = _time_to_minutes(self.end_time)
        now = _time_to_minutes(now_time)

        if start == end:
            return True
        if start < end:
            return start <= now < end
        return now >= start or now < end

    def _patrol_finished_callback(self, msg):
        self.patrol_finished = bool(msg.data)

    def run(self):
        rate = rospy.Rate(1.0 / self.check_period_sec)
        while not rospy.is_shutdown():
            self._check_schedule()
            rate.sleep()

    def _check_schedule(self):
        now = datetime.datetime.now()
        should_run = self._should_patrol_now(now.time())

        if self.parent is not None:
            if not should_run:
                self.request_patrol_stop(
                    "outside scheduled patrol window (now %s, window %s-%s)"
                    % (
                        now.strftime("%H:%M"),
                        self.start_time.strftime("%H:%M"),
                        self.end_time.strftime("%H:%M"),
                    )
                )
            elif self.max_run_duration.total_seconds() > 0.0 and now - self.run_started_at >= self.max_run_duration:
                self.request_patrol_stop("scheduled max run time elapsed")

            if self.stop_requested and self.patrol_finished:
                self.stop_patrol("patrol finished after scheduled stop request", pause_first=False)
            elif (
                self.stop_requested
                and self.graceful_stop_timeout_sec > 0.0
                and (now - self.stop_requested_at).total_seconds() >= self.graceful_stop_timeout_sec
            ):
                self.stop_patrol("graceful stop timeout elapsed", pause_first=True)
            return

        if (
            should_run
            and (self.next_allowed_start is None or now >= self.next_allowed_start)
        ):
            self.start_patrol()
        elif self.publish_pause_on_idle:
            self.pause_pub.publish(Bool(data=True))

    def start_gazebo(self):
        if self.gazebo_parent is not None:
            return

        rospy.loginfo("Preloading Gazebo before scheduled patrol.")
        self.pause_pub.publish(Bool(data=True))

        self.gazebo_uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.gazebo_uuid)
        launch_file = roslaunch.rlutil.resolve_launch_arguments(
            ["night_patrol_robot", "gazebo_robot.launch"]
        )[0]
        gazebo_arg_names = ["world_name", "spawn_x", "spawn_y", "spawn_z", "spawn_yaw"]
        cli_args = [
            "%s:=%s" % (name, self.patrol_arg_values[name])
            for name in gazebo_arg_names
            if name in self.patrol_arg_values
        ]
        self.gazebo_parent = roslaunch.parent.ROSLaunchParent(
            self.gazebo_uuid, [(launch_file, cli_args)]
        )
        self.gazebo_parent.start()

    def start_patrol(self):
        rospy.loginfo("Starting scheduled patrol.")
        self.stop_requested = False
        self.stop_requested_at = None
        self.patrol_finished = False
        self.stop_request_pub.publish(Bool(data=False))
        self.pause_pub.publish(Bool(data=False))
        self.run_started_at = datetime.datetime.now()

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        launch_file = roslaunch.rlutil.resolve_launch_arguments(
            ["night_patrol_robot", "patrol_one_button.launch"]
        )[0]
        cli_args = list(self.patrol_cli_args)
        if self.preload_gazebo:
            cli_args = _with_launch_arg(cli_args, "use_gazebo", "false")

        self.parent = roslaunch.parent.ROSLaunchParent(self.uuid, [(launch_file, cli_args)])
        self.parent.start()

    def request_patrol_stop(self, reason):
        if self.stop_requested:
            return

        rospy.loginfo("Requesting scheduled patrol stop: %s.", reason)
        self.stop_requested = True
        self.stop_requested_at = datetime.datetime.now()
        self.stop_request_pub.publish(Bool(data=True))

    def stop_patrol(self, reason, pause_first=True):
        rospy.loginfo("Stopping scheduled patrol: %s.", reason)
        if pause_first:
            self.pause_pub.publish(Bool(data=True))
        if pause_first and self.pause_before_shutdown_sec > 0.0:
            rospy.sleep(self.pause_before_shutdown_sec)

        if self.parent is not None:
            self.parent.shutdown()
            self.parent = None
            self.uuid = None
            self.run_started_at = None
            self.stop_requested = False
            self.stop_requested_at = None
            self.patrol_finished = False
            self.stop_request_pub.publish(Bool(data=False))
            if self.rest_after_run.total_seconds() > 0.0:
                self.next_allowed_start = datetime.datetime.now() + self.rest_after_run

    def shutdown(self):
        if self.parent is not None:
            self.stop_patrol("scheduler shutdown")
        if self.gazebo_parent is not None:
            self.gazebo_parent.shutdown()
            self.gazebo_parent = None
            self.gazebo_uuid = None
        elif self.publish_pause_on_idle:
            self.pause_pub.publish(Bool(data=True))


def main():
    rospy.init_node("scheduled_patrol_node")
    try:
        ScheduledPatrolNode().run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
