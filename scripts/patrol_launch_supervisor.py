#!/usr/bin/env python3
import os

import roslaunch
import rospy


def _param_as_launch_value(name):
    value = rospy.get_param("~" + name)
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


class PatrolLaunchSupervisor:
    def __init__(self):
        self.parent = None
        self.map_file = rospy.get_param("~map_file")
        requested_mapping = str(rospy.get_param("~mapping", "auto")).lower()

        if requested_mapping == "auto":
            mapping = not os.path.exists(self.map_file)
            reason = "map file missing" if mapping else "map file exists"
        elif requested_mapping in ("true", "1", "yes"):
            mapping = True
            reason = "forced by mapping:=true"
        elif requested_mapping in ("false", "0", "no"):
            mapping = False
            reason = "forced by mapping:=false"
        else:
            raise rospy.ROSInitException(
                "Invalid mapping value '%s'. Use auto, true, or false." % requested_mapping
            )

        self.mapping_value = "true" if mapping else "false"
        rospy.loginfo("Patrol launch mode: mapping=%s (%s)", self.mapping_value, reason)

    def start(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = roslaunch.rlutil.resolve_launch_arguments(
            ["night_patrol_robot", "patrol_runtime.launch"]
        )[0]

        arg_names = [
            "map_output",
            "map_file",
            "patrol_loop",
            "return_home",
            "patrol_wait_sec",
            "use_rviz",
            "use_camera_viewer",
            "use_fire_detection",
            "camera_view_topic",
            "mapping_strategy",
            "rviz_config",
            "world_name",
            "frontier_goal_timeout_sec",
            "frontier_min_goal_distance",
            "frontier_max_goal_distance",
            "frontier_blacklist_radius",
            "frontier_area_blacklist_radius",
            "frontier_blacklist_ttl_sec",
            "frontier_blacklist_max_size",
            "frontier_clearance_cells",
            "frontier_viewpoint_clearance_cells",
            "frontier_viewpoint_require_known_space",
            "frontier_occupied_threshold",
            "frontier_information_radius_cells",
            "frontier_obstacle_penalty_radius_cells",
            "frontier_information_gain_weight",
            "frontier_distance_weight",
            "frontier_obstacle_penalty_weight",
            "frontier_min_cluster_size",
            "frontier_size_weight",
            "frontier_max_score_distance",
            "frontier_prefer_nearest",
            "frontier_nearest_score_tolerance",
            "frontier_local_radius",
            "frontier_local_bonus",
            "exploration_complete_wait_sec",
            "frontier_viewpoint_min_distance_cells",
            "frontier_viewpoint_max_distance_cells",
            "frontier_max_goal_failures_before_blacklist",
            "frontier_suppress_reached_frontier",
            "frontier_completion_spin_attempts",
            "frontier_completion_spin_angular_speed",
            "frontier_minimum_mapping_time_sec",
            "frontier_progress_timeout_sec",
            "frontier_free_cell_growth_threshold",
            "frontier_min_suppressed_for_completion",
            "frontier_force_completion_after_repeated_failures",
            "spawn_x",
            "spawn_y",
            "spawn_z",
            "spawn_yaw",
            "initial_pose_x",
            "initial_pose_y",
            "initial_pose_yaw",
        ]
        cli_args = ["mapping:=%s" % self.mapping_value]
        cli_args.extend("%s:=%s" % (name, _param_as_launch_value(name)) for name in arg_names)

        self.parent = roslaunch.parent.ROSLaunchParent(uuid, [(launch_file, cli_args)])
        self.parent.start()
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        if self.parent is not None:
            self.parent.shutdown()


def main():
    rospy.init_node("patrol_launch_supervisor")
    supervisor = PatrolLaunchSupervisor()
    supervisor.start()
    rospy.spin()


if __name__ == "__main__":
    main()
