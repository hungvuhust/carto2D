include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  -- use_pose_graph = false,
  -- use_local_slam = true,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,

}

MAP_BUILDER.use_trajectory_builder_2d = true


--Localization
-- TRAJECTORY_BUILDER.pure_localization = true
-- TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep = 3
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 1,
-- }
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 1000
TRAJECTORY_BUILDER_2D.min_range = 0.04
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =math.rad(2.0)
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 100
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 100

-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.01
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.50 -- 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7


-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e5
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e9
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 100
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 200
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e4
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e6
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e7
-- POSE_GRAPH.optimization_problem.global_sampling_ratio = 0.3
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10e9 -- 10e9
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations=10 -- 5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false -- true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}

POSE_GRAPH.optimize_every_n_nodes = 10

return options