-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

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
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  -- use_pose_extrapolator = true,
  use_nav_sat = false,
  use_landmarks = true,
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
  --max_submaps_to_keep = 3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 12


--Localization
-- TRAJECTORY_BUILDER.pure_localization = true
-- TRAJECTORY_BUILDER.pure_localization_trimmer.max_submaps_to_keep = 3
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 1,
-- }
-- TRAJECTORY_BUILDER_2D.submaps.num_range_data = 1000
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 15.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.01
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window =math.rad(5.0)

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.67
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 100
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 100

-- TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.01
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)

POSE_GRAPH.constraint_builder.min_score = 0.65
-- PÓSE_GRAPH.constraint_builder.sampling_ratio = 0.03 -- 0.3
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

POSE_GRAPH.optimize_every_n_nodes = 10 --  10
-- POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e5
-- POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e9
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 100
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 200
-- POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
-- POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e4
-- POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e6
-- POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e7
-- POSE_GRAPH.global_sampling_ratio = 3e4 -- 0.003
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 10e9
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations=3 -- 5
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60 -- 150

return options
