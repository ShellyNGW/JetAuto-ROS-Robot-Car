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

include "map_builder.lua"
include "trajectory_builder.lua"

master_name = os.getenv("MASTER")
robot_name = os.getenv("HOST")
prefix = os.getenv("prefix")
MAP_FRAME = "map"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
if(prefix ~= "/")
then
    MAP_FRAME = master_name .. "/" .. MAP_FRAME
    ODOM_FRAME = robot_name .. "/" .. ODOM_FRAME
    BASE_FRAME = robot_name .. "/" .. BASE_FRAME
end
options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息(configuration information of map_builder.lua)
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息(configuration information of trajectory_builder.lua)
  
  map_frame = MAP_FRAME,        -- 地图坐标系的名字(map coordinate system name)
  tracking_frame = BASE_FRAME,  -- 将所有传感器数据转换到这个坐标系下(transfer all the sensor data to this coordinate system)
  published_frame = ODOM_FRAME, -- tf: map -> odom
  odom_frame = ODOM_FRAME,      -- 里程计的坐标系名字(odometer coordinate system name)
  provide_odom_frame = false,   -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint(whether tf of odom is provided. If true, tf tree is map->odom->footprint)
                                            -- 如果为false tf树为map->footprint(if false, tf tree is map->footprint)
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上(whether to map the coordinate system on the plane)

  use_odometry = true,                      -- 是否使用里程计,如果使用要求一定要有odom的tf(whether to use odometer. tf of odom is required when in use)
  use_nav_sat = false,                      -- 是否使用gps(whether to use gps)
  use_landmarks = false,                    -- 是否使用landmark(whether to use landmark)
  num_laser_scans = 1,                      -- 是否使用单线激光数据(whether to use single-line laser data)
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据(whether to use multi_echo_laser_scans data)
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1(one frame of data is divided into several parts for processing. In general, it is set as 1)
  num_point_clouds = 0,                     -- 是否使用点云数据(whether to use point cloud)
  
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间(find tf timeout)
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔(interval between each data publishing)
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率(sampling frequency of sensor data)
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 10.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.min_score = 0.65

return options
