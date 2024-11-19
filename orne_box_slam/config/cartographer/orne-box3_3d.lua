include "map_builder.lua"
include "trajectory_builder.lua"

-- options テーブルの設定
options = {
  map_builder = MAP_BUILDER,  -- マップビルダーの設定（2Dまたは3D）
  trajectory_builder = TRAJECTORY_BUILDER,  -- トラジェクトリービルダーの設定
  map_frame = "map",  -- 地図の座標フレーム
  tracking_frame = "imu",  -- IMUの座標フレーム（位置追跡用）
  published_frame = "base_link",  -- 出力されるフレームの座標
  odom_frame = "odom",  -- オドメトリ座標フレーム
  provide_odom_frame = true,  -- オドメトリフレームを提供するか（falseにするとオドメトリの影響を無視）
  publish_frame_projected_to_2d = false,  -- 3D座標フレームを2Dに投影するか
  use_pose_extrapolator = true,  -- ポーズの補間機能を使うか
  use_odometry = false,  -- オドメトリを使用するか（trueにすると精度向上が見込める）
  use_nav_sat = false,  -- GNSSデータの利用可否（trueにすると衛星データを使用）
  use_landmarks = false,  -- ランドマークデータの利用可否
  num_laser_scans = 0,  -- レーザースキャンの数
  num_multi_echo_laser_scans = 0,  -- マルチエコーレーザースキャンの数
  num_subdivisions_per_laser_scan = 1,  -- 各レーザースキャンの分割数（高いほど詳細なデータ取得）
  num_point_clouds = 1,  -- 点群データの数
  lookup_transform_timeout_sec = 0.2,  -- 変換を見つけるまでのタイムアウト時間（短いと変換待ちが少なくなる）
  submap_publish_period_sec = 0.3,  -- サブマップの公開間隔（小さくすると頻繁に公開）
  pose_publish_period_sec = 5e-3,  -- ポーズの公開間隔（小さくするとリアルタイムに近い位置情報を提供）
  trajectory_publish_period_sec = 30e-3,  -- トラジェクトリーの公開間隔
  rangefinder_sampling_ratio = 1.0,  -- 距離計データのサンプリング割合（0にすると距離計を無視）
  odometry_sampling_ratio = 1.0,  -- オドメトリデータのサンプリング割合（高くすると精度向上が期待される）
  fixed_frame_pose_sampling_ratio = 1.0,  -- 固定フレームのポーズサンプリング割合（低いと計算が減る）
  imu_sampling_ratio = 1.0,  -- IMUデータのサンプリング割合（高いほど精度が上がるが計算負荷も増える）
  landmarks_sampling_ratio = 1.0,  -- ランドマークデータのサンプリング割合（低くするとランドマーク無視）
}

------------------------------------------------------------------------------------------------
--LOCAL SLAM SETTING
------------------------------------------------------------------------------------------------

--Map Builder settings
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 15

--TRAJECTORY BUILDER 3D MAIN SETTING
TRAJECTORY_BUILDER_3D.min_range = 1.
TRAJECTORY_BUILDER_3D.max_range = 200.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 9.81
--TRAJECTORY_BUILDER_3D.num_odometry_states = 1

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 5.
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 8.
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 400.

--Motion filter settings
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.004
 
--ONLINE CORRELATIVE SCAN MATCHING SETTING
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = 0.610865238 --1deg=0.01745329
--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

--CERES SCAN MATCHER SETTINGS
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10 --Cost of translating a new scan from the submap.
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4 --Cost of rotating a new scan from the submap. Default=4e2, reccommended=2e1
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.0
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.0
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nomonotonic_steps = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 40
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

--TRAJECTORY BUILDER SUBMAP SETTING
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.25
TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 80   --Efectively the 'size' of each submap.
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability = 0.49
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.num_free_space_voxels = 2


-----------------------------------------------------------------------------------------------
--GLOBAL SLAM SETTING
-----------------------------------------------------------------------------------------------

--Global Slam Basic Setting
POSE_GRAPH.optimize_every_n_nodes = 110
POSE_GRAPH.matcher_rotation_weight = 1.6e3
POSE_GRAPH.matcher_translation_weight = 5e2
--POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.001 

--MAP_BUILDER.pose_graph.constraint_builder.adaptive_voxel_filter = TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter

--Global SLAM constraint builder options
POSE_GRAPH.constraint_builder.min_score = 0.5
POSE_GRAPH.constraint_builder.sampling_ratio = 0.005
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1e3
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e7
POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.constraint_builder.max_constraint_distance= 100.

--Global SLAM optimization settings
POSE_GRAPH.optimization_problem.huber_scale = 3e2
POSE_GRAPH.optimization_problem.acceleration_weight = 750
POSE_GRAPH.optimization_problem.rotation_weight = 140
POSE_GRAPH.optimization_problem.log_solver_summary = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
--POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 1

--GLOBAL SLAM CERES SCAN Matcher
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 3
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
--POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 7

--Global SLAM Ceres 3D scan matcher
--POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 5.0
--POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 30.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 1

--fast correlative scan matcher
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 8
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 3
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.min_rotational_score = 0.77
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 30.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 10.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(60.)

POSE_GRAPH.global_constraint_search_after_n_seconds = 25.

return options