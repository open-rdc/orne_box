-- Cartographerの設定ファイル。各パラメータはSLAMの動作を調整します。

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  -- 基本設定
  map_builder = MAP_BUILDER,  -- マップビルダー設定の選択
  trajectory_builder = TRAJECTORY_BUILDER,  -- 軌道ビルダー設定の選択
  map_frame = "map",  -- マップフレームの名前
  tracking_frame = "base_link",  -- ロボットの基準フレーム（位置追跡用）
  published_frame = "base_footprint",  -- 公開されるフレーム
  odom_frame = "odom",  -- オドメトリフレームの名前
  provide_odom_frame = true,  -- オドメトリフレームを提供するかどうか
  publish_frame_projected_to_2d = false,  -- フレームを2Dに投影するか
  use_pose_extrapolator = true,  -- 姿勢推定の外挿器を使用するか
  use_odometry = true,  -- オドメトリを使用するか
  use_nav_sat = false,  -- ナビゲーション用衛星データを使用するか
  use_landmarks = false,  -- ランドマークを使用するか
  num_laser_scans = 1,  -- 使用するレーザースキャンの数
  num_multi_echo_laser_scans = 0,  -- 使用するマルチエコーレーザースキャンの数
  num_subdivisions_per_laser_scan = 1,  -- レーザースキャンの細分化数
  num_point_clouds = 0,  -- 使用するポイントクラウドの数
  lookup_transform_timeout_sec = 0.5,  -- トランスフォームの検索タイムアウト（秒）。短くすると失敗しやすく、長くすると遅延が増加します。
  submap_publish_period_sec = 0.5,  -- サブマップの公開頻度（秒）。短くすると最新情報が得られやすいが、通信量が増加します。
  pose_publish_period_sec = 5e-3,  -- 姿勢の公開頻度（秒）。頻度を高くすると追跡がスムーズだが、計算負荷が増えます。
  trajectory_publish_period_sec = 30e-3,  -- 軌道の公開頻度（秒）。高くするとリアルタイム性が向上しますが、CPU負荷が増加します。
  rangefinder_sampling_ratio = 1.0,  -- レンジファインダーのサンプリング割合。低くすると計算負荷が減少するが、精度が落ちる。
  odometry_sampling_ratio = 1.0,  -- オドメトリのサンプリング割合。同上。
  fixed_frame_pose_sampling_ratio = 1.0,  -- 固定フレームのサンプリング割合。同上。
  imu_sampling_ratio = 1.0,  -- IMUのサンプリング割合。低くすると負荷軽減。
  landmarks_sampling_ratio = 1.0,  -- ランドマークのサンプリング割合。同上。
}

-- ---------------------------------------------------------------------------------------------
-- GLOBAL SLAM設定
-- ---------------------------------------------------------------------------------------------

POSE_GRAPH.optimize_every_n_nodes = 50
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 15
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 1.1e4
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.log_matches = true

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 7
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(30)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 10

POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 90
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 50
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 18

POSE_GRAPH.matcher_translation_weight = 5e2
POSE_GRAPH.matcher_rotation_weight = 1.6e3

POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1.4e3
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_translation_weight = 4.0e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 3.0e4
POSE_GRAPH.optimization_problem.fixed_frame_pose_translation_weight = 1e1
POSE_GRAPH.optimization_problem.fixed_frame_pose_rotation_weight = 1e2

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.max_num_final_iterations = 200
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10

-- ---------------------------------------------------------------------------------------------
-- LOCAL SLAM設定
-- ---------------------------------------------------------------------------------------------

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 0.5
TRAJECTORY_BUILDER_2D.max_range = 200 
TRAJECTORY_BUILDER_2D.min_z = -0.1
TRAJECTORY_BUILDER_2D.max_z = 30
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20)
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 90
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2

-- ---------------------------------------------------------------------------------------------
-- その他の重要な設定
-- ---------------------------------------------------------------------------------------------

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 18

return options
