
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


---------------------------------------------------
--三次元測域センサでpcdの三次元地図を作成する設定--
---------------------------------------------------


VOXEL_SIZE = 0.1

include "transform.lua"

options = {

  --pbstream作成時に指定したtracking_frameと同じものを指定
  tracking_frame = "imu",


  pipeline = {
    {
      --地図書き出し時に使用する測距値の範囲を指定
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 100.,
    },
    {
      --地図書き出し時に使用する測距値の範囲を指定
      action = "vertical_range_filter",
      min_z = 0.0,
      max_z = 3.0
    },
    {
      --移動している物体の点群を削除 
      action = "voxel_filter_and_remove_moving_objects",
      voxel_size = VOXEL_SIZE,
    },
    {
      action = "dump_num_points",
    },

    {
      action = "intensity_to_color",
      min_intensity = 0.,
      max_intensity = 127.,
    },

    -- Gray X-Rays. These only use geometry to color pixels.
    -- {
    --   action = "write_xray_image",
    --   voxel_size = VOXEL_SIZE,
    --   filename = "xray_yz_all",
    --   transform = YZ_TRANSFORM,
    -- },
    -- {
    --   action = "write_xray_image",
    --   voxel_size = VOXEL_SIZE,
    --   filename = "xray_xy_all",
    --   transform = XY_TRANSFORM,
    -- },
    -- {
    --   action = "write_xray_image",
    --   voxel_size = VOXEL_SIZE,
    --   filename = "xray_xz_all",
    --   transform = XZ_TRANSFORM,
    -- },

    -- Now we recolor our points by frame and write another batch of X-Rays. It
    -- is visible in them what was seen by the horizontal and the vertical
    -- laser.
    -- {
    --   action = "color_points",
    --   frame_id = "horizontal_vlp16_link",
    --   color = { 255., 0., 0. },
    -- },
    -- {
    --   action = "color_points",
    --   frame_id = "vertical_vlp16_link",
    --   color = { 0., 255., 0. },
    -- },

    -- {
    --   action = "write_xray_image",
    --   voxel_size = VOXEL_SIZE,
    --   filename = "xray_xy_all_color",
    --   transform = XY_TRANSFORM,
    -- },

    {
      --pcd形式で出力
      action = "write_pcd",
      filename = "cartographer.pcd",
    },

    {
      action = "write_probability_grid",
      draw_trajectories = false,
      resolution = 0.1,
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      filename = "probability_grid",
    },
  }
}

return options
