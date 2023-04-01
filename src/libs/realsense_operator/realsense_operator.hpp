/*
 * Copyright (C) 2023 Alisa.
 *
 * This file is part of 3d_reconstruction.
 *
 * 3d_reconstruction is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * 3d_reconstruction is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 */

#ifndef INC_3D_RECONSTRUCTION_REALSENSE_OPERATOR_HPP
#define INC_3D_RECONSTRUCTION_REALSENSE_OPERATOR_HPP

#include "../../utils/pcl_type_definition.hpp"
#include <functional>
#include <librealsense2/rs.hpp>
#include <string_view>

namespace tdr {

struct realsense_operator_configuration {
    std::string bag_file_path{};
    std::string split_file_save_path{"rs_split"};
    int capture_interval = 30;
    int capture_count = 30;

    bool run_passthrough_filter = false;
    float passthrough_filter_x_min = -0.5f;
    float passthrough_filter_x_max = 0.5f;
    float passthrough_filter_y_min = -0.5f;
    float passthrough_filter_y_max = 0.5f;
    float passthrough_filter_z_min = -0.5f;
    float passthrough_filter_z_max = 0.5f;
};

class realsense_operator {
  private:
    realsense_operator_configuration config;

    rs2::pipeline rs2_pipeline;
    rs2::config rs2_config;
    rs2::pointcloud rs2_pc;

  public:
    explicit realsense_operator(const realsense_operator_configuration &config);

    void split_pointclouds();
};

} // namespace tdr
#endif // INC_3D_RECONSTRUCTION_REALSENSE_OPERATOR_HPP
