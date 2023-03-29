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

class realsense_operator {
  private:
    std::string_view bag_file_path;

    rs2::pipeline pipeline;
    rs2::config config;
    rs2::pointcloud pc;

    std::string_view splitFileSavePath = {"rs_split"};
    bool saveSplitFiles = true;
    int captureInterval = 30;
    int captureCount = 30;

    bool runPassthroughFilter = false;
    float passthroughFilterXMin = 0.0f;
    float passthroughFilterXMax = 0.0f;
    float passthroughFilterYMin = 0.0f;
    float passthroughFilterYMax = 0.0f;
    float passthroughFilterZMin = 0.0f;
    float passthroughFilterZMax = 0.0f;

  public:
    explicit realsense_operator(std::string_view bag_file_path);

    void split_pointclouds();
    void split_pointclouds(const std::function<void(pcl_cloud)> &callback);

    void setSplitFileSavePath(std::string_view v);
    void setSaveSplitFiles(bool v);
    void setCaptureInterval(int v);
    void setCaptureCount(int v);
    void setRunPassthroughFilter(bool v);
    void setPassthroughFilterX(float min, float max);
    void setPassthroughFilterY(float min, float max);
    void setPassthroughFilterZ(float min, float max);
};

} // namespace tdr
#endif // INC_3D_RECONSTRUCTION_REALSENSE_OPERATOR_HPP
