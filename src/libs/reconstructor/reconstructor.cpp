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

#include "reconstructor.hpp"
#include "../../utils/pointcloud_util.hpp"
#include <librealsense2/rs.hpp>
#include <pcl/io/ply_io.h>

tdr::reconstructor::reconstructor(::std::string_view bag_file_path) {
    this->bag_file_path = bag_file_path;
}

void tdr::reconstructor::run() {
    rs2::config rs_config;

    rs_config.enable_device_from_file(this->bag_file_path.data());
    rs_config.enable_stream(
        rs2_stream::RS2_STREAM_DEPTH, rs2_format::RS2_FORMAT_Z16, 30);

    rs2::pipeline rs_pipeline;

    spdlog::info("Starting RealSense pipeline");
    rs_pipeline.start(rs_config);

    rs2::pointcloud rs_pointcloud;
    rs2::points rs_points;

    auto frame = rs_pipeline.wait_for_frames();
    auto depth = frame.get_depth_frame();

    rs_points = rs_pointcloud.calculate(depth);

    auto pcl_pc = tdr::rs2_points_to_pcl(rs_points);
}
