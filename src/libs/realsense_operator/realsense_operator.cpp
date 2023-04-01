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

#include "realsense_operator.hpp"

#include "../../utils/fs_utils.hpp"
#include "../../utils/pointcloud_util.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

pcl_cloud run_filter(const pcl_cloud &src, std::string_view fn, float min,
                     float max);

tdr::realsense_operator::realsense_operator(
    const realsense_operator_configuration &config) {

    this->config = config;

    this->rs2_config.enable_device_from_file(this->config.bag_file_path.data());
    this->rs2_config.enable_stream(rs2_stream::RS2_STREAM_DEPTH);
}

void tdr::realsense_operator::split_pointclouds() {

    tdr::utils::fs::ensure_directory_empty(this->config.split_file_save_path);

    int current_frame = 1;
    int current_split_count = 1;

    this->rs2_pipeline.start(this->rs2_config);

    bool has_frame = true;
    bool continue_capture = true;

    while (has_frame && continue_capture) {
        rs2::frameset frame;

        has_frame = this->rs2_pipeline.try_wait_for_frames(&frame);

        if (has_frame) {

            spdlog::debug("Split {:0>3}, Frame: {:0>2}",
                          current_split_count,
                          current_frame);

            if (current_frame == 1) {
                spdlog::info("Capture split {:0>3}...", current_split_count);

                auto depth = frame.get_depth_frame();
                auto points = this->rs2_pc.calculate(depth);

                auto cloud = tdr::utils::points::rs2_points_to_pcl(points);

                // Filter
                if (this->config.run_passthrough_filter) {
                    spdlog::info("Running passthrough filter...");
                    cloud = run_filter(cloud,
                                       "x",
                                       this->config.passthrough_filter_x_min,
                                       this->config.passthrough_filter_x_max);
                    cloud = run_filter(cloud,
                                       "y",
                                       this->config.passthrough_filter_y_min,
                                       this->config.passthrough_filter_y_max);
                    cloud = run_filter(cloud,
                                       "z",
                                       this->config.passthrough_filter_z_min,
                                       this->config.passthrough_filter_z_max);
                    spdlog::info("Filter done.");
                }

                // Save
                std::stringstream fn;
                fn << this->config.split_file_save_path.data() << "/"
                   << fmt::format("{:0>3}", current_split_count) << ".pcd";
                spdlog::info("Saving split {:0>3} to {}...",
                             current_split_count,
                             fn.str());
                pcl::io::savePCDFile(fn.str(), *cloud, true);

                current_split_count++;
            }

            current_frame++;
            if (current_frame > this->config.capture_interval) {
                current_frame = 1;
            }

            continue_capture =
                current_split_count <= this->config.capture_count;
        } else {
            spdlog::warn("No more frames to capture.");
        }
    }

    this->rs2_pipeline.stop();
}

pcl_cloud run_filter(const pcl_cloud &src, std::string_view fn, float min,
                     float max) {

    spdlog::debug("Running passthrough filter for {}, min: {}, max: {}",
                  fn.data(),
                  min,
                  max);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl_cloud filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(src);
    pass.setFilterFieldName(fn.data());
    pass.setFilterLimits(min, max);
    pass.filter(*filtered_cloud);

    return filtered_cloud;
}
