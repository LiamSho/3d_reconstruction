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
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

tdr::realsense_operator::realsense_operator(::std::string_view bag_file_path) {
    this->bag_file_path = bag_file_path;

    this->config.enable_device_from_file(this->bag_file_path.data());
    this->config.enable_stream(rs2_stream::RS2_STREAM_DEPTH);
}

void tdr::realsense_operator::split_pointclouds() {
    this->split_pointclouds(nullptr);
}

void tdr::realsense_operator::split_pointclouds(
    const std::function<void(pcl_cloud)> &callback) {

    if (this->saveSplitFiles) {
        tdr::utils::fs::ensure_directory_empty(this->splitFileSavePath);
    }

    int current_frame = 1;
    int current_split_count = 1;

    this->pipeline.start(this->config);

    bool has_frame = true;
    bool continue_capture = true;

    while (has_frame && continue_capture) {
        rs2::frameset frame;

        has_frame = this->pipeline.try_wait_for_frames(&frame);

        if (has_frame) {

            spdlog::debug(
                "Split {}, Frame: {}", current_split_count, current_frame);

            if (current_frame == 1) {
                spdlog::info("Capture split {}...", current_split_count);

                auto depth = frame.get_depth_frame();
                auto points = this->pc.calculate(depth);

                auto pcl_points = tdr::utils::points::rs2_points_to_pcl(points);

                if (this->saveSplitFiles) {
                    std::stringstream fn;
                    fn << this->splitFileSavePath << "/" << current_split_count
                       << ".pcd";
                    spdlog::info("Saving split {} to {}...",
                                 current_split_count,
                                 fn.str());
                    pcl::io::savePCDFile(fn.str(), *pcl_points, true);
                }

                if (callback != nullptr) {
                    callback(pcl_points);
                }

                current_split_count++;
            }

            current_frame++;
            if (current_frame > this->captureInterval) {
                current_frame = 1;
            }

            continue_capture = current_split_count <= this->captureCount;
        } else {
            spdlog::warn("No more frames to capture.");
        }
    }

    this->pipeline.stop();
}

void tdr::realsense_operator::setSplitFileSavePath(std::string_view v) {
    this->splitFileSavePath = v;
}

void tdr::realsense_operator::setSaveSplitFiles(bool v) {
    this->saveSplitFiles = v;
}

void tdr::realsense_operator::setCaptureInterval(int v) {
    this->captureInterval = v;
}

void tdr::realsense_operator::setCaptureCount(int v) {
    this->captureCount = v;
}
