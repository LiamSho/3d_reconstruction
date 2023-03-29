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
#include <pcl/io/ply_io.h>
#include <spdlog/spdlog.h>

pcl_cloud run_filter(const pcl_cloud &src, std::string_view fn, float min,
                     float max);

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

            spdlog::debug("Split {:0>3}, Frame: {:0>2}",
                          current_split_count,
                          current_frame);

            if (current_frame == 1) {
                spdlog::info("Capture split {:0>3}...", current_split_count);

                auto depth = frame.get_depth_frame();
                auto points = this->pc.calculate(depth);

                auto pcl_points = tdr::utils::points::rs2_points_to_pcl(points);

                // Filter
                if (this->runPassthroughFilter) {
                    spdlog::info("Running passthrough filter...");
                    pcl_points = run_filter(pcl_points,
                                            "x",
                                            this->passthroughFilterXMin,
                                            this->passthroughFilterXMax);
                    pcl_points = run_filter(pcl_points,
                                            "y",
                                            this->passthroughFilterYMin,
                                            this->passthroughFilterYMax);
                    pcl_points = run_filter(pcl_points,
                                            "z",
                                            this->passthroughFilterZMin,
                                            this->passthroughFilterZMax);
                    spdlog::info("Filter done.");
                }

                // Save
                if (this->saveSplitFiles) {
                    std::stringstream fn;
                    fn << this->splitFileSavePath << "/"
                       << fmt::format("{:0>3}", current_split_count) << ".ply";
                    spdlog::info("Saving split {:0>3} to {}...",
                                 current_split_count,
                                 fn.str());
                    pcl::io::savePLYFile(fn.str(), *pcl_points, true);
                }

                // Callback
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

void tdr::realsense_operator::setRunPassthroughFilter(bool v) {
    this->runPassthroughFilter = v;
}

void tdr::realsense_operator::setPassthroughFilterX(float min, float max) {
    this->passthroughFilterXMin = min;
    this->passthroughFilterXMax = max;
}

void tdr::realsense_operator::setPassthroughFilterY(float min, float max) {
    this->passthroughFilterYMin = min;
    this->passthroughFilterYMax = max;
}

void tdr::realsense_operator::setPassthroughFilterZ(float min, float max) {
    this->passthroughFilterZMin = min;
    this->passthroughFilterZMax = max;
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
