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

#include "pointcloud_fuser.hpp"

#include "../../utils/fs_utils.hpp"
#include "../../utils/matrix_util.hpp"
#include "../../utils/pcl_type_definition.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <spdlog/spdlog.h>

tdr::pointcloud_fuser::pointcloud_fuser(
    const tdr::pointcloud_fuser_configuration &config) {

    this->config = config;
}

void tdr::pointcloud_fuser::fuse() const {
    auto pcd_files =
        tdr::utils::fs::get_directory_content(config.pcd_input_dir);
    auto matrix_files =
        tdr::utils::fs::get_directory_content(config.matrix_input_dire);

    tdr::utils::fs::ensure_directory_empty(config.output_dir);

    std::sort(pcd_files.begin(), pcd_files.end());
    std::sort(matrix_files.begin(), matrix_files.end());

    if (pcd_files.size() != matrix_files.size() + 1) {
        spdlog::error(
            "Number of pcd files ({}) and matrix files ({}) does not match",
            pcd_files.size(),
            matrix_files.size());
        return;
    }

    pcl_cloud source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud target_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // load target cloud
    pcl::io::loadPCDFile(pcd_files[matrix_files.size()], *target_cloud);
    std::stringstream ss;

    for (size_t i = matrix_files.size(); i > 0; i--) {

        // i -> i+1 transform
        auto matrix =
            tdr::utils::matrix::read_matrix4f_from_file(matrix_files[i]);

        ss.str("");
        ss << matrix.array();
        spdlog::info(
            "Transform matrix from {:0>3} to {:0>3}:\n {}", i + 1, i, ss.str());

        // load source cloud
        pcl::io::loadPCDFile(pcd_files[i - 1], *source_cloud);

        // apply transform to target cloud to get the aligned source cloud
        pcl::transformPointCloud(*target_cloud, *target_cloud, matrix);

        // fuse two clouds
        *source_cloud += *target_cloud;
        spdlog::info("Fused cloud size: {}", target_cloud->size());

        // apple voxel grid filter
        pcl_cloud filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(source_cloud);
        voxel_grid.setLeafSize(config.voxel_grid_size,
                               config.voxel_grid_size,
                               config.voxel_grid_size);
        voxel_grid.filter(*filtered_cloud);
        spdlog::info("Filtered fused cloud size: {}", filtered_cloud->size());

        // save final files
        pcl::io::savePCDFile(config.output_dir + "/fused_" + std::to_string(i) +
                                 ".pcd",
                             *source_cloud,
                             true);
        pcl::io::savePCDFile(config.output_dir + "/fused_" + std::to_string(i) +
                                 "_filtered.pcd",
                             *filtered_cloud,
                             true);

        spdlog::info("Fused fuse_of_{:0>3} to cloud {:0>3}", i, i + 1);

        target_cloud = source_cloud;
    }
}
