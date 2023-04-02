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

#include "pointcloud_processor.hpp"

#include "../../utils/fs_utils.hpp"
#include "../../utils/pcl_type_definition.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <spdlog/spdlog.h>

tdr::pointcloud_processor::pointcloud_processor(
    const pointcloud_processor_configuration &config) {

    this->config = config;
}

void tdr::pointcloud_processor::run() {

    spdlog::info("Input directory: {}", config.input_directory);
    spdlog::info("Input directory: {}", config.output_directory);

    auto content =
        tdr::utils::fs::get_directory_content(config.input_directory);
    std::sort(content.begin(), content.end());

    spdlog::info("Total input files: {}", content.size());

    tdr::utils::fs::ensure_directory_empty(config.output_directory);

    Eigen::Affine3d translation_transform = Eigen::Affine3d::Identity();
    Eigen::Affine3d rotation_transform = Eigen::Affine3d::Identity();
    Eigen::Vector3d unit = Eigen::Vector3d::UnitY();

    if (config.run_translation) {

        spdlog::info("Run translation: {}, {}, {}",
                     config.translation_x,
                     config.translation_y,
                     config.translation_z);

        translation_transform.translation() << config.translation_x,
            config.translation_y, config.translation_z;
    }
    if (config.run_rotation) {
        char unit_c = 'y';

        if (!config.unit.empty()) {
            switch (config.unit[0]) {
            case 'x':
            case 'X':
                unit = Eigen::Vector3d::UnitX();
                unit_c = 'x';
                break;
            case 'y':
            case 'Y':
                unit = Eigen::Vector3d::UnitY();
                unit_c = 'y';
                break;
            case 'z':
            case 'Z':
                unit = Eigen::Vector3d::UnitZ();
                unit_c = 'z';
                break;
            }
        }

        spdlog::info("Run rotation with base deg {}, incremental {}, unit {}",
                     config.rotation_degree_base,
                     config.incremental_rotation,
                     unit_c);
    }

    size_t i = 1;
    for (const auto &s : content) {
        std::stringstream output_file;
        output_file << config.output_directory << "/" << std::setw(3)
                    << std::setfill('0') << i << ".pcd";

        pcl_cloud cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile(s.string(), *cloud);
        spdlog::info("Load file: {}", s.string());

        if (config.run_translation) {
            pcl::transformPointCloud(*cloud, *cloud, translation_transform);

            spdlog::info("Applied translation transform");
        }

        if (config.run_rotation) {

            rotation_transform = Eigen::Affine3d::Identity();

            double deg = config.rotation_degree_base;

            if (config.incremental_rotation) {
                deg *= (double)i;
            }

            double rad = deg * M_PI / 180.0;

            rotation_transform.rotate(Eigen::AngleAxisd(rad, unit));

            pcl::transformPointCloud(*cloud, *cloud, rotation_transform);
            spdlog::info(
                "Applied rotation transform, {} degrees or {} rads", deg, rad);
        }

        pcl::io::savePCDFile(output_file.str(), *cloud, true);
        spdlog::info("Saved file: {}", output_file.str());
    }
}
