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

#ifndef INC_3D_RECONSTRUCTION_PCL_VIEWER_UTIL_HPP
#define INC_3D_RECONSTRUCTION_PCL_VIEWER_UTIL_HPP

#include "pcl_type_definition.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>

namespace tdr::utils::visualizer {

pcl_viewer create_pcl_viewer(std::string_view title) {

    spdlog::debug("Start to create PCL viewer {}", title.data());

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer());

    viewer->setWindowName(title.data());
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    spdlog::info("PCL viewer {} created", title.data());

    return viewer;
}

pcl_viewer add_pointcloud(const pcl_viewer &viewer, std::string_view cloud_id,
                          const pcl_cloud &cloud) {

    spdlog::debug(R"(Start to add pointcloud "{}" to viewer "{}")",
                  cloud_id.data(),
                  viewer->getRenderWindow()->GetWindowName());

    viewer->addPointCloud<pcl::PointXYZ>(cloud, cloud_id.data());
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_id.data());

    spdlog::info(R"(Pointcloud "{}" added to viewer "{}")",
                 cloud_id.data(),
                 viewer->getRenderWindow()->GetWindowName());

    return viewer;
}

void show_pcl_viewer(const pcl_viewer &viewer) {
    spdlog::info(R"(Spin PCL viewer "{}")",
                 viewer->getRenderWindow()->GetWindowName());
    viewer->spin();

    spdlog::info(R"(Stop PCL viewer "{}")",
                 viewer->getRenderWindow()->GetWindowName());
}

} // namespace tdr::utils::visualizer

#endif // INC_3D_RECONSTRUCTION_PCL_VIEWER_UTIL_HPP
