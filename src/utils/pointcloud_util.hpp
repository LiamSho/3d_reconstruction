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

#ifndef INC_3D_RECONSTRUCTION_POINTCLOUD_UTIL_HPP
#define INC_3D_RECONSTRUCTION_POINTCLOUD_UTIL_HPP

#include "pcl_type_definition.hpp"
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace tdr::utils::points {

pcl_cloud rs2_points_to_pcl(const rs2::points &points) {
    pcl_cloud cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();

    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto &p : cloud->points) {
        p.x = ptr->x;
        p.y = -ptr->y;
        p.z = ptr->z;
        ptr++;
    }

    return cloud;
}

} // namespace tdr::utils::points

#endif // INC_3D_RECONSTRUCTION_POINTCLOUD_UTIL_HPP
