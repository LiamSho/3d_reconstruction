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

#ifndef INC_3D_RECONSTRUCTION_PCL_TYPE_DEFINITION_HPP
#define INC_3D_RECONSTRUCTION_PCL_TYPE_DEFINITION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

using pcl_viewer = pcl::visualization::PCLVisualizer::Ptr;
using pcl_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using pcl_cloud_normal = pcl::PointCloud<pcl::PointNormal>::Ptr;

#endif // INC_3D_RECONSTRUCTION_PCL_TYPE_DEFINITION_HPP
