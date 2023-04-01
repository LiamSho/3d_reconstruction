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

#ifndef INC_3D_RECONSTRUCTION_POINTCLOUD_ALIGNER_HPP
#define INC_3D_RECONSTRUCTION_POINTCLOUD_ALIGNER_HPP

#include "../../utils/pcl_type_definition.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace tdr {

struct pointcloud_aligner_configuration {
    std::string file_save_directory{"icp_align"};
    std::string source_files_directory{"source"};

    bool visualization = false;
    bool visualization_per_iteration = false;
    uint iteration_count = 30;
    int k_search = 30;
};

class AlignerPointRepresentation
    : public pcl::PointRepresentation<pcl::PointNormal> {
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

  public:
    explicit AlignerPointRepresentation();
    void copyToFloatArray(const pcl::PointNormal &p, float *out) const override;
};

class pointcloud_aligner {
  private:
    std::vector<std::filesystem::path> cloud_files;
    Eigen::Matrix4f global_transform;

    pcl::visualization::PCLVisualizer *visualizer;
    int v_vp_1 = 1;
    int v_vp_2 = 2;

    pointcloud_aligner_configuration config;

    void clean_visualization();
    void show_clouds_left(const pcl_cloud &pc_tgt, const pcl_cloud &pc_src);
    void show_clouds_right(const pcl_cloud_normal &pc_tgt,
                           const pcl_cloud_normal &pc_src);
    void pair_align(const pcl_cloud &pc_src, const pcl_cloud &pc_tgt,
                    const pcl_cloud &output, Eigen::Matrix4f &final_transform,
                    size_t align_count);

  public:
    explicit pointcloud_aligner(const pointcloud_aligner_configuration &config);
    void align();

    Eigen::Matrix4f get_global_transform();
};

} // namespace tdr

#endif // INC_3D_RECONSTRUCTION_POINTCLOUD_ALIGNER_HPP
