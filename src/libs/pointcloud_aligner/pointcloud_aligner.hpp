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

//
// Created by Liam Sho on 2023/3/15.
//

#ifndef INC_3D_RECONSTRUCTION_POINTCLOUD_ALIGNER_HPP
#define INC_3D_RECONSTRUCTION_POINTCLOUD_ALIGNER_HPP

#include "../../utils/pcl_type_definition.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace tdr {

class AlignerPointRepresentation
    : public pcl::PointRepresentation<pcl::PointNormal> {
    using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;

  public:
    explicit AlignerPointRepresentation();
    void copyToFloatArray(const pcl::PointNormal &p, float *out) const override;
};

class pointcloud_aligner {
  private:
    const std::string_view file_save_directory{"icp_align"};

    std::vector<pcl_cloud> clouds;
    Eigen::Matrix4f global_transform;

    bool save_every_aligned_pair = false;
    bool visualization = false;
    size_t iteration_count = 30;
    int k_search = 30;

    pcl::visualization::PCLVisualizer *visualizer;
    int v_vp_1 = 1;
    int v_vp_2 = 2;

    void clean_visualization();
    void show_clouds_left(const pcl_cloud &pc_tgt, const pcl_cloud &pc_src);
    void show_clouds_right(const pcl_cloud_normal &pc_tgt,
                           const pcl_cloud_normal &pc_src);

  public:
    void pair_align(const pcl_cloud &pc_src, const pcl_cloud &pc_tgt,
                    const pcl_cloud &output, Eigen::Matrix4f &final_transform);

    explicit pointcloud_aligner(std::vector<pcl_cloud> v_clouds);
    void align();

    void setSaveEveryAlignedPair(bool b);
    void setVisualization(bool v);
    void setIterationCount(size_t v);
    void setKSearch(int v);

    Eigen::Matrix4f get_global_transform();
};

} // namespace tdr

#endif // INC_3D_RECONSTRUCTION_POINTCLOUD_ALIGNER_HPP
