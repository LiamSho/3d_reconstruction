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

#include "pointcloud_aligner.hpp"

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <spdlog/spdlog.h>
#include <utility>

tdr::AlignerPointRepresentation::AlignerPointRepresentation() {
    this->nr_dimensions_ = 4;
}
void tdr::AlignerPointRepresentation::copyToFloatArray(
    const pcl::PointNormal &p, float *out) const {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
}

tdr::pointcloud_aligner::pointcloud_aligner(std::vector<pcl_cloud> v_clouds) {
    this->clouds = std::move(v_clouds);

    this->global_transform = Eigen::Matrix4f::Identity();

    this->visualizer = new pcl::visualization::PCLVisualizer();
    this->visualizer->setWindowName(
        "TDR - Incremental Registration Visualization");
    this->visualizer->setBackgroundColor(0.0, 0.0, 0.0);
    this->visualizer->createViewPort(0.0, 0, 0.5, 1.0, this->v_vp_1);
    this->visualizer->createViewPort(0.5, 0, 1.0, 1.0, this->v_vp_2);
}

void tdr::pointcloud_aligner::pair_align(const pcl_cloud &pc_src,
                                         const pcl_cloud &pc_tgt,
                                         const pcl_cloud &output,
                                         Eigen::Matrix4f &final_transform) {

    pcl_cloud src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud tgt(new pcl::PointCloud<pcl::PointXYZ>);

    src = pc_src;
    tgt = pc_tgt;

    // Compute surface normals and curvature
    pcl_cloud_normal points_with_normals_src(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl_cloud_normal points_with_normals_tgt(
        new pcl::PointCloud<pcl::PointNormal>);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_set;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    norm_set.setSearchMethod(tree);
    norm_set.setKSearch(30);

    norm_set.setInputCloud(src);
    norm_set.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_set.setInputCloud(tgt);
    norm_set.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    // Point representation and weight
    AlignerPointRepresentation point_rep;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_rep.setRescaleValues(alpha);

    // Align
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
    reg.setTransformationEpsilon(1e-6);
    reg.setMaxCorrespondenceDistance(0.1);
    reg.setPointRepresentation(
        pcl::make_shared<const AlignerPointRepresentation>(point_rep));

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Optimization
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prev = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f target_to_source = Eigen::Matrix4f::Identity();
    pcl_cloud_normal reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);

    for (size_t i = 0; i < 30; ++i) {
        spdlog::info("Iteration Nr.{}", i);

        points_with_normals_src = reg_result;

        // Estimate
        reg.setInputSource(points_with_normals_src);
        reg.align(*reg_result);

        // Accumulate transforms between each Iteration
        Ti = reg.getFinalTransformation() * Ti;

        // Check threshold
        if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) <
            reg.getTransformationEpsilon()) {
            reg.setMaxCorrespondenceDistance(
                reg.getMaxCorrespondenceDistance() - 0.001);
        }

        prev = reg.getLastIncrementalTransformation();

        if (this->visualization) {
            this->show_clouds_right(points_with_normals_tgt,
                                    points_with_normals_src);
        }
    }

    // Target -> Source transformation
    target_to_source = Ti.inverse();

    // Transform target to source frame
    pcl::transformPointCloud(*pc_tgt, *output, target_to_source);

    if (this->visualization) {
        this->visualizer->removePointCloud("vp2_source");
        this->visualizer->removePointCloud("vp2_target");

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(
            output, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(
            pc_src, 255, 0, 0);

        this->visualizer->addPointCloud(
            output, tgt_h, "vp2_target", this->v_vp_2);
        this->visualizer->addPointCloud(
            pc_src, src_h, "vp2_source", this->v_vp_2);

        this->visualizer->spin();

        this->visualizer->removePointCloud("vp2_source");
        this->visualizer->removePointCloud("vp2_target");
    }

    *output += *pc_src;
    final_transform = target_to_source;
}

void tdr::pointcloud_aligner::align() {
    if (this->visualization) {
        clean_visualization();
    }

    auto pc_count = this->clouds.size();
    if (pc_count < 2) {
        spdlog::warn("Not enough pointclouds");
        return;
    }

    Eigen::Matrix4f pair_transform = Eigen::Matrix4f::Identity();
    this->global_transform.setZero();

    pcl_cloud source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud result(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 1; i < pc_count; ++i) {

        spdlog::info("Align: No.{} to No.{}", i - 1, i);

        source = this->clouds[i - 1];
        target = this->clouds[i];

        if (this->visualization) {
            this->show_clouds_left(source, target);
        }

        pcl_cloud temp(new pcl::PointCloud<pcl::PointXYZ>);

        pair_align(source, target, temp, pair_transform);

        pcl::transformPointCloud(*temp, *result, this->global_transform);

        // Update global transform
        this->global_transform *= pair_transform;

        if (this->save_every_aligned_pair) {
            std::stringstream ss;
            ss << i << ".pcd";
            pcl::io::savePCDFile(ss.str(), *result, true);
        }
    }
}

void tdr::pointcloud_aligner::setSaveEveryAlignedPair(bool v) {
    this->save_every_aligned_pair = v;
}

void tdr::pointcloud_aligner::setVisualization(bool v) {
    this->visualization = v;
}

Eigen::Matrix4f tdr::pointcloud_aligner::get_global_transform() {
    return this->global_transform;
}

void tdr::pointcloud_aligner::clean_visualization() {
    this->visualizer->removeAllPointClouds();
}

void tdr::pointcloud_aligner::show_clouds_left(const pcl_cloud &pc_tgt,
                                               const pcl_cloud &pc_src) {
    this->visualizer->removePointCloud("vp1_target");
    this->visualizer->removePointCloud("vp1_source");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(
        pc_tgt, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(
        pc_src, 255, 0, 0);

    this->visualizer->addPointCloud(pc_tgt, tgt_h, "vp1_target", v_vp_1);
    this->visualizer->addPointCloud(pc_src, src_h, "vp1_source", v_vp_1);

    this->visualizer->spin();
}

void tdr::pointcloud_aligner::show_clouds_right(
    const pcl_cloud_normal &pc_tgt, const pcl_cloud_normal &pc_src) {
    this->visualizer->removePointCloud("vp2_source");
    this->visualizer->removePointCloud("vp2_target");

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal>
        tgt_color_handler(pc_tgt, "curvature");
    if (!tgt_color_handler.isCapable()) {
        spdlog::warn(
            "Can not create curvature color handler for Target in vp2");
    }

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal>
        src_color_handler(pc_src, "curvature");
    if (!src_color_handler.isCapable()) {
        spdlog::warn(
            "Can not create curvature color handler for Source in vp2");
    }

    this->visualizer->addPointCloud(
        pc_tgt, tgt_color_handler, "vp2_target", this->v_vp_2);
    this->visualizer->addPointCloud(
        pc_src, src_color_handler, "vp2_source", this->v_vp_2);

    this->visualizer->spinOnce();
}