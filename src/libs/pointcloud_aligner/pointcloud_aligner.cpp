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

#include "pointcloud_aligner.hpp"

#include "../../utils/fs_utils.hpp"
#include "../../utils/matrix_util.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/transforms.h>
#include <spdlog/spdlog.h>
#include <vector>

#ifdef USE_OPENMP
#include <pcl/features/normal_3d_omp.h>
#else
#include <pcl/features/normal_3d.h>
#endif

#ifdef USE_CUDA
#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#else
#include <pcl/registration/icp_nl.h>
#endif

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

tdr::pointcloud_aligner::pointcloud_aligner(
    const pointcloud_aligner_configuration &config) {
    this->config = config;

    this->cloud_files = tdr::utils::fs::get_directory_content(
        this->config.source_files_directory);
    std::sort(this->cloud_files.begin(), this->cloud_files.end());

    if (this->config.visualization) {
        this->visualizer = new pcl::visualization::PCLVisualizer();
        this->visualizer->setWindowName(
            "TDR - Incremental Registration Visualization");
        this->visualizer->setBackgroundColor(0.0, 0.0, 0.0);
        this->visualizer->createViewPort(0.0, 0, 0.5, 1.0, this->v_vp_1);
        this->visualizer->createViewPort(0.5, 0, 1.0, 1.0, this->v_vp_2);
        this->visualizer->setCameraPosition(0, 0, -2, 0, 1, 0, v_vp_1);
        this->visualizer->setCameraPosition(0, 0, -2, 0, 1, 0, v_vp_2);
        this->visualizer->addCoordinateSystem(0.2, 0, 0, 0, "vp1", v_vp_1);
        this->visualizer->addCoordinateSystem(0.2, 0, 0, 0, "vp2", v_vp_2);
    }
}

void tdr::pointcloud_aligner::pair_align(const pcl_cloud &pc_src,
                                         const pcl_cloud &pc_tgt,
                                         const pcl_cloud &output,
                                         Eigen::Matrix4f &final_transform,
                                         size_t align_count = -1) {

    pcl_cloud src(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_cloud tgt(new pcl::PointCloud<pcl::PointXYZ>);

    src = pc_src;
    tgt = pc_tgt;

    // Compute surface normals and curvature
    pcl_cloud_normal points_with_normals_src(
        new pcl::PointCloud<pcl::PointNormal>);
    pcl_cloud_normal points_with_normals_tgt(
        new pcl::PointCloud<pcl::PointNormal>);

#ifdef USE_OPENMP
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> norm_set;
#else
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> norm_set;
#endif
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);

    norm_set.setSearchMethod(tree);
    norm_set.setKSearch(this->config.k_search);

    spdlog::info("[P{:0>3}] Computing surface normals and curvature for source",
                 align_count);
    norm_set.setInputCloud(src);
    norm_set.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    spdlog::info("[P{:0>3}] Computing surface normals and curvature for target",
                 align_count);
    norm_set.setInputCloud(tgt);
    norm_set.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    spdlog::info("[P{:0>3}] Surface normals and curvature compute done",
                 align_count);

    // Point representation and weight
    AlignerPointRepresentation point_rep;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};
    point_rep.setRescaleValues(alpha);

    // Align
#ifdef USE_CUDA
    fast_gicp::FastVGICPCuda<pcl::PointNormal, pcl::PointNormal> reg;
#else
    pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> reg;
#endif
    reg.setTransformationEpsilon(config.epsilon);
    reg.setMaxCorrespondenceDistance(config.distance_threshold);
    reg.setPointRepresentation(
        pcl::make_shared<const AlignerPointRepresentation>(point_rep));

    reg.setInputSource(points_with_normals_src);
    reg.setInputTarget(points_with_normals_tgt);

    // Optimization
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f prev = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f target_to_source = Eigen::Matrix4f::Identity();
    pcl_cloud_normal reg_result = points_with_normals_src;
    reg.setMaximumIterations(500);

    for (uint i = 1; i <= this->config.iteration_count; ++i) {

        if (align_count >= 0) {
            spdlog::info("[P{:0>3}] Iteration Nr.{:0>3}", align_count, i);
        } else {
            spdlog::info("Iteration Nr.{:0>3}", i);
        }

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
                reg.getMaxCorrespondenceDistance() -
                config.distance_threshold_step);
        }

        prev = reg.getLastIncrementalTransformation();
    }

    // Target -> Source transformation
    target_to_source = Ti.inverse();

    // Transform target to source frame
    pcl::transformPointCloud(*pc_tgt, *output, target_to_source);

    // Log ICP result
    spdlog::info(
        "[P{:0>3}] ICP has converged: {}", align_count, reg.hasConverged());
    spdlog::info("[P{:0>3}] ICP score: {}", align_count, reg.getFitnessScore());

    this->show_clouds_right(output, pc_src);

    *output += *pc_src;
    final_transform = target_to_source;

    if (align_count >= 0) {
        spdlog::info(
            "[P{:0>3}] Align count {:0>3} done", align_count, align_count);
    } else {
        spdlog::info("Pair align done");
    }
}

void tdr::pointcloud_aligner::align() {
    clean_visualization();

    tdr::utils::fs::ensure_directory_empty(this->config.file_save_directory);

    auto pc_count = this->cloud_files.size();
    if (pc_count < 2) {
        spdlog::warn("Not enough pointclouds");
        return;
    }

    std::vector<pcl_cloud> clouds;
    for (const auto &f : this->cloud_files) {
        pcl_cloud cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(f, *cloud);
        clouds.push_back(cloud);
    }

#ifdef USE_CUDA
    spdlog::info("CUDA is enabled. Will run VGICP on CUDA.");
#else
    spdlog::info("CUDA is disabled. Will run PCL builtin ICP with L-M method on CPU.");
#endif

    for (size_t i = 1; i < pc_count; ++i) {
        Eigen::Matrix4f pair_transform = Eigen::Matrix4f::Identity();

        pcl_cloud source(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_cloud target(new pcl::PointCloud<pcl::PointXYZ>);
        pcl_cloud result(new pcl::PointCloud<pcl::PointXYZ>);

        spdlog::info("[P{:0>3}] Align: No.{:0>3} to No.{:0>3}", i, i, i + 1);
        source = clouds[i - 1];
        target = clouds[i];

        this->show_clouds_left(source, target);

        pair_align(source, target, result, pair_transform, i);

        // Update and log transform
        std::stringstream logStream;
        logStream << "[P" << std::setw(3) << std::setfill('0') << i << "] "
                  << "Pair transform: \n"
                  << pair_transform.array();
        spdlog::info(logStream.str());

        std::stringstream ss;
        ss << config.file_save_directory << "/" << fmt::format("{:0>3}", i)
           << ".pcd";
        pcl::io::savePCDFile(ss.str(), *result, true);

        ss.str("");
        ss << config.file_save_directory << "/" << fmt::format("{:0>3}", i)
           << "_transform.txt";
        tdr::utils::matrix::write_matrix4f_to_file(pair_transform, ss.str());

        ss.clear();
    }
}

void tdr::pointcloud_aligner::clean_visualization() {

    if (!this->config.visualization) {
        return;
    }

    this->visualizer->removeAllPointClouds();
}

void tdr::pointcloud_aligner::show_clouds_left(const pcl_cloud &pc_tgt,
                                               const pcl_cloud &pc_src) {

    if (!this->config.visualization) {
        return;
    }

    this->visualizer->removePointCloud("vp1_target");
    this->visualizer->removePointCloud("vp1_source");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(
        pc_tgt, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(
        pc_src, 255, 0, 0);

    this->visualizer->addPointCloud(pc_tgt, tgt_h, "vp1_target", v_vp_1);
    this->visualizer->addPointCloud(pc_src, src_h, "vp1_source", v_vp_1);

    spdlog::info("Update visualizer, press q to start the alignment");
    this->visualizer->spin();
}

void tdr::pointcloud_aligner::show_clouds_right(const pcl_cloud &pc_tgt,
                                                const pcl_cloud &pc_src) {

    if (!this->config.visualization) {
        return;
    }

    this->visualizer->removePointCloud("vp2_source");
    this->visualizer->removePointCloud("vp2_target");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(
        pc_tgt, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(
        pc_src, 255, 0, 0);

    this->visualizer->addPointCloud(pc_tgt, tgt_h, "vp2_target", this->v_vp_2);
    this->visualizer->addPointCloud(pc_src, src_h, "vp2_source", this->v_vp_2);

    spdlog::info("Pair align done. Press q to continue the registration");
    this->visualizer->spin();
}
