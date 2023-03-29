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

#include "../include/meojson.hpp"
#include "libs/pointcloud_aligner/pointcloud_aligner.hpp"
#include "libs/realsense_operator/realsense_operator.hpp"
#include "utils/env_util.hpp"
#include "utils/pcl_type_definition.hpp"
#include <GLFW/glfw3.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/pcl_base.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <vector>

struct configuration {
    std::string bag_file_path;

    // Realsense operator
    int capture_count{};
    int capture_interval{};
    bool save_split_files{};

    bool run_filter{};
    float filter_x_min{};
    float filter_x_max{};
    float filter_y_min{};
    float filter_y_max{};
    float filter_z_min{};
    float filter_z_max{};

    // Pointcloud aligner
    int k_search{};
    int iteration_count{};
    bool visualization{};
    bool visualization_per_iteration{};
    bool save_every_aligned_pair{};
};

int logger_init();
void lib_init();
std::optional<configuration> parse_config(std::string_view config_file_path);

int main(int argc, char **argv) {
    auto logger_init_result = logger_init();
    if (logger_init_result != 0) {
        return -1;
    }

    spdlog::info("Starting 3d_reconstruction");
    lib_init();

    // Get the configuration file path
    std::string_view config_file;
    if (argc == 2) {
        spdlog::debug("Use config file parsed from command line");
        config_file = argv[1];
    } else {
        spdlog::debug("Use config file parsed from environment variable");
        config_file = tdr::utils::get_env("TDR_CONFIG_FILE_PATH");
    }

    if (config_file.empty()) {
        spdlog::error("Failed to get config file path");
        return -2;
    }

    // parse the configuration file
    auto config = parse_config(config_file);
    if (!config) {
        spdlog::error("Failed to parse config file");
        return -2;
    }

    std::vector<pcl_cloud> clouds;

    tdr::realsense_operator rs_operator(config->bag_file_path);
    rs_operator.setSaveSplitFiles(config->save_split_files);
    rs_operator.setCaptureCount(config->capture_count);
    rs_operator.setCaptureInterval(config->capture_interval);
    rs_operator.setRunPassthroughFilter(config->run_filter);
    rs_operator.setPassthroughFilterX(config->filter_x_min,
                                      config->filter_x_max);
    rs_operator.setPassthroughFilterY(config->filter_y_min,
                                      config->filter_y_max);
    rs_operator.setPassthroughFilterZ(config->filter_z_min,
                                      config->filter_z_max);
    rs_operator.split_pointclouds(
        [&](const pcl_cloud &c) { clouds.push_back(c); });

    tdr::pointcloud_aligner aligner(clouds);
    aligner.setKSearch(config->k_search);
    aligner.setIterationCount(config->iteration_count);
    aligner.setVisualization(config->visualization);
    aligner.setVisualizationPerIteration(config->visualization_per_iteration);
    aligner.setSaveEveryAlignedPair(config->save_every_aligned_pair);
    aligner.align();

    std::stringstream logStream;
    logStream << "Final global transformation matrix: " << std::endl
              << aligner.get_global_transform().array() << std::endl;
    spdlog::info(logStream.str());

    return 0;
}

int logger_init() {
    try {
        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(spdlog::level::debug);

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            "logs/log.txt", true);
        file_sink->set_level(spdlog::level::trace);

        spdlog::sinks_init_list sink_list = {file_sink, console_sink};

        spdlog::logger logger("default", sink_list.begin(), sink_list.end());
        logger.set_level(spdlog::level::debug);

        spdlog::set_default_logger(std::make_shared<spdlog::logger>(
            "default", spdlog::sinks_init_list({console_sink, file_sink})));

        return 0;
    } catch (const spdlog::spdlog_ex &ex) {
        std::cout << "Log initialization failed: " << ex.what() << std::endl;

        return 1;
    }
}

void lib_init() {
    spdlog::info("============[LIB VERSIONS]=============");
    spdlog::info("spdlog: {}.{}.{}",
                 SPDLOG_VER_MAJOR,
                 SPDLOG_VER_MINOR,
                 SPDLOG_VER_PATCH);
    spdlog::info("pcl: {}.{}.{}",
                 PCL_MAJOR_VERSION,
                 PCL_MINOR_VERSION,
                 PCL_REVISION_VERSION);
    spdlog::info("librealsense: {}", RS2_API_VERSION_STR);
    spdlog::info("glfw: {}.{}.{}",
                 GLFW_VERSION_MAJOR,
                 GLFW_VERSION_MINOR,
                 GLFW_VERSION_REVISION);
    spdlog::info("=======================================");
}

std::optional<configuration> parse_config(std::string_view config_file_path) {
    configuration config;

    std::ifstream config_file(config_file_path.data());
    std::string config_str((std::istreambuf_iterator<char>(config_file)),
                           std::istreambuf_iterator<char>());
    config_file.close();

    auto ret = json::parse(config_str);
    if (!ret) {
        spdlog::error("Failed to parse config file");
        return std::nullopt;
    }

    auto config_value = *ret;

    auto rs_operator = config_value["rs_operator"];
    config.bag_file_path = rs_operator.get("bag_file_path", "input.bag");
    config.capture_count = rs_operator.get("capture_count", 2);
    config.capture_interval = rs_operator.get("capture_interval", 30);
    config.save_split_files = rs_operator.get("save_split_files", true);

    auto filter = rs_operator["filter"];
    config.run_filter = filter.get("run_filter", true);
    config.filter_x_min = filter["x"].get("min", -0.5f);
    config.filter_x_max = filter["x"].get("max", 0.5f);
    config.filter_y_min = filter["y"].get("min", -0.5f);
    config.filter_y_max = filter["y"].get("max", 0.5f);
    config.filter_z_min = filter["z"].get("min", -0.5f);
    config.filter_z_max = filter["z"].get("max", 0.5f);

    auto pointcloud_aligner = config_value["pointcloud_aligner"];
    config.k_search = pointcloud_aligner.get("k_search", 30);
    config.iteration_count = pointcloud_aligner.get("iteration_count", 5);
    config.visualization = pointcloud_aligner.get("visualization", true);
    config.visualization_per_iteration =
        pointcloud_aligner.get("visualization_per_iteration", true);
    config.save_every_aligned_pair =
        pointcloud_aligner.get("save_every_aligned_pair", true);

    return config;
}
