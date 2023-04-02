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

#include "../include/lyra.hpp"
#include "libs/pointcloud_aligner/pointcloud_aligner.hpp"
#include "libs/pointcloud_fuser/pointcloud_fuser.hpp"
#include "libs/pointcloud_processor/pointcloud_processor.hpp"
#include "libs/realsense_operator/realsense_operator.hpp"
#include <GLFW/glfw3.h>
#include <iostream>
#include <librealsense2/rs.hpp>
#include <pcl/pcl_base.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <vector>

struct init_configuration {
    std::string log_file_path = "log.txt";
    bool debug_level = false;
    bool verbose_level = false;
};

int init(const init_configuration &config);

void build_cli(lyra::cli &cli, const std::string &command,
               const std::string &help_message, bool *show_help,
               init_configuration *init_config,
               const std::function<void(lyra::command &)> &builder,
               const std::function<void(const lyra::group &)> &run);

struct realsense_operator_command {
    tdr::realsense_operator_configuration config;
    bool show_help = false;

    init_configuration init_config;

    explicit realsense_operator_command(lyra::cli &cli) {
        build_cli(
            cli,
            "realsense-operator",
            "Run realsense operator module",
            &show_help,
            &init_config,
            [this](lyra::command &c) {
                c.add_argument(
                     lyra::opt(config.bag_file_path, "file path")
                         .required()
                         .name("-s")
                         .name("--source")
                         .help(
                             "The path of RealSense record file, end in .bag"))
                    .add_argument(
                        lyra::opt(config.split_file_save_path, "directory")
                            .optional()
                            .name("-o")
                            .name("--save-dir")
                            .help("The directory to save the result, "
                                  "default to \"rs_split\""))
                    .add_argument(
                        lyra::opt(config.capture_interval, "interval")
                            .optional()
                            .name("-i")
                            .name("--interval")
                            .help("The frames interval to capture, every first "
                                  "frame in the interval will be captured, "
                                  "default to 30"))
                    .add_argument(lyra::opt(config.capture_count, "count")
                                      .optional()
                                      .name("-c")
                                      .name("--count")
                                      .help("Total frames to capture"))
                    .add_argument(
                        lyra::opt(config.run_passthrough_filter)
                            .optional()
                            .name("-p")
                            .name("--passthrough-filter")
                            .help(
                                "Run passthrough filter with provided limits"))
                    .add_argument(
                        lyra::opt(config.passthrough_filter_x_min, "min")
                            .optional()
                            .name("-x")
                            .name("--x-min")
                            .help("Set the minimum x value for passthrough "
                                  "filter, default to -0.5"))
                    .add_argument(
                        lyra::opt(config.passthrough_filter_x_max, "max")
                            .optional()
                            .name("-X")
                            .name("--x-max")
                            .help("Set the maximum x value for passthrough "
                                  "filter, default to 0.5"))
                    .add_argument(
                        lyra::opt(config.passthrough_filter_x_min, "min")
                            .optional()
                            .name("-y")
                            .name("--y-min")
                            .help("Set the minimum y value for passthrough "
                                  "filter, default to -0.5"))
                    .add_argument(
                        lyra::opt(config.passthrough_filter_x_max, "max")
                            .optional()
                            .name("-Y")
                            .name("--y-max")
                            .help("Set the maximum y value for passthrough "
                                  "filter, default to 0.5"))
                    .add_argument(
                        lyra::opt(config.passthrough_filter_x_min, "min")
                            .optional()
                            .name("-z")
                            .name("--z-min")
                            .help("Set the minimum z value for passthrough "
                                  "filter, default to -0.5"))
                    .add_argument(
                        lyra::opt(config.passthrough_filter_x_max, "max")
                            .optional()
                            .name("-Z")
                            .name("--z-max")
                            .help("Set the maximum z value for passthrough "
                                  "filter, default to 0.5"));
            },
            [this](const lyra::group &g) { run(g); });
    }

    void run(const lyra::group &g) const {
        if (show_help) {
            std::cout << g;
            return;
        }

        auto init_result = init(init_config);
        if (init_result != 0) {
            return;
        }

        tdr::realsense_operator rs_operator(config);
        rs_operator.split_pointclouds();

        spdlog::info("Realsense operator module finished");
    }
};

struct pointcloud_aligner_command {
    tdr::pointcloud_aligner_configuration config;
    bool show_help = false;

    init_configuration init_config;

    explicit pointcloud_aligner_command(lyra::cli &cli) {
        build_cli(
            cli,
            "pointcloud-aligner",
            "Run pointcloud aligner module",
            &show_help,
            &init_config,
            [this](lyra::command &c) {
                c.add_argument(
                     lyra::opt(config.file_save_directory, "directory")
                         .optional()
                         .name("-o")
                         .name("--save-dir")
                         .help("The directory to save the result, "
                               "default to \"icp_align\""))
                    .add_argument(
                        lyra::opt(config.source_files_directory, "directory")
                            .optional()
                            .name("-s")
                            .name("--source-dir")
                            .help("The directory where source pcd files are "
                                  "saved, "
                                  "default to \"source\""))
                    .add_argument(lyra::opt(config.visualization)
                                      .optional()
                                      .name("-z")
                                      .name("--visualization")
                                      .help("Enable visualization"))
                    .add_argument(
                        lyra::opt(config.visualization_per_iteration)
                            .optional()
                            .name("-p")
                            .name("--visualization-per-iteration")
                            .help("Pause and visualize every iteration"))
                    .add_argument(
                        lyra::opt(config.iteration_count, "count")
                            .optional()
                            .name("-i")
                            .name("--iteration")
                            .help("Set the iteration count, default to 30"))
                    .add_argument(
                        lyra::opt(config.k_search, "depth")
                            .optional()
                            .name("-k")
                            .name("--k-search")
                            .help(
                                "Set the depth of the kd-tree, default to 30"))
                    .add_argument(lyra::opt(config.epsilon, "value")
                                      .optional()
                                      .name("-e")
                                      .name("--epsilon")
                                      .help("Set the ICP transformation "
                                            "epsilon, default to 1e-6"))
                    .add_argument(lyra::opt(config.distance_threshold, "value")
                                      .optional()
                                      .name("-t")
                                      .name("--threshold")
                                      .help("Set the ICP distance threshold, "
                                            "default to 0.05"))
                    .add_argument(
                        lyra::opt(config.epsilon, "value")
                            .optional()
                            .name("-u")
                            .name("--threshold-step")
                            .help(
                                "Set the ICP distance threshold step, must "
                                "be smaller than threshold, default to 0.001"));
            },
            [this](const lyra::group &g) { run(g); });
    }

    void run(const lyra::group &g) const {
        if (show_help) {
            std::cout << g;
            return;
        }

        auto init_result = init(init_config);
        if (init_result != 0) {
            return;
        }

        tdr::pointcloud_aligner aligner(config);
        aligner.align();

        spdlog::info("Pointcloud aligner module finished");

        std::stringstream ss;
        ss << aligner.get_global_transform().array();
        spdlog::info("Global transform: \n{}", ss.str());
    }
};

struct pointcloud_processor_command {
    tdr::pointcloud_processor_configuration config;
    bool show_help = false;

    init_configuration init_config;

    explicit pointcloud_processor_command(lyra::cli &cli) {
        build_cli(
            cli,
            "pointcloud-processor",
            "Run pointcloud processor module",
            &show_help,
            &init_config,
            [this](lyra::command &c) {
                c.add_argument(
                     lyra::opt(config.input_directory, "directory")
                         .optional()
                         .name("-i")
                         .name("--input-dir")
                         .help(
                             "The directory where source pcd files are saved, "
                             "default to \"input\""))
                    .add_argument(
                        lyra::opt(config.output_directory, "directory")
                            .optional()
                            .name("-o")
                            .name("--output-dir")
                            .help("The directory to save the output pcd files, "
                                  "default to \"output\""))
                    .add_argument(lyra::opt(config.run_translation)
                                      .optional()
                                      .name("-t")
                                      .name("--translation")
                                      .help("Run translation transform"))
                    .add_argument(
                        lyra::opt(config.translation_x, "x")
                            .optional()
                            .name("-x")
                            .name("--x-translation")
                            .help("X axis of translation transform, default to "
                                  "0.0"))
                    .add_argument(
                        lyra::opt(config.translation_y, "y")
                            .optional()
                            .name("-y")
                            .name("--y-translation")
                            .help("Y axis of translation transform, default to "
                                  "0.0"))
                    .add_argument(
                        lyra::opt(config.translation_z, "z")
                            .optional()
                            .name("-z")
                            .name("--z-translation")
                            .help("Z axis of translation transform, default to "
                                  "0.0"))
                    .add_argument(lyra::opt(config.run_rotation)
                                      .optional()
                                      .name("-r")
                                      .name("--rotation")
                                      .help("Run rotation transform"))
                    .add_argument(lyra::opt(config.incremental_rotation)
                                      .optional()
                                      .name("-s")
                                      .name("--incremental-rotation")
                                      .help("Incremental rotation mode for "
                                            "rotation transform"))
                    .add_argument(
                        lyra::opt(config.rotation_degree_base, "degree")
                            .optional()
                            .name("-g")
                            .name("--degree")
                            .help("The rotation degree base value, default to "
                                  "0.0"))
                    .add_argument(
                        lyra::opt(config.unit, "unit")
                            .optional()
                            .name("-u")
                            .name("--unit")
                            .choices("x", "y", "z")
                            .help("Rotation axis unit, could be \"x\", \"y\" "
                                  "or \"z\", default to \"y\""));
            },
            [this](const lyra::group &g) { run(g); });
    }

    void run(const lyra::group &g) const {
        if (show_help) {
            std::cout << g;
            return;
        }

        auto init_result = init(init_config);
        if (init_result != 0) {
            return;
        }

        tdr::pointcloud_processor processor(config);
        processor.run();

        spdlog::info("Pointcloud processor module finished");
    }
};

struct pointcloud_fuser_command {
    tdr::pointcloud_fuser_configuration config;
    bool show_help = false;

    init_configuration init_config;

    explicit pointcloud_fuser_command(lyra::cli &cli) {
        build_cli(
            cli,
            "pointcloud-fuser",
            "Run pointcloud fuser module",
            &show_help,
            &init_config,
            [this](lyra::command &c) {
                c.add_argument(
                     lyra::opt(config.pcd_input_dir, "directory")
                         .optional()
                         .name("-p")
                         .name("--pcd-input-dir")
                         .help(
                             "The directory where source pcd files are saved, "
                             "default to \"pcd_input\""))
                    .add_argument(
                        lyra::opt(config.matrix_input_dire, "directory")
                            .optional()
                            .name("-m")
                            .name("--matrix-input-dir")
                            .help("The directory where source matrix files are "
                                  "saved, default to \"matrix_input\""))
                    .add_argument(
                        lyra::opt(config.output_dir, "directory")
                            .optional()
                            .name("-o")
                            .name("--output-dir")
                            .help("The directory to save the output pcd files, "
                                  "default to \"output\""))
                    .add_argument(lyra::opt(config.voxel_grid_size, "value")
                                      .optional()
                                      .name("-g")
                                      .name("--grid-size")
                                      .help("The voxel grid cube size (m), "
                                            "default to 0.01"));
            },
            [this](const lyra::group &g) { run(g); });
    }

    void run(const lyra::group &g) const {
        if (show_help) {
            std::cout << g;
            return;
        }

        auto init_result = init(init_config);
        if (init_result != 0) {
            return;
        }

        tdr::pointcloud_fuser fuser(config);
        fuser.fuse();

        spdlog::info("Pointcloud fuser module finished");
    }
};

int main(int argc, char **argv) {
    auto cli = lyra::cli();
    std::string command;
    bool show_help = false;
    cli.add_argument(lyra::help(show_help).description("Show help message"));
    [[maybe_unused]] realsense_operator_command realsenseOperatorCommand{cli};
    [[maybe_unused]] pointcloud_aligner_command pointcloudAlignerCommand{cli};
    [[maybe_unused]] pointcloud_processor_command pointcloudProcessorCommand{
        cli};
    [[maybe_unused]] pointcloud_fuser_command pointcloudFuserCommand{cli};
    auto result = cli.parse({argc, argv});

    if (show_help) {
        std::cout << cli;
        return 0;
    }
    if (!result) {
        std::cerr << result.message() << "\n";
    }
    return result ? 0 : 1;
}

int init(const init_configuration &config) {
    spdlog::level::level_enum level = spdlog::level::info;
    if (config.debug_level) {
        level = spdlog::level::debug;
    }
    if (config.verbose_level) {
        level = spdlog::level::trace;
    }

    try {
        auto console_sink =
            std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        console_sink->set_level(level);

        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            config.log_file_path, true);
        file_sink->set_level(level);

        spdlog::sinks_init_list sink_list = {file_sink, console_sink};

        spdlog::logger logger("default", sink_list.begin(), sink_list.end());
        logger.set_level(level);

        spdlog::set_default_logger(std::make_shared<spdlog::logger>(
            "default", spdlog::sinks_init_list({console_sink, file_sink})));
    } catch (const spdlog::spdlog_ex &ex) {
        std::cerr << "Log initialization failed: " << ex.what() << std::endl;
        return 1;
    }

    spdlog::info("Starting 3d_reconstruction");
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

    return 0;
}

void build_cli(lyra::cli &cli, const std::string &command,
               const std::string &help_message, bool *show_help,
               init_configuration *init_config,
               const std::function<void(lyra::command &)> &builder,
               const std::function<void(const lyra::group &)> &run) {
    auto cmd =
        lyra::command(command, run)
            .help(help_message)
            .add_argument(
                lyra::help(*show_help).description("Show help message"))
            .add_argument(lyra::opt(init_config->debug_level)
                              .optional()
                              .name("-d")
                              .name("--debug")
                              .help("Set log level to debug, will be "
                                    "override by verbose"))
            .add_argument(lyra::opt(init_config->verbose_level)
                              .optional()
                              .name("-v")
                              .name("--verbose")
                              .help("Set log level to verbose"))
            .add_argument(
                lyra::opt(init_config->log_file_path, "file path")
                    .optional()
                    .name("-g")
                    .name("--log-file")
                    .help("The log file save path, default to \"log.txt\""));
    builder(cmd);
    cli.add_argument(cmd);
}
