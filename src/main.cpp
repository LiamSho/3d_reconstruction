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

#include "utils/env_util.hpp"
#include <iostream>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

int logger_init();

int main(int argc, char **argv) {
    auto logger_init_result = logger_init();
    if (logger_init_result != 0) {
        return -1;
    }

    spdlog::info("Starting 3d_reconstruction");

    std::string_view bag_file;
    if (argc == 2) {
        spdlog::debug("Use bag file parsed from command line");
        bag_file = argv[1];
    } else {
        spdlog::debug("Use bag file parsed from environment variable");
        bag_file = tdr::get_env("TDR_BAG_FILE_PATH");
    }

    if (bag_file.empty()) {
        spdlog::error("Bag file path is empty");
        return -2;
    }

    spdlog::info("Use bag file: {}", bag_file);

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
