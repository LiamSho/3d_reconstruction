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

#ifndef INC_3D_RECONSTRUCTION_POINTCLOUD_PROCESSOR_HPP
#define INC_3D_RECONSTRUCTION_POINTCLOUD_PROCESSOR_HPP

#include <string>

namespace tdr {

struct pointcloud_processor_configuration {
    std::string input_directory{"input"};
    std::string output_directory{"output"};

    bool run_translation = false;
    double translation_x = 0.0;
    double translation_y = 0.0;
    double translation_z = 0.0;

    bool run_rotation = false;
    bool incremental_rotation = false;
    double rotation_degree_base = 0.0;
    std::string unit{"y"};
};

class pointcloud_processor {
  private:
    pointcloud_processor_configuration config;

  public:
    explicit pointcloud_processor(
        const pointcloud_processor_configuration &config);

    void run();
};

} // namespace tdr

#endif // INC_3D_RECONSTRUCTION_POINTCLOUD_PROCESSOR_HPP
