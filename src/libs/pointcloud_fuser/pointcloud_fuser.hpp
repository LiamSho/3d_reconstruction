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

#ifndef INC_3D_RECONSTRUCTION_POINTCLOUD_FUSER_HPP
#define INC_3D_RECONSTRUCTION_POINTCLOUD_FUSER_HPP

#include <string>

namespace tdr {

struct pointcloud_fuser_configuration {
    std::string pcd_input_dir{"pcd_input"};
    std::string matrix_input_dire{"matrix_input"};
    std::string output_dir{"output"};

    float voxel_grid_size = 0.01;
};

class pointcloud_fuser {
  private:
    pointcloud_fuser_configuration config;

  public:
    explicit pointcloud_fuser(const pointcloud_fuser_configuration &config);
    void fuse() const;
};

} // namespace tdr

#endif // INC_3D_RECONSTRUCTION_POINTCLOUD_FUSER_HPP
