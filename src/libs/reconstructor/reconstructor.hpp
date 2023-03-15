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

#ifndef INC_3D_RECONSTRUCTION_RECONSTRUCTOR_HPP
#define INC_3D_RECONSTRUCTION_RECONSTRUCTOR_HPP

#include <spdlog/spdlog.h>
#include <string_view>

namespace tdr {

class reconstructor {
  private:
    std::string_view bag_file_path;

  public:
    explicit reconstructor(std::string_view bag_file_path);
    void run();
};

} // namespace tdr
#endif // INC_3D_RECONSTRUCTION_RECONSTRUCTOR_HPP
