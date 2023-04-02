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

#ifndef INC_3D_RECONSTRUCTION_MATRIX_UTIL_HPP
#define INC_3D_RECONSTRUCTION_MATRIX_UTIL_HPP

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

namespace tdr::utils::matrix {

inline void write_matrix4f_to_file(const Eigen::Matrix4f &matrix,
                                   const std::string &file) {

    std::ofstream ofs(file);

    for (int row = 0; row < 4; row++) {
        for (int col = 0; col < 4; col++) {
            if (col == 3) {
                ofs << matrix(row, col) << std::endl;
            } else {
                ofs << matrix(row, col) << ",";
            }
        }
    }

    ofs.flush();
    ofs.close();
}

inline Eigen::Matrix4f read_matrix4f_from_file(const std::string &file) {
    Eigen::Matrix4f matrix;

    std::ifstream ifs(file);

    for (int row = 0; row < 4; row++) {
        std::string line;
        std::getline(ifs, line);

        std::stringstream ss(line);
        std::string item;
        int col = 0;
        while (std::getline(ss, item, ',')) {
            matrix(row, col) = std::stof(item);
            col++;
        }
    }

    ifs.close();

    return matrix;
}

} // namespace tdr::utils::matrix

#endif // INC_3D_RECONSTRUCTION_MATRIX_UTIL_HPP
