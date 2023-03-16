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
// Created by Liam Sho on 2023/3/16.
//

#ifndef INC_3D_RECONSTRUCTION_FS_UTILS_HPP
#define INC_3D_RECONSTRUCTION_FS_UTILS_HPP

#include <filesystem>
#include <string_view>
#include <vector>

namespace tdr::utils::fs {

namespace stdfs = std::filesystem;

inline void create_directory_if_not_exist(std::string_view directory_path) {

    if (stdfs::exists(directory_path)) {
        return;
    }

    stdfs::create_directory(directory_path);
}

inline void ensure_directory_empty(std::string_view directory_path) {

    if (stdfs::exists(directory_path)) {
        stdfs::remove_all(directory_path);
    }

    stdfs::create_directory(directory_path);
}

inline void get_directory_content(std::string_view directory_path) {

    stdfs::directory_iterator iterator(directory_path);
    std::vector<const stdfs::path> files;

    for (auto const &dir_entry : iterator) {
        files.push_back(dir_entry.path());
    }
}

} // namespace tdr::utils::fs

#endif // INC_3D_RECONSTRUCTION_FS_UTILS_HPP
