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

#ifndef INC_3D_RECONSTRUCTION_ENV_UTIL_HPP
#define INC_3D_RECONSTRUCTION_ENV_UTIL_HPP

#include <iostream>
#include <spdlog/spdlog.h>

namespace tdr::utils {

inline std::string_view get_env(const char *key) {
    if (key == nullptr) {
        spdlog::error("Null pointer passed as environment variable name");
        return std::string_view{};
    }
    if (*key == '\0') {
        spdlog::error(
            "Value requested for the empty-name environment variable");
        return std::string_view{};
    }
    const char *ev_val = getenv(key);

    if (ev_val == nullptr) {
        spdlog::error("The environment variable {} is not set", key);
        return std::string_view{};
    }

    return std::string_view{ev_val};
}

} // namespace tdr::utils

#endif // INC_3D_RECONSTRUCTION_ENV_UTIL_HPP
