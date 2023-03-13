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

namespace tdr {

inline std::string_view get_env(const char *key) {
    if (key == nullptr) {
        throw std::invalid_argument(
            "Null pointer passed as environment variable name");
    }
    if (*key == '\0') {
        throw std::invalid_argument(
            "Value requested for the empty-name environment variable");
    }
    const char *ev_val = getenv(key);

    if (ev_val == nullptr) {
        return std::string_view{};
    }

    return std::string_view{ev_val};
}

} // namespace tdr

#endif // INC_3D_RECONSTRUCTION_ENV_UTIL_HPP
