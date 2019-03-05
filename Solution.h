#pragma once

#include "primitives.h"

#include <vector>

struct Solution
{
    std::vector<primitives::point_id_t> ordered_points; // in order of traversal.
    primitives::length_t length{0};
    size_t iterations {0};
    primitives::length_t total_improvement{0};
};

