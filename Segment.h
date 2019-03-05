#pragma once

#include "DistanceCalculator.h"
#include "constants.h"
#include "primitives.h"

struct Segment
{
    Segment(primitives::point_id_t a, primitives::point_id_t b, const DistanceCalculator& dc)
        : min(std::min(a, b)), max(std::max(a, b)), length(dc.compute_length(a, b)) {}
    primitives::point_id_t min {constants::invalid_point};
    primitives::point_id_t max {constants::invalid_point};
    primitives::length_t length {0};
};

