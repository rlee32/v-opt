#pragma once

// For point i, all adjacent segments are removed.
// For point j, only j to next[j] is removed.
// In the case of a perturbing move, improvement will refer
//  only to the difference between the minimum of the new and old
//  segment lengths, not the overall improvement.

#include "constants.h"
#include "primitives.h"

struct VMove
{
    primitives::point_id_t i {constants::invalid_point};
    primitives::point_id_t j {constants::invalid_point};
    primitives::length_t improvement {0};

    void apply(const VMove& other)
    {
        if (other.improvement > improvement)
        {
            i = other.i;
            j = other.j;
            improvement = other.improvement;
        }
    }
};

