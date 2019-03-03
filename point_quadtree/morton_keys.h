#pragma once

// Morton keys are interleaved coordinates, which are integer reprentations of x, y coordinates normalized to [0, 1].

#include "Domain.h"
#include <constants.h>
#include <primitives.h>

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <vector>

namespace point_quadtree {
namespace morton_keys {

inline primitives::morton_key_t interleave_coordinates(double normalized_coordinate1, double normalized_coordinate2)
{
    // if c1 and c2 are x and y respectively, then the curve looks like an "N" in "typical" coordinate space (+y is up, +x is right).
    using IntegerCoordinate = uint32_t;
    constexpr IntegerCoordinate IntegerCoordinateMax {static_cast<IntegerCoordinate>(1) << (constants::max_tree_depth - 1)}; // to be multiplied by the normalized (0,1) coordinate.
    IntegerCoordinate c1 {static_cast<IntegerCoordinate>(IntegerCoordinateMax * normalized_coordinate1)};
    IntegerCoordinate c2 {static_cast<IntegerCoordinate>(IntegerCoordinateMax * normalized_coordinate2)};
    primitives::morton_key_t morton_key {static_cast<primitives::morton_key_t>(0)};
    constexpr int bytes {sizeof(IntegerCoordinate)};
    constexpr int bits {8 * bytes};
    for (int i {bits - 1}; i >= 0; --i)
    {
        constexpr primitives::morton_key_t last_bit_mask {static_cast<primitives::morton_key_t>(1)};
        morton_key |= (c1 >> i) & last_bit_mask;
        morton_key <<= 1;
        morton_key |= (c2 >> i) & last_bit_mask;
        if (i != 0)
        {
            morton_key <<= 1;
        }
    }
    return morton_key;
}

inline std::vector<primitives::morton_key_t> compute_point_morton_keys(const std::vector<double>& x, const std::vector<double>& y,
    const Domain& domain)
{
    const size_t point_count {x.size()};
    std::vector<primitives::morton_key_t> point_morton_keys;
    for (size_t i {0}; i < point_count; ++i)
    {
        double x_normalized {(x[i] - domain.xmin()) / domain.xdim(0)};
        double y_normalized {(y[i] - domain.ymin()) / domain.ydim(0)};
        if (x_normalized < 0.0 or x_normalized > 1.0)
        {
            std::cout << __func__ << ": error: out-of-bounds normalized x coordinate: "
                << x_normalized << std::endl;
            std::abort();
        }
        if (y_normalized < 0.0 or y_normalized > 1.0)
        {
            std::cout << __func__ << ": error: out-of-bounds normalized y coordinate: "
                << y_normalized << std::endl;
            std::abort();
        }
        primitives::morton_key_t morton_key {interleave_coordinates(x_normalized, y_normalized)};
        point_morton_keys.push_back(morton_key);
    }
    return point_morton_keys;
}

inline std::array<primitives::quadrant_t, constants::max_tree_depth - 1> point_insertion_path(primitives::morton_key_t key)
{
    std::array<primitives::quadrant_t, constants::max_tree_depth - 1> path;
    for(int i {1}; i < constants::max_tree_depth; ++i)
    {
        const auto shift_bits {2 * (constants::max_tree_depth - i - 1)};
        primitives::morton_key_t level {key >> shift_bits};
        constexpr primitives::morton_key_t quadrant_mask {static_cast<primitives::morton_key_t>(3)}; // binary: 11
        const auto quadrant {static_cast<primitives::quadrant_t>(level & quadrant_mask)};
        path[i - 1] = quadrant;
    }
    return path;
}

inline std::vector<primitives::quadrant_t> segment_insertion_path(primitives::morton_key_t key1, primitives::morton_key_t key2)
{
    constexpr primitives::morton_key_t MORTON_THREE
        = static_cast<primitives::morton_key_t>(3); // quadrant mask.
    std::vector<primitives::quadrant_t> path;
    // We skip i = 0 because that corresponds to the root node,
    //  for which no bits are reserved.
    for(int i = 1; i < constants::max_tree_depth; ++i)
    {
        constexpr int max_quadrants {constants::max_tree_depth - 1};
        const int quadrant_shift {max_quadrants - i};
        const int bit_shift {2 * quadrant_shift};
        primitives::morton_key_t level1 = key1 >> bit_shift;
        primitives::morton_key_t level2 = key2 >> bit_shift;
        if (level1 == level2)
        {
            primitives::quadrant_t quadrant
                = static_cast<primitives::quadrant_t>(level1 & MORTON_THREE);
            path.push_back(quadrant);
        }
        else
        {
            break;
        }
    }
    return path;
}

} // namespace morton_keys
} // namespace point_quadtree
