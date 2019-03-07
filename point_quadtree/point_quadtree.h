#pragma once

#include "Node.h"
#include "morton_keys.h"
#include <check.h>
#include <primitives.h>

namespace point_quadtree {

inline primitives::grid_t quadrant_x(primitives::quadrant_t q)
{
    // assuming "N" curve; see morton_keys::interleave_coordinates for an explanation.
    switch(q)
    {
        case 0:
        case 1:
        {
            return 0;
        }
        case 2:
        case 3:
        {
            return 1;
        }
        default: return 0;
    }
}

inline primitives::grid_t quadrant_y(primitives::quadrant_t q)
{
    // assuming "N" curve; see morton_keys::interleave_coordinates for an explanation.
    switch(q)
    {
        case 0:
        case 2:
        {
            return 0;
        }
        case 1:
        case 3:
        {
            return 1;
        }
        default: return 0;
    }
}

inline const Node* insert_point(const std::vector<primitives::morton_key_t>& morton_keys
    , primitives::point_id_t point_id
    , Node* root
    , const Domain& domain)
{
    auto point_destination {root};
    primitives::depth_t depth {0};
    primitives::grid_t x {0};
    primitives::grid_t y {0};
    for (const auto quadrant : morton_keys::point_insertion_path(morton_keys[point_id]))
    {
        ++depth;
        x <<= 1;
        x += quadrant_x(quadrant);
        y <<= 1;
        y += quadrant_y(quadrant);
        auto child = point_destination->child(quadrant);
        if (not child)
        {
            point_destination->create_child(quadrant, domain, x, y, depth);
            child = point_destination->child(quadrant);
        }
        point_destination = child;
    }
    point_destination->insert(point_id);
    return point_destination;
}

inline std::vector<const point_quadtree::Node*> initialize_points(point_quadtree::Node& root
    , const std::vector<primitives::morton_key_t>& morton_keys
    , const point_quadtree::Domain& domain)
{
    std::vector<const point_quadtree::Node*> leaf_nodes(morton_keys.size(), nullptr);
    for (primitives::point_id_t i {0}; i < morton_keys.size(); ++i)
    {
        const auto node {point_quadtree::insert_point(morton_keys, i, &root, domain)};
        leaf_nodes[i] = node;
    }
    check::all_true(leaf_nodes, "node assignments to every point");
    return leaf_nodes;
}

} // namespace point_quadtree

