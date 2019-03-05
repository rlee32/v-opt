#pragma once

#include "DistanceCalculator.h"
#include "Segment.h"
#include "Solution.h"
#include "VMove.h"
#include "check.h"
#include "constants.h"
#include "point_quadtree/Domain.h"
#include "point_quadtree/Node.h"
#include "point_quadtree/morton_keys.h"
#include "point_quadtree/point_quadtree.h"
#include "primitives.h"
#include "tour.h"

#include <array>
#include <vector>

namespace solver {

inline std::vector<primitives::length_t> compute_next_lengths(const DistanceCalculator& dc
    , const std::vector<primitives::point_id_t>& next)
{
    std::vector<primitives::length_t> next_lengths(next.size(), 0);
    for (size_t i {0}; i < next.size(); ++i)
    {
        next_lengths[i] = dc.compute_length(i, next[i]);
    }
    return next_lengths;
}

inline std::vector<std::array<primitives::length_t, 2>> compute_segment_lengths(
    const DistanceCalculator& dc, const std::vector<std::array<primitives::point_id_t, 2>>& adjacent_pairs)
{
    std::vector<std::array<primitives::length_t, 2>> segment_lengths(adjacent_pairs.size(), {0, 0});
    for (size_t i {0}; i < adjacent_pairs.size(); ++i)
    {
        segment_lengths[i][0] = dc.compute_length(i, adjacent_pairs[i][0]);
        segment_lengths[i][1] = dc.compute_length(i, adjacent_pairs[i][1]);
    }
    return segment_lengths;
}

inline std::vector<const point_quadtree::Node*> get_leaf_nodes(point_quadtree::Node& root
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

inline void initialize_max_segment_lengths(point_quadtree::Node& root
    , const std::vector<primitives::morton_key_t>& morton_keys
    , const std::vector<primitives::point_id_t>& next
    , const DistanceCalculator& dc)
{
    root.reset_max_segment_lengths();
    // initial addition of segments.
    for (primitives::point_id_t i {0}; i < next.size(); ++i)
    {
        const auto segment_length {dc.compute_length(i, next[i])};
        const auto segment_insertion_path
            {point_quadtree::morton_keys::segment_insertion_path(
                morton_keys[i], morton_keys[next[i]])};
        root.add_segment(std::cbegin(segment_insertion_path)
            , std::cend(segment_insertion_path), segment_length);
    }
}

inline void update_search_nodes(
    std::vector<const point_quadtree::Node*>& search_nodes
    , const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<std::array<primitives::length_t, 2>>& segment_lengths)
{
    for (primitives::point_id_t i {0}; i < x.size(); ++i)
    {
        auto old_segments_length {segment_lengths[i][0] + segment_lengths[i][1]};
        search_nodes[i] = {leaf_nodes[i]->expand(x[i], y[i], old_segments_length)};
    }
}

inline std::vector<const point_quadtree::Node*> get_search_nodes(
    const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<std::array<primitives::length_t, 2>>& segment_lengths)
{
    std::vector<const point_quadtree::Node*> search_nodes(x.size(), nullptr);
    update_search_nodes(search_nodes, x, y, leaf_nodes, segment_lengths);
    return search_nodes;
}

inline std::vector<VMove> find_perturbations(const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const point_quadtree::Domain& domain
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , const DistanceCalculator& dc)
{
    const auto morton_keys {point_quadtree::morton_keys::compute_point_morton_keys(x, y, domain)};
    point_quadtree::Node root(nullptr, domain, 0, 0, 0);
    const auto leaf_nodes{get_leaf_nodes(root, morton_keys, domain)};
    initialize_max_segment_lengths(root, morton_keys, next, dc);
    const auto segment_lengths {compute_segment_lengths(dc, adjacents)};
    const auto search_nodes {get_search_nodes(x, y, leaf_nodes, segment_lengths)};

    // call search on each node.
    std::vector<VMove> perturbations;
    for (primitives::point_id_t i {0}; i < x.size(); ++i)
    {
        search_nodes[i]->search_perturbation(i, next, dc
            , std::min(segment_lengths[i][0], segment_lengths[i][1]), perturbations);
    }
    return perturbations;
}

inline std::vector<VMove> find_improvements(const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const point_quadtree::Domain& domain
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , const DistanceCalculator& dc)
{
    // quadtree
    const auto morton_keys {point_quadtree::morton_keys::compute_point_morton_keys(x, y, domain)};
    point_quadtree::Node root(nullptr, domain, 0, 0, 0);
    const auto leaf_nodes{get_leaf_nodes(root, morton_keys, domain)};
    initialize_max_segment_lengths(root, morton_keys, next, dc);
    const auto segment_lengths {compute_segment_lengths(dc, adjacents)};
    const auto search_nodes {get_search_nodes(x, y, leaf_nodes, segment_lengths)};

    // call search on each node.
    const auto next_lengths {compute_next_lengths(dc, next)};
    std::vector<VMove> improvements;
    for (primitives::point_id_t i {0}; i < x.size(); ++i)
    {
        auto old_segments_length {segment_lengths[i][0] + segment_lengths[i][1]};
        const auto best_improvement {search_nodes[i]->search(i
            , next, adjacents, dc, next_lengths, old_segments_length)};
        if (best_improvement.improvement > 0)
        {
            improvements.push_back(best_improvement);
        }
    }
    return improvements;
}

inline VMove find_best_improvement(const std::vector<const point_quadtree::Node*>& search_nodes
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , const DistanceCalculator& dc)
{
    // call search on each node.
    const auto next_lengths {compute_next_lengths(dc, next)};
    VMove best_move;
    for (primitives::point_id_t i {0}; i < next.size(); ++i)
    {
        auto old_segments_length {next_lengths[i]};
        if (next[adjacents[i][0]] == i)
        {
            old_segments_length += next_lengths[adjacents[i][0]];
        }
        else if (next[adjacents[i][1]] == i)
        {
            old_segments_length += next_lengths[adjacents[i][1]];
        }
        else
        {
            std::cout << __func__ << ": error: inconsistency between next and adjacents" << std::endl;
            std::abort();
        }
        const auto move {search_nodes[i]->search(i
            , next, adjacents, dc, next_lengths, old_segments_length)};
        best_move.apply(move);
    }
    return best_move;
}

inline std::array<Segment, 3> compute_new_segments(const VMove& move
    , const DistanceCalculator& dc
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents)
{
    return
    {{
        {move.i, move.j, dc}
        , {move.i, next[move.j], dc}
        , {adjacents[move.i][0], adjacents[move.i][1], dc}
    }};
}

inline std::array<Segment, 3> compute_old_segments(const VMove& move
    , const DistanceCalculator& dc
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents)
{
    return
    {{
        {move.j, next[move.j], dc}
        , {move.i, adjacents[move.i][0], dc}
        , {move.i, adjacents[move.i][1], dc}
    }};
}

inline void remove_segment_length(std::vector<std::array<primitives::length_t, 2>>& segment_lengths
    , primitives::point_id_t point, primitives::length_t length)
{
    if (segment_lengths[point][0] == length)
    {
        segment_lengths[point][0] = 0;
    }
    else if (segment_lengths[point][1] == length)
    {
        segment_lengths[point][1] = 0;
    }
    else
    {
        std::cout << __func__ << ": error: attempted to remove non-existent segment length." << std::endl;
        std::abort();
    }
}

inline void add_segment_length(std::vector<std::array<primitives::length_t, 2>>& segment_lengths
    , primitives::point_id_t point, primitives::length_t length)
{
    if (segment_lengths[point][0] == 0)
    {
        segment_lengths[point][0] = length;
    }
    else if (segment_lengths[point][1] == 0)
    {
        segment_lengths[point][1] = length;
    }
    else
    {
        std::cout << __func__ << ": error: attempted to add segment length to fully-occupied point." << std::endl;
        std::abort();
    }
}

inline void update_segment_lengths(const std::array<Segment, 3>& old_segments
    , const std::array<Segment, 3>& new_segments
    , std::vector<std::array<primitives::length_t, 2>>& segment_lengths)
{
    for (const auto& s : old_segments)
    {
        remove_segment_length(segment_lengths, s.min, s.length);
        remove_segment_length(segment_lengths, s.max, s.length);
    }
    for (const auto& s : new_segments)
    {
        add_segment_length(segment_lengths, s.min, s.length);
        add_segment_length(segment_lengths, s.max, s.length);
    }
}

inline Solution hill_climb(
    const std::vector<primitives::point_id_t>& ordered_points
    , const std::vector<primitives::morton_key_t>& morton_keys
    , point_quadtree::Node& root
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const DistanceCalculator& dc
    )
{
    auto adjacents {tour::compute_adjacents(ordered_points)};
    auto next {tour::compute_next(adjacents)};
    initialize_max_segment_lengths(root, morton_keys, next, dc);
    auto segment_lengths {compute_segment_lengths(dc, adjacents)};
    auto search_nodes {get_search_nodes(x, y, leaf_nodes, segment_lengths)};

    while (true)
    {
        const auto best_move {find_best_improvement(search_nodes, next, adjacents, dc)};
        if (best_move.improvement == 0)
        {
            break;
        }
        const auto old_segments {compute_old_segments(best_move, dc, next, adjacents)};
        for (const auto& s : old_segments)
        {
            const auto segment_insertion_path {point_quadtree::morton_keys::segment_insertion_path(
                morton_keys[s.min], morton_keys[s.max])};
            root.remove_segment(std::cbegin(segment_insertion_path), std::cend(segment_insertion_path), s.length);
        }
        const auto new_segments {compute_new_segments(best_move, dc, next, adjacents)};
        for (const auto& s : new_segments)
        {
            const auto segment_insertion_path {point_quadtree::morton_keys::segment_insertion_path(
                morton_keys[s.min], morton_keys[s.max])};
            root.add_segment(std::cbegin(segment_insertion_path), std::cend(segment_insertion_path), s.length);
        }
        update_segment_lengths(old_segments, new_segments, segment_lengths);
        update_search_nodes(search_nodes, x, y, leaf_nodes, segment_lengths);
        tour::apply_move(best_move, adjacents, next);
    }
    return {tour::compute_ordered_points(next)};
}

} // namespace solver
