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

inline void update_search_nodes(
    std::vector<const point_quadtree::Node*>& search_nodes
    , const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<std::array<primitives::length_t, 2>>& adjacent_lengths)
{
    for (primitives::point_id_t i {0}; i < x.size(); ++i)
    {
        auto old_segments_length {adjacent_lengths[i][0] + adjacent_lengths[i][1]};
        search_nodes[i] = {leaf_nodes[i]->expand(x[i], y[i], old_segments_length)};
    }
}

inline std::vector<const point_quadtree::Node*> get_search_nodes(
    const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<std::array<primitives::length_t, 2>>& adjacent_lengths)
{
    std::vector<const point_quadtree::Node*> search_nodes(x.size(), nullptr);
    update_search_nodes(search_nodes, x, y, leaf_nodes, adjacent_lengths);
    return search_nodes;
}

inline VMove find_best_improvement(const std::vector<const point_quadtree::Node*>& search_nodes
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , const DistanceCalculator& dc
    , const Segment& permanent_segment = {})
{
    // call search on each node.
    const auto next_lengths {tour::compute_next_lengths(next, dc)};
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
        if (permanent_segment.length == 0)
        {
            const auto move {search_nodes[i]->search(i
                , next, adjacents, dc, next_lengths, old_segments_length)};
            best_move.apply(move);
        }
        else
        {
            const auto move {search_nodes[i]->search(i
                , next, adjacents, dc, next_lengths, old_segments_length, permanent_segment)};
            best_move.apply(move);
        }
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

inline void remove_segment_length(std::vector<std::array<primitives::length_t, 2>>& adjacent_lengths
    , primitives::point_id_t point, primitives::length_t length)
{
    if (adjacent_lengths[point][0] == length)
    {
        adjacent_lengths[point][0] = 0;
    }
    else if (adjacent_lengths[point][1] == length)
    {
        adjacent_lengths[point][1] = 0;
    }
    else
    {
        std::cout << __func__ << ": error: attempted to remove non-existent segment length." << std::endl;
        std::abort();
    }
}

inline void add_segment_length(
    std::vector<std::array<primitives::length_t, 2>>& segment_lengths
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

inline void apply_move(
    std::vector<const point_quadtree::Node*>& search_nodes
    , std::vector<std::array<primitives::length_t, 2>>& segment_lengths
    , point_quadtree::Node& root
    , std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , std::vector<primitives::point_id_t>& next
    , const VMove& move
    , const std::vector<primitives::morton_key_t>& morton_keys
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const DistanceCalculator& dc)
{
    const auto old_segments {compute_old_segments(move, dc, next, adjacents)};
    for (const auto& s : old_segments)
    {
        const auto segment_insertion_path {point_quadtree::morton_keys::segment_insertion_path(
            morton_keys[s.min], morton_keys[s.max])};
        root.remove_segment(std::cbegin(segment_insertion_path), std::cend(segment_insertion_path), s.length);
    }
    const auto new_segments {compute_new_segments(move, dc, next, adjacents)};
    for (const auto& s : new_segments)
    {
        const auto segment_insertion_path {point_quadtree::morton_keys::segment_insertion_path(
            morton_keys[s.min], morton_keys[s.max])};
        root.add_segment(std::cbegin(segment_insertion_path), std::cend(segment_insertion_path), s.length);
    }
    update_segment_lengths(old_segments, new_segments, segment_lengths);
    update_search_nodes(search_nodes, x, y, leaf_nodes, segment_lengths);
    tour::apply_move(move, adjacents, next);
}

inline std::vector<primitives::point_id_t> hill_climb(
    const std::vector<primitives::point_id_t>& ordered_points
    , const std::vector<primitives::morton_key_t>& morton_keys
    , point_quadtree::Node& root
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const DistanceCalculator& dc
    , const Segment& permanent_segment = {})
{
    auto adjacents {tour::compute_adjacents(ordered_points)};
    auto next {tour::compute_next(adjacents)};
    auto segments {tour::compute_segments(next, dc)};
    root.reset_segments();
    for (const auto& s : segments)
    {
        root.add_segment(s, morton_keys);
    }
    auto segment_lengths {tour::compute_adjacent_lengths(adjacents, dc)};
    auto search_nodes {get_search_nodes(x, y, leaf_nodes, segment_lengths)};

    int iteration {0};
    while (true)
    {
        const auto best_move {find_best_improvement(search_nodes, next, adjacents, dc, permanent_segment)};
        if (best_move.improvement == 0)
        {
            break;
        }

        apply_move(search_nodes, segment_lengths, root, adjacents, next, best_move, morton_keys, leaf_nodes, x, y, dc);
        ++iteration;
        if (constants::verify)
        {
            tour::verify(tour::compute_ordered_points(next));
        }
        if (constants::print_iterations)
        {
            std::cout << "Iteration: " << iteration << " length: " << tour::compute_length(tour::compute_ordered_points(next), dc) << std::endl;
        }
    }
    return tour::compute_ordered_points(next);
}

inline std::vector<VMove> find_perturbations(
    const std::vector<primitives::point_id_t>& ordered_points
    , const std::vector<primitives::morton_key_t>& morton_keys
    , point_quadtree::Node& root
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const DistanceCalculator& dc)
{
    auto adjacents {tour::compute_adjacents(ordered_points)};
    auto next {tour::compute_next(adjacents)};
    auto segments {tour::compute_segments(next, dc)};
    root.reset_segments();
    for (const auto& s : segments)
    {
        root.add_segment(s, morton_keys);
    }
    const auto segment_lengths {tour::compute_adjacent_lengths(adjacents, dc)};
    const auto search_nodes {get_search_nodes(x, y, leaf_nodes, segment_lengths)};

    // call search on each node.
    std::vector<VMove> perturbations;
    const auto next_lengths {tour::compute_next_lengths(next, dc)};
    for (primitives::point_id_t i {0}; i < x.size(); ++i)
    {
        search_nodes[i]->search_perturbation(i
            , next
            , next_lengths
            , dc
            , std::min(segment_lengths[i][0], segment_lengths[i][1])
            , dc.compute_length(adjacents[i][0], adjacents[i][1])
            , perturbations);
    }
    return perturbations;
}

inline std::vector<primitives::point_id_t> perturbed_hill_climb(
    const std::vector<primitives::point_id_t>& ordered_points
    , const std::vector<primitives::morton_key_t>& morton_keys
    , point_quadtree::Node& root
    , const std::vector<const point_quadtree::Node*>& leaf_nodes
    , const std::vector<primitives::space_t>& x
    , const std::vector<primitives::space_t>& y
    , const DistanceCalculator& dc)
{
    auto original_adjacents {tour::compute_adjacents(ordered_points)};
    auto original_next {tour::compute_next(original_adjacents)};
    auto segment_lengths {tour::compute_adjacent_lengths(original_adjacents, dc)};

    // TODO: top-down root search instead of predetermined search nodes.
    std::vector<const point_quadtree::Node*> perturbation_search_nodes(x.size(), nullptr);
    for (primitives::point_id_t i {0}; i < x.size(); ++i)
    {
        auto min_segments_length {std::min(segment_lengths[i][0], segment_lengths[i][1])};
        perturbation_search_nodes[i] = {leaf_nodes[i]->expand_simple(x[i], y[i], min_segments_length)};
    }

    std::vector<VMove> perturbations;
    const auto next_lengths {tour::compute_next_lengths(original_next, dc)};
    for (primitives::point_id_t i {0}; i < x.size(); ++i)
    {
        const auto max_adjacent_length {std::max(segment_lengths[i][0], segment_lengths[i][1])};
        if (max_adjacent_length == 0)
        {
            std::cout << "zero-sized segment" << std::endl;
            std::abort();
        }
        if (max_adjacent_length == 1)
        {
            continue;
        }
        perturbation_search_nodes[i]->search_perturbation_lax(i
            , original_next
            , next_lengths
            , dc
            , max_adjacent_length
            , dc.compute_length(original_adjacents[i][0], original_adjacents[i][1])
            , perturbations);
    }

    std::vector<primitives::point_id_t> best_solution;
    auto best_length {tour::compute_length(ordered_points, dc)};
    for (const auto& perturbation : perturbations)
    {
        const auto perturbed_points {tour::perturb(perturbation, ordered_points)};
        const auto min_old_length
        {
            std::min(
                {
                    next_lengths[perturbation.j]
                    , segment_lengths[perturbation.i][0]
                    , segment_lengths[perturbation.i][1]
                }
            )
        };
        const std::array<Segment, 3> permanent_segments
        {{
            {perturbation.i, perturbation.j, dc}
            , {perturbation.i, original_next[perturbation.j], dc}
            , {original_adjacents[perturbation.i][0], original_adjacents[perturbation.i][1], dc}
        }};
        for (const auto& s : permanent_segments)
        {
            if (s.length < min_old_length)
            {
                auto solution = hill_climb(perturbed_points, morton_keys, root, leaf_nodes, x, y, dc, s);
                auto length {tour::compute_length(solution, dc)};
                if (length < best_length)
                {
                    best_solution = solution;
                    best_length = length;
                }
            }
        }
    }
    return best_solution;
}
nn
} // namespace solver
