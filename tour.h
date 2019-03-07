#pragma once

// These routines transform data structures relating to point traversal ordering and arrangement.

#include "Segment.h"
#include "VMove.h"
#include "constants.h"
#include "primitives.h"

#include <array>
#include <cstdlib> // abort
#include <vector>

namespace tour {

inline std::vector<primitives::point_id_t> compute_ordered_points(const std::vector<primitives::point_id_t>& next)
{
    std::vector<primitives::point_id_t> ordered_points(1, 0);
    while (ordered_points.size() < next.size())
    {
        ordered_points.push_back(next[ordered_points.back()]);
    }
    return ordered_points;
}

inline void fill_adjacent(std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , primitives::point_id_t point
    , primitives::point_id_t new_adjacent)
{

    if (adjacents[point].front() == constants::invalid_point)
    {
        adjacents[point].front() = new_adjacent;
    }
    else if (adjacents[point].back() == constants::invalid_point)
    {
        adjacents[point].back() = new_adjacent;
    }
    else
    {
        std::cout << __func__ << ": error: failed to assign adjacency." << std::endl;
        std::abort();
    }
}

inline void create_adjacency(std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , primitives::point_id_t point1
    , primitives::point_id_t point2)
{
    fill_adjacent(adjacents, point1, point2);
    fill_adjacent(adjacents, point2, point1);
}

inline void update_adjacents(std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , const std::vector<primitives::point_id_t>& ordered_points)
{
    auto prev = ordered_points.back();
    for (auto p : ordered_points)
    {
        create_adjacency(adjacents, p, prev);
        prev = p;
    }
}

inline std::vector<std::array<primitives::point_id_t, 2>> compute_adjacents(const std::vector<primitives::point_id_t>& ordered_points)
{
    std::vector<std::array<primitives::point_id_t, 2>> adjacents(ordered_points.size()
        , {constants::invalid_point, constants::invalid_point});
    update_adjacents(adjacents, ordered_points);
    return adjacents;
}

inline void update_next(std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents)
{
    primitives::point_id_t current {0};
    next[current] = adjacents[current].front();
    do
    {
        auto prev = current;
        current = next[current];
        const auto& a = adjacents[current];
        if (a.front() == prev)
        {
            next[current] = a.back();
        }
        else
        {
            next[current] = a.front();
        }
    } while (current != 0); // tour cycle condition.
}

inline std::vector<primitives::point_id_t> compute_next(const std::vector<std::array<primitives::point_id_t, 2>>& adjacents)
{
    std::vector<primitives::point_id_t> next(adjacents.size(), constants::invalid_point);
    update_next(next, adjacents);
    return next;
}

inline void vacate_adjacent_slot(std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , primitives::point_id_t point, primitives::point_id_t adjacent)
{
    if (adjacents[point][0] == adjacent)
    {
        adjacents[point][0] = constants::invalid_point;
    }
    else if (adjacents[point][1] == adjacent)
    {
        adjacents[point][1] = constants::invalid_point;
    }
    else
    {
        std::cout << __func__ << ": error: attempted to remove non-existent adjacency." << std::endl;
        std::abort();
    }
}

inline void break_adjacency(std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , primitives::point_id_t point1, primitives::point_id_t point2)
{
    vacate_adjacent_slot(adjacents, point1, point2);
    vacate_adjacent_slot(adjacents, point2, point1);
}

inline void apply_move(const VMove& move
    , std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , std::vector<primitives::point_id_t>& next)
{
    const auto old_adjacents {adjacents[move.i]};
    break_adjacency(adjacents, move.i, adjacents[move.i][0]);
    break_adjacency(adjacents, move.i, adjacents[move.i][1]);
    break_adjacency(adjacents, move.j, next[move.j]);
    create_adjacency(adjacents, move.i, move.j);
    create_adjacency(adjacents, move.i, next[move.j]);
    create_adjacency(adjacents, old_adjacents[0], old_adjacents[1]);
    update_next(next, adjacents);
}

inline std::vector<primitives::point_id_t> perturb(const VMove& move, const std::vector<primitives::point_id_t>& ordered_points)
{
    auto adjacents {tour::compute_adjacents(ordered_points)};
    auto next {tour::compute_next(adjacents)};
    apply_move(move, adjacents, next);
    return compute_ordered_points(next);
}

inline primitives::length_t compute_length(
    const std::vector<primitives::point_id_t>& ordered_points
    , const DistanceCalculator& dc)
{
    auto prev {ordered_points.back()};
    primitives::length_t length {0};
    for (auto p : ordered_points)
    {
        length += dc.compute_length(prev, p);
        prev = p;
    }
    return length;
}

inline std::vector<std::array<primitives::length_t, 2>> compute_adjacent_lengths(
    const std::vector<std::array<primitives::point_id_t, 2>>& adjacent_pairs
    , const DistanceCalculator& dc)
{
    std::vector<std::array<primitives::length_t, 2>> adjacent_lengths(adjacent_pairs.size(), {0, 0});
    for (size_t i {0}; i < adjacent_pairs.size(); ++i)
    {
        adjacent_lengths[i][0] = dc.compute_length(i, adjacent_pairs[i][0]);
        adjacent_lengths[i][1] = dc.compute_length(i, adjacent_pairs[i][1]);
    }
    return adjacent_lengths;
}

inline std::vector<primitives::length_t> compute_next_lengths(const std::vector<primitives::point_id_t>& next
    , const DistanceCalculator& dc)
{
    std::vector<primitives::length_t> next_lengths(next.size(), 0);
    for (primitives::point_id_t i {0}; i < next.size(); ++i)
    {
        next_lengths[i] = dc.compute_length(i, next[i]);
    }
    return next_lengths;
}

inline std::vector<Segment> compute_segments(
    const std::vector<primitives::point_id_t>& next
    , const DistanceCalculator& dc)
{
    std::vector<Segment> segments;
    for (primitives::point_id_t i {0}; i < next.size(); ++i)
    {
        segments.emplace_back(i, next[i], dc);
    }
    return segments;
}

inline void verify(const std::vector<primitives::point_id_t>& ordered_points)
{
    std::vector<bool> seen(ordered_points.size(), false);
    for (auto point : ordered_points)
    {
        if (seen[point])
        {
            if (point >= ordered_points.size())
            {
                std::cout << __func__ << ": error: invalid point id in tour." << std::endl;
                std::abort();
            }
            std::cout << __func__ << ": error: repeated visits in tour." << std::endl;
            std::abort();
        }
        seen[point] = true;
    }
    for (auto s : seen)
    {
        if (not s)
        {
            std::cout << __func__ << ": error: not all points were visited." << std::endl;
            std::abort();
        }
    }
    std::cout << "Tour verified." << std::endl;
}

} // namespace tour

