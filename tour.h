#pragma once

// These routines transform data structures relating to point traversal ordering and arrangement.

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

} // namespace tour

