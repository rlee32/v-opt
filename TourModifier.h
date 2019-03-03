#pragma once

#include "Connection.h"
#include "DistanceCalculator.h"
#include "constants.h"
#include "primitives.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <utility> // std::swap
#include <vector>

class TourModifier
{
    using Adjacents = std::array<primitives::point_id_t, 2>;
public:
    TourModifier(const std::vector<primitives::point_id_t>& initial_tour);

    void initialize(const std::vector<primitives::point_id_t>& initial_tour);
    std::vector<primitives::point_id_t> current_tour() const;
    primitives::point_id_t next(primitives::point_id_t i) const { return m_next[i]; }
    const std::vector<primitives::point_id_t>& next() const { return m_next; }

    primitives::length_t current_length(const DistanceCalculator& dc) const;
    const std::vector<Adjacents>& adjacents() const { return m_adjacents; }

private:
    std::vector<Adjacents> m_adjacents;
    std::vector<primitives::point_id_t> m_next;

    void update_next();

    primitives::point_id_t get_other(primitives::point_id_t point, primitives::point_id_t adjacent) const;
    void create_adjacency(const Connection& c);
    void create_adjacency(primitives::point_id_t point1, primitives::point_id_t point2);
    void fill_adjacent(primitives::point_id_t point, primitives::point_id_t new_adjacent);
    void break_adjacency(const Connection& c);
    void break_adjacency(primitives::point_id_t point1, primitives::point_id_t point2);
    void vacate_adjacent_slot(primitives::point_id_t point, primitives::point_id_t adjacent, int slot);
};
