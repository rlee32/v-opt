#include "TourModifier.h"

TourModifier::TourModifier(const std::vector<primitives::point_id_t>& initial_tour)
    : m_adjacents(initial_tour.size(), {constants::invalid_point, constants::invalid_point})
    , m_next(initial_tour.size(), constants::invalid_point)
{
    initialize(initial_tour);
}

void TourModifier::move(const VMove& move)
{
    const auto old_adjacents {m_adjacents[move.i]};
    break_adjacency(move.i, m_adjacents[move.i][0]);
    break_adjacency(move.i, m_adjacents[move.i][1]);
    break_adjacency(move.j, m_next[move.j]);
    create_adjacency(move.i, move.j);
    create_adjacency(move.i, m_next[move.j]);
    create_adjacency(old_adjacents[0], old_adjacents[1]);
    update_next();
}

primitives::length_t TourModifier::current_length(const DistanceCalculator& dc) const
{
    primitives::point_id_t current {0};
    constexpr primitives::point_id_t start {0};
    primitives::length_t length {0};
    auto remaining {m_next.size()};
    do
    {
        if (remaining == 0)
        {
            std::cout << __func__ << ": error: summing more lengths than there are points." << std::endl;
            std::abort();
        }
        const auto next {m_next[current]};
        length += dc.compute_length(current, next);
        current = next;
        --remaining;
    } while (current != start);
    return length;
}

void TourModifier::initialize(const std::vector<primitives::point_id_t>& initial_tour)
{
    auto prev = initial_tour.back();
    for (auto p : initial_tour)
    {
        create_adjacency(p, prev);
        prev = p;
    }
    update_next();
}

std::vector<primitives::point_id_t> TourModifier::current_tour() const
{
    std::vector<primitives::point_id_t> points(m_next.size(), constants::invalid_point);
    constexpr primitives::point_id_t start_point{0};
    primitives::point_id_t i{start_point};
    int sequence{0};
    do
    {
        points[sequence] = i;
        i = m_next[i];
        ++sequence;
    } while (i != start_point);
    return points;
}

void TourModifier::update_next()
{
    primitives::point_id_t current{0};
    m_next[current] = m_adjacents[current].front();
    do
    {
        auto prev = current;
        current = m_next[current];
        m_next[current] = get_other(current, prev);
    } while (current != 0); // tour cycle condition.
}

primitives::point_id_t TourModifier::get_other(primitives::point_id_t point, primitives::point_id_t adjacent) const
{
    const auto& a = m_adjacents[point];
    if (a.front() == adjacent)
    {
        return a.back();
    }
    else
    {
        return a.front();
    }
}

void TourModifier::create_adjacency(primitives::point_id_t point1, primitives::point_id_t point2)
{
    fill_adjacent(point1, point2);
    fill_adjacent(point2, point1);
}

void TourModifier::fill_adjacent(primitives::point_id_t point, primitives::point_id_t new_adjacent)
{
    if (m_adjacents[point].front() == constants::invalid_point)
    {
        m_adjacents[point].front() = new_adjacent;
    }
    else if (m_adjacents[point].back() == constants::invalid_point)
    {
        m_adjacents[point].back() = new_adjacent;
    }
}

void TourModifier::break_adjacency(primitives::point_id_t point1, primitives::point_id_t point2)
{
    vacate_adjacent_slot(point1, point2, 0);
    vacate_adjacent_slot(point1, point2, 1);
    vacate_adjacent_slot(point2, point1, 0);
    vacate_adjacent_slot(point2, point1, 1);
}

void TourModifier::vacate_adjacent_slot(primitives::point_id_t point, primitives::point_id_t adjacent, int slot)
{
    if (m_adjacents[point][slot] == adjacent)
    {
        m_adjacents[point][slot] = constants::invalid_point;
    }
}

