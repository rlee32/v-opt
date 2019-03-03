#pragma once

// Children are indexed by Morton key quadrant.

#include "VMove.h"
#include "morton_keys.h"
#include <DistanceCalculator.h>
#include <primitives.h>

#include <algorithm> // min, max, find, max_element
#include <array>
#include <iostream>
#include <limits> // numeric_limits
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace point_quadtree {

class Node
{
    using ChildArray = std::array<std::unique_ptr<Node>, 4>;
public:
    Node(Node* parent, const Domain&, primitives::grid_t x, primitives::grid_t y, primitives::depth_t);

    const ChildArray& children() const { return m_children; }
    Node* child(primitives::quadrant_t q) { return m_children[q].get(); }

    const std::vector<primitives::point_id_t>& points() const { return m_points; }

    Node* parent() { return m_parent; }
    const Node* parent() const { return m_parent; }

    void create_child(primitives::quadrant_t, const Domain&
        , primitives::grid_t x, primitives::grid_t y, primitives::depth_t);

    primitives::grid_t x() const { return m_x; }
    primitives::grid_t y() const { return m_y; }
    primitives::space_t xmin() const { return m_xmin; }
    primitives::space_t xmax() const { return m_xmax; }
    primitives::space_t ymin() const { return m_ymin; }
    primitives::space_t ymax() const { return m_ymax; }
    void xmin(primitives::space_t c) { m_xmin = c; }
    void xmax(primitives::space_t c) { m_xmax = c; }
    void ymin(primitives::space_t c) { m_ymin = c; }
    void ymax(primitives::space_t c) { m_ymax = c; }

    void insert(primitives::point_id_t i);
    // Returns the node that encompasses the circle with center x, y and radius min_radius.
    const Node* expand(primitives::space_t x, primitives::space_t y
        , primitives::space_t min_radius) const;

    primitives::length_t max_segment_length() const { return m_max_segment_length; }

    void remove_segment(std::vector<primitives::quadrant_t>::const_iterator next_quadrant
        , const std::vector<primitives::quadrant_t>::const_iterator quadrant_end
        , primitives::length_t length);
    void add_segment(std::vector<primitives::quadrant_t>::const_iterator next_quadrant
        , std::vector<primitives::quadrant_t>::const_iterator quadrant_end
        , primitives::length_t length);
    VMove search(primitives::point_id_t i
        , const std::vector<primitives::point_id_t>& next
        , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents
        , const DistanceCalculator&
        , const std::vector<primitives::length_t>& next_lengths
        , primitives::length_t old_segments_length) const;
    VMove search_perturbation(primitives::point_id_t i
        , const std::vector<primitives::point_id_t>& next
        , const DistanceCalculator& dc
        , primitives::length_t min_old_segment_length) const;

private:
    Node* m_parent{nullptr};
    ChildArray m_children; // index corresponds to Morton order quadrant.

    // max segment length in this node and children nodes.
    primitives::length_t m_max_segment_length {0};

    // points immediately under this node (not under children).
    std::vector<primitives::point_id_t> m_points;

    std::vector<primitives::length_t> m_segment_lengths;

    primitives::grid_t m_x{0};
    primitives::grid_t m_y{0};

    primitives::space_t m_xmin{0};
    primitives::space_t m_ymin{0};
    primitives::space_t m_xmax{0};
    primitives::space_t m_ymax{0};

};

} // namespace point_quadtree
