#include "Node.h"

namespace point_quadtree {

Node::Node(Node* parent, const Domain& domain
    , primitives::grid_t x, primitives::grid_t y, primitives::depth_t depth)
    : m_parent(parent)
    , m_x(x)
    , m_y(y)
    , m_xmin(x * domain.xdim(depth))
    , m_ymin(y * domain.ydim(depth))
    , m_xmax((x + 1) * domain.xdim(depth))
    , m_ymax((y + 1) * domain.ydim(depth))
{
}

void Node::reset_max_segment_lengths()
{
    m_max_segment_length = 0;
    for (const auto& unique_ptr : m_children)
    {
        if (unique_ptr)
        {
            unique_ptr->reset_max_segment_lengths();
        }
    }
}

void Node::reset_segments()
{
    m_max_segment_length = 0;
    m_segment_lengths.clear();
    for (const auto& unique_ptr : m_children)
    {
        if (unique_ptr)
        {
            unique_ptr->reset_segments();
        }
    }
}

void Node::search_perturbation(const primitives::point_id_t i
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<primitives::length_t>& next_lengths
    , const DistanceCalculator& dc
    , const primitives::length_t min_adjacent_length
    , const primitives::length_t new_adjacent_length
    , std::vector<VMove>& perturbations) const
{
    for (auto p : m_points)
    {
        if (p == i or next[p] == i)
        {
            continue;
        }
        auto min_new_length
        {
            std::min(
                {
                    dc.compute_length(i, p)
                    , dc.compute_length(i, next[p])
                    , new_adjacent_length
                }
            )
        };
        const auto min_old_length = std::min(min_adjacent_length, next_lengths[p]);
        if (min_new_length < min_old_length)
        {
            auto improvement {min_old_length - min_new_length};
            perturbations.push_back({i, p, improvement});
        }
    }
    for (const auto& unique_ptr : m_children)
    {
        if (unique_ptr)
        {
            unique_ptr->search_perturbation(i, next, next_lengths, dc, min_adjacent_length, new_adjacent_length, perturbations);
        }
    }
}

void Node::search_perturbation_lax(const primitives::point_id_t i
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<primitives::length_t>& next_lengths
    , const DistanceCalculator& dc
    , const primitives::length_t max_adjacent_length
    , const primitives::length_t new_adjacent_length
    , std::vector<VMove>& perturbations) const
{
    for (auto p : m_points)
    {
        if (p == i or next[p] == i)
        {
            continue;
        }
        auto min_new_length
        {
            std::min(
                {
                    dc.compute_length(i, p)
                    , dc.compute_length(i, next[p])
                    , new_adjacent_length
                }
            )
        };
        const auto max_old_length = std::max(max_adjacent_length, next_lengths[p]);
        if (min_new_length < max_old_length)
        {
            auto improvement {max_old_length - min_new_length};
            perturbations.push_back({i, p, improvement});
        }
    }
    for (const auto& unique_ptr : m_children)
    {
        if (unique_ptr)
        {
            unique_ptr->search_perturbation(i, next, next_lengths, dc, max_adjacent_length, new_adjacent_length, perturbations);
        }
    }
}
VMove Node::search(primitives::point_id_t i
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , const DistanceCalculator& dc
    , const std::vector<primitives::length_t>& next_lengths
    , primitives::length_t old_segments_length) const
{
    VMove move;
    for (auto p : m_points)
    {
        if (p == i or next[p] == i)
        {
            continue;
        }
        auto reduction {old_segments_length + next_lengths[p]};
        auto new_length {dc.compute_length(i, p)};
        if (new_length > reduction)
        {
            continue;
        }
        new_length += dc.compute_length(i, next[p]);
        if (new_length > reduction)
        {
            continue;
        }
        new_length += dc.compute_length(adjacents[i][0], adjacents[i][1]);
        if (new_length < reduction)
        {
            auto improvement {reduction - new_length};
            move.apply({i, p, improvement});
        }
    }
    for (const auto& unique_ptr : m_children)
    {
        if (unique_ptr)
        {
            move.apply(unique_ptr->search(
                i, next, adjacents, dc, next_lengths, old_segments_length));
        }
    }
    return move;
}

VMove Node::search(primitives::point_id_t i
    , const std::vector<primitives::point_id_t>& next
    , const std::vector<std::array<primitives::point_id_t, 2>>& adjacents
    , const DistanceCalculator& dc
    , const std::vector<primitives::length_t>& next_lengths
    , primitives::length_t old_segments_length
    , const Segment& permanent_segment) const
{
    VMove move;
    for (auto p : m_points)
    {
        if (p == i or next[p] == i)
        {
            continue;
        }
        const bool removes_permanent {permanent_segment.same(p, next[p])
            or permanent_segment.same(i, adjacents[i][0])
            or permanent_segment.same(i, adjacents[i][1])};
        if (removes_permanent)
        {
            continue;
        }
        auto reduction {old_segments_length + next_lengths[p]};
        auto new_length {dc.compute_length(i, p)};
        if (new_length > reduction)
        {
            continue;
        }
        new_length += dc.compute_length(i, next[p]);
        if (new_length > reduction)
        {
            continue;
        }
        new_length += dc.compute_length(adjacents[i][0], adjacents[i][1]);
        if (new_length < reduction)
        {
            auto improvement {reduction - new_length};
            move.apply({i, p, improvement});
        }
    }
    for (const auto& unique_ptr : m_children)
    {
        if (unique_ptr)
        {
            move.apply(unique_ptr->search(
                i, next, adjacents, dc, next_lengths, old_segments_length));
        }
    }
    return move;
}

void Node::remove_segment(
    std::vector<primitives::quadrant_t>::const_iterator next_quadrant
    , const std::vector<primitives::quadrant_t>::const_iterator quadrant_end
    , primitives::length_t length)
{
    const bool remove_here {next_quadrant == quadrant_end};
    if (remove_here)
    {
        const auto it {std::find(std::cbegin(m_segment_lengths)
            , std::cend(m_segment_lengths)
            , length)};
        if (it == std::cend(m_segment_lengths))
        {
            std::cout << __func__
                << ": error: tried to erase a length that does not exist."
                << std::endl;
            std::abort();
        }
        m_segment_lengths.erase(it);
    }
    else
    {
        const auto& child {m_children[*next_quadrant]};
        if (not child)
        {
            std::cout << __func__
                << ": error: child does not exist for segment pathway." << std::endl;
            std::abort();
        }
        child->remove_segment(++next_quadrant, quadrant_end, length);
    }
    if (length > m_max_segment_length)
    {
        std::cout << __func__
            << ": error: attempted to remove a segment length longer than the maximum."
            << std::endl;
        std::abort();
    }
    const bool need_update {length == m_max_segment_length};
    if (need_update)
    {
        if (m_segment_lengths.size() > 0)
        {
            m_max_segment_length = *std::max_element(std::cbegin(m_segment_lengths)
                , std::cend(m_segment_lengths));
        }
        for (const auto& unique_ptr : m_children)
        {
            if (unique_ptr)
            {
                m_max_segment_length = std::max(m_max_segment_length
                    , unique_ptr->max_segment_length());
            }
        }
    }
}

void Node::add_segment(const Segment& s, const std::vector<primitives::morton_key_t>& morton_keys)
{
    const auto insertion_path {point_quadtree::morton_keys::segment_insertion_path(morton_keys[s.min], morton_keys[s.max])};
    add_segment(std::cbegin(insertion_path), std::cend(insertion_path), s.length);
}

void Node::add_segment(std::vector<primitives::quadrant_t>::const_iterator next_quadrant
    , const std::vector<primitives::quadrant_t>::const_iterator quadrant_end
    , primitives::length_t length)
{
    const bool add_here {next_quadrant == quadrant_end};
    if (add_here)
    {
        m_segment_lengths.push_back(length);
    }
    else
    {
        const auto& child {m_children[*next_quadrant]};
        if (not child)
        {
            std::cout << __func__
                << ": error: child does not exist for segment pathway." << std::endl;
            std::abort();
        }
        child->add_segment(++next_quadrant, quadrant_end, length);
    }
    m_max_segment_length = std::max(m_max_segment_length, length);
}

void Node::insert(primitives::point_id_t i)
{
    m_points.push_back(i);
}

void Node::create_child(primitives::quadrant_t quadrant
    , const Domain& domain
    , primitives::grid_t x, primitives::grid_t y, primitives::depth_t depth)
{
    if (m_children[quadrant])
    {
        return;
    }
    m_children[quadrant] = std::make_unique<Node>(this, domain, x, y, depth);
}

const Node* Node::expand(primitives::space_t x, primitives::space_t y
    , primitives::space_t old_segments_length) const
{
    if (not m_parent)
    {
        return this;
    }
    // Assumption: x, y is inside the current node.
    auto margin_dx {std::min(x - m_xmin, m_xmax - x)};
    auto margin_dy {std::min(y - m_ymin, m_ymax - y)};
    auto margin_sq {margin_dx * margin_dx + margin_dy * margin_dy};
    auto min_radius {old_segments_length + m_max_segment_length};
    if (margin_sq >= min_radius * min_radius)
    {
        return this;
    }
    return m_parent->expand(x, y, min_radius);
}

const Node* Node::expand_simple(primitives::space_t x, primitives::space_t y
    , primitives::space_t min_radius) const
{
    if (not m_parent)
    {
        return this;
    }
    // Assumption: x, y is inside the current node.
    auto margin_dx {std::min(x - m_xmin, m_xmax - x)};
    auto margin_dy {std::min(y - m_ymin, m_ymax - y)};
    auto margin_sq {margin_dx * margin_dx + margin_dy * margin_dy};
    if (margin_sq >= min_radius * min_radius)
    {
        return this;
    }
    return m_parent->expand(x, y, min_radius);
}

} // namespace point_quadtree
