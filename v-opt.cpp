
#include "DistanceCalculator.h"
#include "TourModifier.h"
#include "check.h"
#include "fileio/PointSet.h"
#include "fileio/fileio.h"
#include "point_quadtree/Domain.h"
#include "point_quadtree/Node.h"
#include "point_quadtree/morton_keys.h"
#include "point_quadtree/point_quadtree.h"
#include "primitives.h"
#include "solver.h"
// #include "solver.h"
// #include "utility.h"

#include <iostream>

int main(int argc, const char** argv)
{
    if (argc < 2)
    {
        std::cout << "Arguments: point_set_file_path optional_tour_file_path" << std::endl;
        return 0;
    }
    // Read input files.
    const fileio::PointSet point_set(argv[1]);
    const auto initial_tour = fileio::initial_tour(argc, argv, point_set.count());

    // Initialize distance table.
    DistanceCalculator dc(point_set.x(), point_set.y());
    TourModifier tour_modifier(initial_tour);
    const auto initial_tour_length = tour_modifier.current_length(dc);
    std::cout << "Initial tour length: " << initial_tour_length << std::endl;

    // quadtree
    point_quadtree::Domain domain(point_set.x(), point_set.y());
    const auto& x {point_set.x()};
    const auto& y {point_set.y()};
    const auto morton_keys {point_quadtree::morton_keys::compute_point_morton_keys(x, y, domain)};
    point_quadtree::Node root(nullptr, domain, 0, 0, 0);
    std::vector<const point_quadtree::Node*> leaf_nodes(point_set.count(), nullptr);
    for (primitives::point_id_t i {0}; i < point_set.count(); ++i)
    {
        const auto node {point_quadtree::insert_point(morton_keys, i, &root, domain)};
        leaf_nodes[i] = node;
    }
    check::all_true(leaf_nodes, "node assignments to every point");

    // initial addition of segments.
    const auto& next {tour_modifier.next()};
    for (primitives::point_id_t i {0}; i < point_set.count(); ++i)
    {
        const auto segment_length {dc.compute_length(i, next[i])};
        const auto segment_insertion_path
            {point_quadtree::morton_keys::segment_insertion_path(
                morton_keys[i], morton_keys[next[i]])};
        root.add_segment(std::cbegin(segment_insertion_path)
            , std::cend(segment_insertion_path), segment_length);
    }

    // now that max segment lengths are known, determine search nodes for each point.
    const auto segment_lengths {solver::compute_segment_lengths(dc, tour_modifier.adjacents())};
    std::vector<const point_quadtree::Node*> search_nodes(point_set.count(), nullptr);
    for (primitives::point_id_t i {0}; i < point_set.count(); ++i)
    {
        auto old_segments_length {segment_lengths[i][0] + segment_lengths[i][1]};
        search_nodes[i] = {leaf_nodes[i]->expand(x[i], y[i], old_segments_length)};
    }

    // call search on each node.
    const auto next_lengths {solver::compute_next_lengths(dc, next)};
    std::vector<VMove> perturbations;
    std::vector<VMove> improvements;
    for (primitives::point_id_t i {0}; i < point_set.count(); ++i)
    {
        auto old_segments_length {segment_lengths[i][0] + segment_lengths[i][1]};
        const auto best_improvement {search_nodes[i]->search(i
            , next, tour_modifier.adjacents(), dc, next_lengths, old_segments_length)};
        if (best_improvement.improvement > 0)
        {
            improvements.push_back(best_improvement);
        }
        const auto best_perturbation {search_nodes[i]->search_perturbation(i, next, dc
            , std::min(segment_lengths[i][0], segment_lengths[i][1]))};
        if (best_perturbation.improvement > 0)
        {
            perturbations.push_back(best_perturbation);
        }
    }
    std::cout << "Found " << improvements.size() << " improvements." << std::endl;
    std::cout << "Found " << perturbations.size() << " perturbations." << std::endl;
    std::cout << "Search finished." << std::endl;
    return 0;
}
