
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

inline void local_stats(TourModifier& tour_modifier, const fileio::PointSet& point_set
    , const DistanceCalculator& dc, const point_quadtree::Domain& domain)
{
    const auto improvements {solver::find_improvements(point_set.x(), point_set.y(), domain
        , tour_modifier.next(), tour_modifier.adjacents(), dc)};
    const auto perturbations {solver::find_perturbations(point_set.x(), point_set.y(), domain
        , tour_modifier.next(), tour_modifier.adjacents(), dc)};

    if (improvements.size() > 0)
    {
        tour_modifier.move(improvements.front());
        std::cout << "new tour length: " << tour_modifier.current_length(dc) << std::endl;
    }
    std::cout << "Found " << improvements.size() << " improvements." << std::endl;
    std::cout << "Found " << perturbations.size() << " perturbations." << std::endl;
    std::cout << "Search finished." << std::endl;
}

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
    point_quadtree::Domain domain(point_set.x(), point_set.y());
    TourModifier tour_modifier(initial_tour);
    const auto initial_tour_length = tour_modifier.current_length(dc);
    std::cout << "Initial tour length: " << initial_tour_length << std::endl;

    const auto morton_keys {point_quadtree::morton_keys::compute_point_morton_keys(point_set.x(), point_set.y(), domain)};
    point_quadtree::Node root(nullptr, domain, 0, 0, 0);
    const auto leaf_nodes{solver::get_leaf_nodes(root, morton_keys, domain)};

    solver::hill_climb(tour_modifier.current_tour()
        , morton_keys, root, leaf_nodes, point_set.x(), point_set.y(), dc);

    return 0;
}
