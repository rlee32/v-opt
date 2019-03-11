This is a novel hill-climbing improvement heuristic.
For every P point in a tour, if there exists another point O to which an edge shorter than both currently associated with P can be created,
then such an edge P-O is introduced into the tour.
Two additional steps are required to ensure that the new tour is a valid cycle:
1. Destroy all existing edges with P and create an edge between the 2 immediate neighbors of P.
2. Create a new edge between P and one neighbor of O, and destroy the edge between O and said neighbor.

In its current state, v-opt is not meant to be competitive with k-opt. However, it could serve as a useful heuristic for localized perturbations,
because any arbitrary new (shorter) edges can be introduced into the tour.

TODO:
1. Local perturbations are currently disabled; clean up solver.h and make t least one variant the default.
