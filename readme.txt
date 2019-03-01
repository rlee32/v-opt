This is a hill-climbing improvement heuristic.
For every P point in a tour, if there exists another point O to which an edge shorter than both currently associated with P can be created, then such an edge P-O is introduced into the tour.
Two additional steps are required to ensure that the new tour is a valid cycle:
1. Destroy all existing edges with P and create an edge between the 2 immediate neighbors of P.
2. Create a new edge between P and one neighbor of O, and destroy the edge between O and said neighbor.


