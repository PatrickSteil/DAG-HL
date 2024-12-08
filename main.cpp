#include <stdlib.h>

#include <iostream>
#include <numeric>
#include <set>

#include "datastructures/graph.h"
#include "datastructures/pruned_landmark_labeling.h"

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <graph_filename>\n"
              << "  - <graph_filename>: Path to the graph edge list file.\n";
    return 1;
  }

  const std::string inputFileName = argv[1];

  Graph g;
  g.readDimacs(inputFileName);

  Graph rev = g.reverseGraph();

  std::vector<Vertex> ordering;

  auto isOrdering = [](const std::vector<Vertex> &ordering,
                       const std::size_t numVertices) -> bool {
    std::set<Vertex> orderedSet(ordering.begin(), ordering.end());

    if (orderedSet.size() != numVertices) {
      std::cout << "The ordering does not contain all vertices!" << std::endl;
      std::cout << "Ordering has " << orderedSet.size() << ", but there are "
                << numVertices << " many vertices!" << std::endl;
      return false;
    }
    if (!orderedSet.contains(0)) {
      std::cout << "The ordering does not contain 0!" << std::endl;
      return false;
    }
    if (!orderedSet.contains(numVertices - 1)) {
      std::cout << "The ordering does not contain the last vertex!"
                << std::endl;
      return false;
    }

    return true;
  };

  ordering.assign(g.numVertices(), 0);
  std::iota(ordering.begin(), ordering.end(), 0);
  std::sort(ordering.begin(), ordering.end(),
            [&](const auto left, const auto right) {
              return g.degree(left) + rev.degree(left) >
                     g.degree(right) + rev.degree(right);
            });

  if (!isOrdering(ordering, g.numVertices())) {
    std::cout
        << "Given ordering is not valid, check that there are no duplicates!"
        << std::endl;
    return -1;
  }

  PLL pll(g, rev);
  pll.run(ordering);
  pll.showStats();
  return 0;
}
