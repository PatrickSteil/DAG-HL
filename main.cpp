#include <stdlib.h>

#include <iostream>
#include <numeric>
#include <set>

#include "datastructures/graph.h"
#include "datastructures/pruned_landmark_labeling.h"
#include "external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph",
                                   "Input graph in DIMACs format.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<int>("t", "num_threads", 1, "Number of threads to use.");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);

  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string outputFileName = parser.get<std::string>("o");
  /* const int numThreads = parse.get<int>("t"); */
  const auto showStats = parser.get<bool>("s");

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
  pll.sortAllLabels();

  if (showStats) pll.showStats();

  if (outputFileName != "") pll.saveToFile(outputFileName);
  return 0;
}
