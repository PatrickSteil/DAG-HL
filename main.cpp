/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#include <stdlib.h>

#include <cstdlib>
#include <iostream>
#include <numeric>
#include <thread>
#include <utility>

#include "datastructures/graph.h"
#include "datastructures/hub_labels.h"
#include "datastructures/topological_sort.h"
#include "datastructures/weighted_pll.h"
#include "external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph", "Input graph file.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<std::string>(
      "r", "ordering_file", "",
      "File containing the ordering and the centrality measure per line");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string outputFileName = parser.get<std::string>("o");
  const std::string orderingFile = parser.get<std::string>("r");
  const auto showstats = parser.get<bool>("s");

  Graph g;
  g.readDimacs(inputFileName);

  if (showstats) {
    std::cout << "Forward ";
    g.showStats();
  }

  std::vector<std::size_t> rank(g.numVertices(), 0);
  TopologicalSort sorter(g);

  for (std::size_t i = 0; i < g.numVertices(); ++i) {
    rank[sorter.getOrdering()[i]] = i;
  }

  Graph rev = g.reverseGraph();

  const int K = 256;

  WeightedPLL<K> hl(g, rev, rank);

  std::vector<Vertex> ordering = getOrdering(orderingFile, hl.graph);
  hl.run(ordering);

  return 0;
}
