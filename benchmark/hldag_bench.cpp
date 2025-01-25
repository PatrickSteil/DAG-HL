/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#include <omp.h>
#include <stdlib.h>

#include <cstdlib>
#include <iostream>
#include <numeric>
#include <thread>
#include <utility>

#include "../datastructures/drawer.h"
#include "../datastructures/graph.h"
#include "../datastructures/hldag.h"
#include "../datastructures/hub_labels.h"
#include "../external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph", "Input graph file.");
  parser.set_optional<std::string>(
      "f", "graph_format", "DIMACS",
      "Graph format: METIS, SNAP, DIMACS, or EDGELIST.");
  parser.set_optional<std::string>(
      "r", "ordering_file", "",
      "File containing the ordering and the centrality measure per line");
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string graphFormat = parser.get<std::string>("f");
  const std::string orderingFile = parser.get<std::string>("r");
  const int numThreads = parser.get<int>("t");

  if (numThreads <= 0) {
    std::cout << "Number of threads should be greater than 0!" << std::endl;
    return -1;
  }

  omp_set_num_threads(numThreads);

  Graph g;
  if (graphFormat == "METIS") {
    g.readMetis(inputFileName);
  } else if (graphFormat == "SNAP") {
    g.readSnap(inputFileName);
  } else if (graphFormat == "DIMACS") {
    g.readDimacs(inputFileName);
  } else if (graphFormat == "EDGELIST") {
    g.readFromEdgeList(inputFileName);
  } else {
    std::cerr << "Unknown graph format: " << graphFormat << std::endl;
    return -1;
  }

  std::vector<std::size_t> rank(g.numVertices(), 0);
  TopologicalSort sorter(g);

  for (std::size_t i = 0; i < g.numVertices(); ++i) {
    rank[sorter.getOrdering()[i]] = i;
  }

  Graph rev = g.reverseGraph();

  auto run = [&](auto &hl) -> void { hl.run(orderingFile); };

  {
    std::cout << "K 64" << std::endl;
    HLDAG<64, LabelThreadSafe> hl(g, rev, rank, numThreads);
    run(hl);
  }
  {
    std::cout << "K 128" << std::endl;
    HLDAG<128, LabelThreadSafe> hl(g, rev, rank, numThreads);
    run(hl);
  }
  {
    std::cout << "K 256" << std::endl;
    HLDAG<256, LabelThreadSafe> hl(g, rev, rank, numThreads);
    run(hl);
  }
  {
    std::cout << "K 512" << std::endl;
    HLDAG<512, LabelThreadSafe> hl(g, rev, rank, numThreads);
    run(hl);
  }
  return 0;
}
