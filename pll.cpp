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

#include "datastructures/graph.h"
#include "datastructures/hub_labels.h"
#include "datastructures/pruned_landmark_labeling.h"
#include "external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph", "Input graph file.");
  parser.set_optional<std::string>(
      "f", "graph_format", "DIMACS",
      "Graph format: METIS, SNAP, DIMACS, or EDGELIST.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<std::string>(
      "r", "ordering_file", "",
      "File containing the ordering and the centrality measure per line");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
  parser.set_optional<bool>("c", "compress_labels", false,
                            "Reorders hubs by frequency");
  parser.set_optional<bool>("b", "benchmark_queries", false,
                            "Runs a small (10,000) query benchmark");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string graphFormat = parser.get<std::string>("f");
  const std::string outputFileName = parser.get<std::string>("o");
  const std::string orderingFile = parser.get<std::string>("r");
  const auto showstats = parser.get<bool>("s");
  const auto compress = parser.get<bool>("c");
  const auto run_benchmark = parser.get<bool>("b");

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

  if (showstats) {
    std::cout << "Forward ";
    g.showStats();
  }

  Graph rev = g.reverseGraph();

  std::array<std::vector<Label>, 2> labels = {
      std::vector<Label>(g.numVertices()), std::vector<Label>(g.numVertices())};
  std::array<std::vector<std::bitset<0>>, 2> bitsets;
  std::vector<std::atomic<bool>> alreadyProcessed(g.numVertices());
  for (std::size_t v = 0; v < g.numVertices(); ++v) {
    alreadyProcessed[v] = false;
  }

  std::array<const Graph *, 2> graph = {&g, &rev};

  PLL<0, Label> pll(labels, bitsets, alreadyProcessed, graph);
  std::vector<Vertex> ordering = getOrdering(orderingFile, graph);

  pll.run(ordering);

  if (compress) {
    auto permutation = computePermutation(labels);

#pragma omp parallel for
    for (Vertex v = 0; v < g.numVertices(); ++v) {
      labels[0][v].applyPermutation(permutation);
      labels[1][v].applyPermutation(permutation);
    }
  }

  sortLabels(labels);

  if (showstats) showStats(labels);

  if (outputFileName != "") saveToFile(labels, outputFileName);

  if (run_benchmark) {
    benchmark_hublabels(labels, 10000);
  }
  return 0;
}
