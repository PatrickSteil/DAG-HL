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
#include "datastructures/hldag.h"
#include "datastructures/hub_labels.h"
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
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
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
  const int numThreads = parser.get<int>("t");
  const auto showStats = parser.get<bool>("s");
  const auto compress = parser.get<bool>("c");
  const auto run_benchmark = parser.get<bool>("b");

  // Min Trees
  const int T = 8;
  // Bitset Width
  const int K = 256;

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

  if (showStats) {
    std::cout << "Forward ";
    g.showStats();
  }

  std::vector<std::size_t> rank(g.numVertices(), 0);
  TopologicalSort sorter(g);

  for (std::size_t i = 0; i < g.numVertices(); ++i) {
    rank[sorter.getOrdering()[i]] = i;
  }

  Graph rev = g.reverseGraph();

  HLDAG<T, K, LabelThreadSafe> hl(g, rev, rank, numThreads);

  /* hl.sample(); */
  /* hl.updateDescendantCounter(); */

  /* Vertex top = hl.pickBestVertex(); */

  /* auto printImportance = [&](const Vertex v) -> void { */
  /*   auto importance = getImportanceByAvg2Max(hl.valuesPerElement[v]); */
  /*   std::cout << "Vertex: " << v << " with " << importance.first << ", " */
  /*             << importance.second << std::endl; */
  /* }; */

  /* std::cout << "Top: " << top << std::endl; */

  /* for (Vertex v = 0; v < g.numVertices(); ++v) { */
  /*   printImportance(v); */
  /* } */
  /* return 0; */

  hl.run(orderingFile);

  if (compress) {
    auto permutation = computePermutation(hl.labels);

#pragma omp parallel for
    for (Vertex v = 0; v < g.numVertices(); ++v) {
      hl.labels[0][v].applyPermutation(permutation);
      hl.labels[1][v].applyPermutation(permutation);
    }
  }

  sortLabels(hl.labels);

  if (showStats) hl.showStats();

  if (outputFileName != "") saveToFile(hl.labels, outputFileName);

  if (run_benchmark) {
    benchmark_hublabels(hl.labels, 10000);
  }
  return 0;
}
