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
  parser.set_required<std::string>("i", "input_graph",
                                   "Input graph in DIMACs format.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<std::string>(
      "r", "ordering_file", "",
      "File containing the ordering and the centrality measure _per line_");
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
  parser.set_optional<bool>("c", "compress_labels", false,
                            "Reorders hubs by frequency");
  parser.set_optional<bool>("b", "benchmark_queries", false,
                            "Runs a small (10.000) query benchmark");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);

  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string outputFileName = parser.get<std::string>("o");
  const std::string orderingFile = parser.get<std::string>("r");
  const int numThreads = parser.get<int>("t");
  const auto showStats = parser.get<bool>("s");
  const auto compress = parser.get<bool>("c");
  const auto run_benchmark = parser.get<bool>("b");

<<<<<<< HEAD
  const int K = 64;
=======
  const int K = 256;
>>>>>>> a63db62 (added bitsets)

  if (numThreads <= 0) {
    std::cout << "Number of threads should be greater than 0!" << std::endl;
    return -1;
  }

  Graph g;
  g.readDimacs(inputFileName);

  if (showStats) {
    std::cout << "Forward ";
    g.showStats();
  }

  Graph rev = g.reverseGraph();

  HLDAG<K, LabelThreadSafe> hl(g, rev);

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
    benchmark(hl.labels, 10000);
  }
  return 0;
}
