#include <stdlib.h>

#include <iostream>
#include <numeric>
#include <set>
#include <thread>

#include "datastructures/graph.h"
#include "datastructures/rxl.h"
#include "external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph",
                                   "Input graph in DIMACs format.");
  parser.set_optional<std::string>("o", "output_file", "",
                                   "Output file to save hub labels into.");
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
  parser.set_optional<bool>("s", "show_stats", false,
                            "Show statistics about the computed hub labels.");
  parser.set_optional<bool>("c", "compress_labels", false,
                            "Reorders hubs, and computes Delta compression");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);

  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string outputFileName = parser.get<std::string>("o");
  const int numThreads = parser.get<int>("t");
  const auto showStats = parser.get<bool>("s");
  const auto compress = parser.get<bool>("c");

  if (numThreads <= 0) {
    std::cout << "Number of threads should be greater than 0!" << std::endl;
    return -1;
  }

  Graph g;
  g.readDimacs(inputFileName);

  if (showStats) g.showStats();

  Graph rev = g.reverseGraph();

  RXL<> hl(g, rev);
  hl.run(numThreads);

  if (compress) {
    auto permutation = computePermutation(hl.labels);

#pragma omp parallel for
    for (Vertex v = 0; v < g.numVertices(); ++v) {
      hl.labels[0][v].applyPermutation(permutation);
      hl.labels[1][v].applyPermutation(permutation);
    }
  }
  hl.sortAllLabels();

  /* std::size_t maxDiff = 0; */

  /* for (Vertex v = 0; v < g.numVertices(); ++v) { */
  /*   for (std::size_t i = 1; i < hl.labels[0][v].size(); ++i) { */
  /*     maxDiff = std::max(maxDiff, */
  /*                        static_cast<std::size_t>(hl.labels[0][v][i] - */
  /*                                                 hl.labels[0][v][i - 1] -
   * 1)); */
  /*   } */
  /*   for (std::size_t i = 1; i < hl.labels[1][v].size(); ++i) { */
  /*     maxDiff = std::max(maxDiff, */
  /*                        static_cast<std::size_t>(hl.labels[1][v][i] - */
  /*                                                 hl.labels[1][v][i - 1] -
   * 1)); */
  /*   } */
  /* } */

  /* std::cout << "Max diff: " << maxDiff << std::endl; */

  if (showStats) hl.showStats();

  if (outputFileName != "") hl.saveToFile(outputFileName);
  return 0;
}
