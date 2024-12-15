#include <stdlib.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <set>
#include <thread>
#include <utility>

#include "datastructures/graph.h"
#include "datastructures/rxl.h"
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
                            "Reorders hubs, and computes Delta compression");
  parser.set_optional<bool>("b", "benchmark_queries", false,
                            "Runs a small (1000) query benchmark");
};

std::vector<std::pair<Vertex, Vertex>> generateRandomQueries(int numQueries,
                                                             int minVertex,
                                                             int maxVertex) {
  std::vector<std::pair<Vertex, Vertex>> queries;
  std::srand(42);

  for (int i = 0; i < numQueries; ++i) {
    Vertex source = minVertex + std::rand() % (maxVertex - minVertex + 1);
    Vertex target = minVertex + std::rand() % (maxVertex - minVertex + 1);

    while (source == target) {
      target = minVertex + std::rand() % (maxVertex - minVertex + 1);
    }

    queries.emplace_back(source, target);
  }

  return queries;
}

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
  const auto benchmark = parser.get<bool>("b");

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

  RXL<> hl(g, rev);

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

  if (benchmark) {
    using std::chrono::duration;
    using std::chrono::duration_cast;
    using std::chrono::high_resolution_clock;
    using std::chrono::milliseconds;

    std::size_t numQueries = 10000;

    auto queries = generateRandomQueries(numQueries, 0, g.numVertices());
    long double totalTime(0);
    for (std::pair<Vertex, Vertex> paar : queries) {
      auto t1 = high_resolution_clock::now();
      query(hl.labels, paar.first, paar.second);
      auto t2 = high_resolution_clock::now();
      duration<double, std::nano> nano_double = t2 - t1;
      totalTime += nano_double.count();
    }

    std::cout << "The " << numQueries << " random queries took in total "
              << totalTime << " [ms] and on average "
              << (double)(totalTime / numQueries) << " [ns]!\n";
  }
  return 0;
}
