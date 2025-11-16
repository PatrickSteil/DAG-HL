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

#include "datastructures/dag_wpll.h"
#include "datastructures/graph.h"
#include "datastructures/hub_labels.h"
#include "datastructures/topological_sort.h"
#include "datastructures/wpll.h"
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
  parser.set_optional<bool>("b", "benchmark_queries", false,
                            "Runs a small (10,000) query benchmark");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string outputFileName = parser.get<std::string>("o");
  const std::string orderingFile = parser.get<std::string>("r");
  const bool showstats = parser.get<bool>("s");
  const bool run_benchmark = parser.get<bool>("b");

  Graph g;
  g.readDimacs(inputFileName);

  if (showstats) {
    std::cout << "Forward ";
    g.showStats();
  }

  /* std::vector<std::size_t> rank(g.numVertices(), 0); */
  /* TopologicalSort sorter(g); */

  /* for (std::size_t i = 0; i < g.numVertices(); ++i) { */
  /*   rank[sorter.getOrdering()[i]] = i; */
  /* } */

  Graph rev = g.reverseGraph();

  const int K = 256;

  std::array<std::vector<Label>, 2> labels{
      std::vector<Label>(g.numVertices(), Label()),
      std::vector<Label>(g.numVertices(), Label())};
  std::array<const Graph *, 2> graph{&g, &rev};
  /* DAG_WPLL<K> hl(g, rev, rank); */
  WPLL<K> hl(labels, graph);

  std::vector<Vertex> ordering = getOrdering(orderingFile, graph);
  hl.run(ordering);

  sortLabels(labels);

  if (showstats) showStats(labels);

  if (outputFileName != "") saveToFile(labels, outputFileName);

  if (run_benchmark) {
    benchmark_hublabels(hl.labels, 10000);
  }

  return 0;
}
