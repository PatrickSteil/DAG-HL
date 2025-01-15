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
  parser.set_optional<int>("t", "num_threads",
                           std::thread::hardware_concurrency(),
                           "Number of threads to use.");
};

template <int K>
void run(HLDAG<K, LabelThreadSafe> &hl) {
  hl.template run<true>("");
  hl.showStats();

  hl.run("");
  hl.showStats();
}

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);

  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const int numThreads = parser.get<int>("t");

  if (numThreads <= 0) {
    std::cout << "Number of threads should be greater than 0!" << std::endl;
    return -1;
  }

  omp_set_num_threads(numThreads);

  Graph g;
  g.readDimacs(inputFileName);

  std::cout << "Forward ";
  g.showStats();

  Graph rev = g.reverseGraph();

  std::cout << "K: 64" << std::endl;
  HLDAG<64, LabelThreadSafe> hl(g, rev, numThreads);
  run(hl);

  std::cout << "K: 128" << std::endl;
  HLDAG<128, LabelThreadSafe> hl128(g, rev, numThreads);
  run(hl128);

  std::cout << "K: 256" << std::endl;
  HLDAG<256, LabelThreadSafe> hl256(g, rev, numThreads);
  run(hl256);
  return 0;
}
