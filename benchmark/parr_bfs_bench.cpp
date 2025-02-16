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

#include "../datastructures/bfs.h"
#include "../datastructures/graph.h"
#include "../datastructures/utils.h"
#include "../external/cmdparser.hpp"

void configure_parser(cli::Parser &parser) {
  parser.set_required<std::string>("i", "input_graph", "Input graph file.");
  parser.set_optional<std::string>(
      "f", "graph_format", "DIMACS",
      "Graph format: METIS, SNAP, DIMACS, or EDGELIST.");
};

int main(int argc, char *argv[]) {
  cli::Parser parser(argc, argv);
  configure_parser(parser);
  parser.run_and_exit_if_error();

  const std::string inputFileName = parser.get<std::string>("i");
  const std::string graphFormat = parser.get<std::string>("f");

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

  std::vector<std::pair<Vertex, Vertex>> queries =
      generateRandomQueries<Vertex>(1000, 0, static_cast<int>(g.numVertices()));

  auto run = [&](auto &bfs) -> void {
    StatusLog log("Running Queries");

    for (const auto &paar : queries) {
      bfs.run(
          paar.first, [](const Vertex) { return false; },
          [](const Vertex, const Vertex) { return false; });
    }
  };

  {
    std::cout << "Sequential BFS" << std::endl;
    bfs::BFS bfs(g);
    run(bfs);
  }
  {
    std::cout << "Frontier BFS [t=2]" << std::endl;
    bfs::BFSParallelFrontier bfs(g, 2);
    run(bfs);
  }
  {
    std::cout << "Frontier BFS [t=4]" << std::endl;
    bfs::BFSParallelFrontier bfs(g, 4);
    run(bfs);
  }
  {
    std::cout << "Frontier BFS [t=6]" << std::endl;
    bfs::BFSParallelFrontier bfs(g, 6);
    run(bfs);
  }
  return 0;
}

/*
>>> ./parr_bfs_bench -i ../data/icice.dimacs
Reading graph from dimacs ... done [183ms]
Sequential BFS
Running Queries ... done [1498ms]
Frontier BFS [t=2]
Running Queries ... done [1457ms]
Frontier BFS [t=4]
Running Queries ... done [935ms]
Frontier BFS [t=6]
Running Queries ... done [835ms]

>>> ./parr_bfs_bench -i ../data/kvv.dimacs
Reading graph from dimacs ... done [2631ms]
Sequential BFS
Running Queries ... done [59352ms]
Frontier BFS [t=2]
Running Queries ... done [56157ms]
Frontier BFS [t=4]
Running Queries ... done [30319ms]
Frontier BFS [t=6]
Running Queries ... done [22769ms]

>>> ./parr_bfs_bench -i ../data/berlin.dimacs
Reading graph from dimacs ... done [5759ms]
Sequential BFS
Running Queries ... done [192551ms]
Frontier BFS [t=2]
Running Queries ... done [159898ms]
Frontier BFS [t=4]
Running Queries ... done [87331ms]
Frontier BFS [t=6]
Running Queries ... done [65007ms]
*/
