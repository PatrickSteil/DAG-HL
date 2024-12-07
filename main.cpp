#include <iostream>
#include <numeric>
#include <set>
#include <stdlib.h>

#include "datastructures/graph.h"
#include "datastructures/pruned_landmark_labeling.h"
#include "datastructures/topological_seperator.h"
#include "datastructures/topological_sort.h"

int main(int argc, char *argv[]) {
  if (argc < 4) {
    std::cerr
        << "Usage: " << argv[0]
        << " <graph_filename> <nested_dissection_ordering> <output_filename>\n"
        << "  - <graph_filename>: Path to the graph edge list file.\n"
        << "  - <nested_dissection_ordering>: Path to the nested dissection "
           "ordering file. (The first vertex is the least important)\n"
        << "  - <output_filename>: Path to the file, where the labels will be "
           "written..\n";
    return 1;
  }

  const std::string inputFileName = argv[1];
  const std::string nestedDisFileName = argv[2];
  const std::string outputFileName = argv[3];

  Graph g;
  g.readDimacs(inputFileName);

  Graph rev = g.reverseGraph();

  std::vector<Vertex> ordering;
  ordering.reserve(g.numVertices());

  auto readOrdering = [&](const std::string &fileName) -> bool {
    std::ifstream file(fileName);
    if (!file.is_open()) {
      std::cout << "Cannot open file: " << fileName << std::endl;
      return false;
    }

    std::string line;

    while (std::getline(file, line)) {
      std::istringstream iss(line);
      Vertex v;
      if (iss >> v) { // Ensure the line can be parsed into a Vertex
        ordering.push_back(v);
      } else {
        std::cout << "Invalid line: " << line << std::endl;
        return false;
      }
    }

    file.close();

    std::reverse(ordering.begin(), ordering.end());

    return true;
  };

  auto isOrdering = [](const std::vector<Vertex> &ordering,
                       const std::size_t numVertices) -> bool {
    std::set<Vertex> orderedSet(ordering.begin(), ordering.end());

    if (orderedSet.size() != numVertices) {
      std::cout << "The ordering does not contain all vertices!" << std::endl;
      std::cout << "Ordering has " << orderedSet.size() << ", but there are "
                << numVertices << " many vertices!" << std::endl;
      return false;
    }
    if (!orderedSet.contains(0)) {
      std::cout << "The ordering does not contain 0!" << std::endl;
      return false;
    }
    if (!orderedSet.contains(numVertices - 1)) {
      std::cout << "The ordering does not contain the last vertex!"
                << std::endl;
      return false;
    }

    return true;
  };

  /* if (!readOrdering(nestedDisFileName)) { */
  /*   return -1; */
  /* }; */

  /* TopologicalSort topoSort(g); */
  /* TopologicalSeperator topoSep(g, topoSort.ordering); */
  /* ordering = topoSep.run(0.03); */

  ordering.assign(g.numVertices(), 0);
  std::iota(ordering.begin(), ordering.end(), 0);
  std::sort(ordering.begin(), ordering.end(),
            [&](const auto left, const auto right) {
              return g.degree(left) + rev.degree(left) >
                     g.degree(right) + rev.degree(right);
            });

  if (!isOrdering(ordering, g.numVertices())) {
    std::cout
        << "Given ordering is not valid, check that there are no duplicates!"
        << std::endl;
    return -1;
  }

  PLL pll(g, rev);
  pll.run(ordering);
  pll.showStats();
  pll.saveToFile(outputFileName);
  return 0;
}
