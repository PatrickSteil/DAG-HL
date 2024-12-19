#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <immintrin.h>
#include <omp.h>

#include <algorithm>
#include <array>
#include <bitset>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

#include "bfs.h"
#include "graph.h"
#include "hub_labels.h"
#include "pruned_landmark_labeling.h"
#include "status_log.h"
#include "topological_sort.h"
#include "utils.h"

template <std::uint16_t SIZE = 64> struct HLDAG {
public:
  std::array<std::vector<Label>, 2> labels;

  std::vector<uint8_t> alreadyProcessed;
  std::array<const Graph *, 2> graph;

  std::array<std::vector<uint8_t>, 2> lookup;
  std::array<bfs::BFS, 2> bfs;

  std::vector<std::size_t> topoRank;
  std::vector<Edge> topoEdges;

  std::vector<PLL> workers;

  HLDAG(const Graph &fwdGraph, const Graph &bwdGraph,
        const int numberOfThreads = 1)
      : labels{std::vector<Label>(), std::vector<Label>()}, alreadyProcessed(),
        graph{&fwdGraph, &bwdGraph},
        lookup{std::vector<uint8_t>(), std::vector<uint8_t>()},
        bfs{bfs::BFS(fwdGraph), bfs::BFS(bwdGraph)}, topoEdges(), workers() {
    if (numberOfThreads < 1) {
      throw std::runtime_error(
          "Number of threads should not be smaller than 1!");
    }
    workers.reserve(numberOfThreads);
    for (int t = 0; t < numberOfThreads; ++t) {
      workers.emplace_back(labels, lookup, alreadyProcessed, graph);
    }
    init(fwdGraph.numVertices());
  };

  void run(const std::string &orderingFileName) {
    StatusLog log("Computing HLs");
    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    const std::size_t numVertices = graph[FWD]->numVertices();

    std::vector<Vertex> ordering = getOrdering(orderingFileName);

    assert(ordering.size() == numVertices);
    assert(isOrdering(ordering, numVertices));

    std::size_t i = 0;

    for (; i < numVertices; ++i) {
      workers[0].runPrunedBFS(ordering[i]);
    }
  }

  std::size_t computeTotalBytes() const {
    std::size_t totalBytes = 0;

    for (const auto &labelSet : labels) {
      for (const auto &label : labelSet) {
        totalBytes += sizeof(Label);
        totalBytes += label.nodes.capacity() * sizeof(Vertex);
      }
    }

    return totalBytes;
  }

  void showStats() const {
    auto computeStats = [](const std::vector<Label> &currentLabels) {
      std::size_t minSize = std::numeric_limits<std::size_t>::max();
      std::size_t maxSize = 0;
      std::size_t totalSize = 0;

      for (const auto &label : currentLabels) {
        std::size_t size = label.size();
        minSize = std::min(minSize, size);
        maxSize = std::max(maxSize, size);
        totalSize += size;
      }

      double avgSize = static_cast<double>(totalSize) / currentLabels.size();
      return std::make_tuple(minSize, maxSize, avgSize, totalSize);
    };

    auto [inMin, inMax, inAvg, inTotal] = computeStats(labels[BWD]);
    auto [outMin, outMax, outAvg, outTotal] = computeStats(labels[FWD]);

    std::cout << "Forward Labels Statistics:" << std::endl;
    std::cout << "  Min Size:     " << outMin << std::endl;
    std::cout << "  Max Size:     " << outMax << std::endl;
    std::cout << "  Avg Size:     " << outAvg << std::endl;
    std::cout << "  # count:      " << outTotal << std::endl;

    std::cout << "Backward Labels Statistics:" << std::endl;
    std::cout << "  Min Size:     " << inMin << std::endl;
    std::cout << "  Max Size:     " << inMax << std::endl;
    std::cout << "  Avg Size:     " << inAvg << std::endl;
    std::cout << "  # count:      " << inTotal << std::endl;

    std::cout << "Both # count:   " << (outTotal + inTotal) << std::endl;

    std::cout << "Total memory consumption [megabytes]:" << std::endl;
    std::cout << "  "
              << static_cast<double>(computeTotalBytes() / (1024.0 * 1024.0))
              << std::endl;
  }

  void print() const {
    for (std::size_t v = 0; v < graph[FWD]->numVertices(); ++v) {
      std::cout << " -> " << v << "\n\t";
      for (auto h : labels[FWD][v].nodes)
        std::cout << h << " ";
      std::cout << "\n <- " << v << "\n\t";
      for (auto h : labels[BWD][v].nodes)
        std::cout << h << " ";
      std::cout << std::endl;
    }
  }

  std::vector<Vertex> getOrdering(const std::string &fileName) {
    std::vector<Vertex> ordering;
    ordering.reserve(graph[FWD]->numVertices());

    if (fileName == "") {
      parallel_assign(ordering, graph[FWD]->numVertices(), Vertex(0));

      std::vector<std::size_t> randomNumber;
      parallel_assign_iota(randomNumber, graph[FWD]->numVertices(),
                           static_cast<std::size_t>(0));

      std::random_device rd;
      std::mt19937 g(rd());

      std::shuffle(randomNumber.begin(), randomNumber.end(), g);

      /* auto degreeComp = [&](const auto left, const auto right) { */
      /*   return graph[FWD]->degree(left) + graph[BWD]->degree(left) > */
      /*          graph[FWD]->degree(right) + graph[BWD]->degree(right); */
      /* }; */

      auto degreeCompRandom = [&](const auto left, const auto right) {
        return std::forward_as_tuple(graph[FWD]->degree(left) +
                                         graph[BWD]->degree(left),
                                     randomNumber[left]) >
               std::forward_as_tuple(graph[FWD]->degree(right) +
                                         graph[BWD]->degree(right),
                                     randomNumber[right]);
      };

      std::iota(ordering.begin(), ordering.end(), 0);
      std::sort(ordering.begin(), ordering.end(), degreeCompRandom);
    } else {
      std::ifstream file(fileName);
      if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + fileName);
      }

      std::string line;
      while (std::getline(file, line)) {
        std::istringstream iss(line);
        Vertex vertex;
        double centrality;

        if (iss >> vertex >> centrality) {
          ordering.push_back(vertex - 1);
        } else {
          throw std::runtime_error("Failed to parse line: " + line);
        }
      }
    }
    return ordering;
  }

  std::vector<Vertex> getOrderingDegTopo() {
    std::vector<Vertex> ordering(graph[FWD]->numVertices(), 0);

    TopologicalSort sorter(*graph[FWD]);
    std::vector<std::size_t> topoRank(graph[FWD]->numVertices(), 0);

    for (std::size_t i = 0; i < sorter.ordering.size(); ++i) {
      topoRank[sorter.ordering[i]] = i;
    }

    std::iota(ordering.begin(), ordering.end(), 0);

    auto rank = [&](auto v) -> double {
      double centrality = 1 - abs((topoRank[v] / topoRank.size()) - 0.5);
      return 0.5 * (graph[FWD]->degree(v) + graph[BWD]->degree(v)) +
             1000 * centrality;
    };

    std::sort(ordering.begin(), ordering.end(),
              [&](const auto left, const auto right) {
                return rank(left) > rank(right);
              });

    return ordering;
  }

  bool isOrdering(const std::vector<Vertex> &ordering,
                  const std::size_t numVertices) {
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

  void init(std::size_t numVertices) {
    parallel_assign(labels[BWD], numVertices, Label());
    parallel_assign(labels[FWD], numVertices, Label());
    parallel_assign(alreadyProcessed, numVertices, uint8_t(0));

    parallel_assign(lookup[BWD], numVertices, uint8_t(0));
    parallel_assign(lookup[FWD], numVertices, uint8_t(0));

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);

    /* parallel_assign(reached[BWD], numVertices, SIZE(0)); */
    /* parallel_assign(reached[FWD], numVertices, SIZE(0)); */

    parallel_assign(topoRank, numVertices, std::size_t(0));

    TopologicalSort topoSort(*graph[FWD]);
    for (std::size_t i = 0; i < topoSort.ordering.size(); ++i) {
      topoRank[topoSort.ordering[i]] = i;
    }

    topoEdges.reserve(graph[FWD]->numEdges());

    for (Vertex from = 0; from < numVertices; ++from) {
      for (std::size_t i = graph[FWD]->beginEdge(from);
           i < graph[FWD]->endEdge(from); ++i) {
        topoEdges.emplace_back(from, graph[FWD]->toVertex[i]);
      }
    }

    std::sort(topoEdges.begin(), topoEdges.end());
  }

  void modifyLookups(const Vertex v, bool value) {
    auto forDir = [&](const DIRECTION dir) -> void {
      for (std::size_t i = 0; i < labels[dir][v].size(); ++i) {
        if (i + 4 < labels[dir][v].size()) {
          PREFETCH(&lookup[dir][labels[dir][v][i + 4]]);
        }
        const Vertex u = labels[dir][v][i];
        assert(u < lookup[dir].size());
        lookup[dir][u] = value;
      }
    };

    forDir(FWD);
    forDir(BWD);
  }

  /*   void runReachabilitySweep(const std::size_t n, */
  /*                             const std::vector<Vertex> &ordering) { */
  /*     parallel_fill(reached[0], 0); */
  /*     parallel_fill(reached[1], 0); */

  /* #pragma GCC unroll(4) */
  /*     for (std::size_t j = 0; j < width; ++j) { */
  /*       reached[0][ordering[n + j]] |= (1 << j); */
  /*       reached[1][ordering[n + j]] |= (1 << j); */
  /*     } */

  /*     for (std::size_t i = 0; i < topoEdges.size(); ++i) { */
  /*       const auto &edge = topoEdges[i]; */

  /*       reached[0][edge.to] |= reached[0][edge.from]; */
  /*     } */
  /*     for (std::size_t i = topoEdges.size(); i > 0; --i) { */
  /*       const auto &edge = topoEdges[i - 1]; */

  /*       reached[0][edge.from] |= reached[0][edge.to]; */
  /*     } */
  /*   } */
};
