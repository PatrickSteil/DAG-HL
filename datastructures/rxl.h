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
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <thread>
#include <vector>

#include "bfs.h"
#include "graph.h"
#include "hub_labels.h"
#include "status_log.h"
#include "topological_sort.h"

template <int K = 64>
struct RXL {
 public:
  union alignas(64) PathCounter {
    PathCounter(std::uint32_t value = 0) {
      std::fill(std::begin(values), std::end(values), value);
    }
    __m256i mValues;
    std::uint32_t values[8];

    PathCounter operator+(const PathCounter &other) const {
      PathCounter result;
      result.mValues = _mm256_add_epi32(this->mValues, other.mValues);
      return result;
    }

    PathCounter &operator+=(const PathCounter &other) {
      this->mValues = _mm256_add_epi32(this->mValues, other.mValues);
      return *this;
    }
  };

  static_assert(sizeof(PathCounter) == 64, "Size of PathCounter is not valid!");

  std::array<std::vector<Label>, 2> labels;
  std::array<std::vector<uint8_t>, 2> lookup;
  std::vector<uint8_t> alreadyProcessed;
  std::array<const Graph *, 2> graph;
  std::array<bfs::BFS, 2> bfs;

  std::array<std::vector<PathCounter>, 2> counter;
  std::vector<std::size_t> topoRank;
  std::vector<Edge> topoEdges;

  RXL(const Graph &fwdGraph, const Graph &bwdGraph)
      : labels{std::vector<Label>(), std::vector<Label>()},
        lookup{std::vector<uint8_t>(), std::vector<uint8_t>()},
        alreadyProcessed(),
        graph{&fwdGraph, &bwdGraph},
        bfs{bfs::BFS(fwdGraph), bfs::BFS(bwdGraph)},
        counter{std::vector<PathCounter>(), std::vector<PathCounter>()},
        topoEdges() {
    init(fwdGraph.numVertices());
  };

  void run(const std::string &orderingFileName, const int numThreads = 1) {
    StatusLog log("Computing HLs");
    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    omp_set_num_threads(numThreads);

    const std::size_t numVertices = graph[FWD]->numVertices();

    std::vector<Vertex> ordering = getOrdering(orderingFileName);

    assert(ordering.size() == numVertices);
    assert(isOrdering(ordering, numVertices));

    for (std::size_t i = 0; i < numVertices; ++i) {
      runPrunedBFS(ordering[i]);
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
      return std::make_tuple(minSize, maxSize, avgSize);
    };

    auto [inMin, inMax, inAvg] = computeStats(labels[BWD]);
    auto [outMin, outMax, outAvg] = computeStats(labels[FWD]);

    std::cout << "Forward Labels Statistics:" << std::endl;
    std::cout << "  Min Size: " << outMin << std::endl;
    std::cout << "  Max Size: " << outMax << std::endl;
    std::cout << "  Avg Size: " << outAvg << std::endl;

    std::cout << "Backward Labels Statistics:" << std::endl;
    std::cout << "  Min Size: " << inMin << std::endl;
    std::cout << "  Max Size: " << inMax << std::endl;
    std::cout << "  Avg Size: " << inAvg << std::endl;

    std::cout << "Total memory consumption [megabytes]:" << std::endl;
    std::cout << "  "
              << static_cast<double>(computeTotalBytes() / (1024.0 * 1024.0))
              << std::endl;
  }

  void print() const {
    for (std::size_t v = 0; v < graph[FWD]->numVertices(); ++v) {
      std::cout << " -> " << v << "\n\t";
      for (auto h : labels[FWD][v].nodes) std::cout << h << " ";
      std::cout << "\n <- " << v << "\n\t";
      for (auto h : labels[BWD][v].nodes) std::cout << h << " ";
      std::cout << std::endl;
    }
  }

  std::vector<Vertex> getOrdering(const std::string &fileName) {
    std::vector<Vertex> ordering;
    ordering.reserve(graph[FWD]->numVertices());

    if (fileName == "") {
      ordering.assign(graph[FWD]->numVertices(), 0);

      std::iota(ordering.begin(), ordering.end(), 0);
      std::sort(ordering.begin(), ordering.end(),
                [&](const auto left, const auto right) {
                  return graph[FWD]->degree(left) + graph[BWD]->degree(left) >
                         graph[FWD]->degree(right) + graph[BWD]->degree(right);
                });
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
    labels[BWD].assign(numVertices, Label());
    labels[FWD].assign(numVertices, Label());

    lookup[BWD].assign(numVertices, false);
    lookup[FWD].assign(numVertices, false);

    alreadyProcessed.assign(numVertices, false);

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);

    counter[FWD].assign(numVertices, PathCounter(0));
    counter[BWD].assign(numVertices, PathCounter(0));

    topoRank.assign(numVertices, 0);
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

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    auto prune = [&](const auto &labels, const auto &lookup) -> bool {
      return std::any_of(labels.begin(), labels.end(),
                         [&](const Vertex h) { return lookup[h]; });
    };

    modifyLookups(v, true);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      bfs[dir].run(v, bfs::noOp, [&](const Vertex w) {
        return alreadyProcessed[w] || prune(labels[!dir][w].nodes, lookup[dir]);
      });
    };

    runOneDirection(FWD);
    bfs[FWD].doForAllVerticesInQ([&](const Vertex u) {
      assert(!labels[!FWD][u].contains(v));
      labels[!FWD][u].add(v);
    });

    runOneDirection(BWD);
    bfs[BWD].doForAllVerticesInQ([&](const Vertex u) {
      assert(!labels[!BWD][u].contains(v));
      labels[!BWD][u].add(v);
    });

    alreadyProcessed[v] = true;
    modifyLookups(v, false);
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

  std::vector<Vertex> sampleKVertices() {
    std::random_device rd;
    std::mt19937 g(rd());

    std::vector<Vertex> sample(graph[FWD]->numVertices(), 0);
    std::iota(sample.begin(), sample.end(), 0);

    std::shuffle(sample.begin(), sample.end(), g);
    sample.resize(std::min(static_cast<std::size_t>(K), sample.size()));
    return sample;
  }
};
