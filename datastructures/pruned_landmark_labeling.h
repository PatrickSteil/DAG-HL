#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "bfs_tools.h"
#include "graph.h"
#include "status_log.h"

struct PLL {
public:
  enum DIRECTION : bool { FWD, BWD };
  struct alignas(64) Label {
    Label(){};

    std::vector<Vertex> nodes;

    Vertex &operator[](std::size_t i) { return nodes[i]; }
    const Vertex &operator[](std::size_t i) const { return nodes[i]; }

    std::size_t size() const { return nodes.size(); }
    void add(const Vertex hub) { nodes.push_back(hub); };
  };

  std::array<std::vector<Label>, 2> labels;
  std::array<std::vector<uint8_t>, 2> lookup;
  std::vector<uint8_t> alreadyProcessed;
  std::array<const Graph *, 2> graph;
  std::array<bfs::FixedSizedQueue<Vertex>, 2> q;
  std::array<bfs::GenerationChecker<>, 2> seen;

  PLL(const Graph &fwdGraph, const Graph &bwdGraph)
      : labels{std::vector<Label>(fwdGraph.numVertices()),
               std::vector<Label>(fwdGraph.numVertices())},
        lookup{std::vector<uint8_t>(fwdGraph.numVertices(), false),
               std::vector<uint8_t>(fwdGraph.numVertices(), false)},
        alreadyProcessed(fwdGraph.numVertices(), false),
        graph{&fwdGraph, &bwdGraph},
        q{bfs::FixedSizedQueue<Vertex>(fwdGraph.numVertices()),
          bfs::FixedSizedQueue<Vertex>(fwdGraph.numVertices())},
        seen{bfs::GenerationChecker<>(fwdGraph.numVertices()),
             bfs::GenerationChecker<>(fwdGraph.numVertices())} {};

  void run(const std::vector<Vertex> &ordering) {
    StatusLog log("Computing HLs");
    assert(ordering.size() == graph[FWD]->numVertices());

    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    init(ordering.size());

    for (std::size_t i = 0; i < ordering.size(); ++i) {
      runPrunedBFS(ordering[i]);
    }
  }

  void saveToFile(const std::string &fileName) {
    std::cout << "TODO implement me, saveToFile(" << fileName << ")\n";
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
  }

private:
  void init(std::size_t numVertices) {
    labels[BWD].assign(numVertices, Label());
    labels[FWD].assign(numVertices, Label());

    lookup[BWD].assign(numVertices, false);
    lookup[FWD].assign(numVertices, false);

    alreadyProcessed.assign(numVertices, false);

    q[FWD].reset();
    q[BWD].reset();
    seen[FWD].reset();
    seen[BWD].reset();

    q[FWD].resize(numVertices);
    q[BWD].resize(numVertices);
    seen[FWD].resize(numVertices);
    seen[BWD].resize(numVertices);
  }

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    auto prune = [&](const auto &labels, const auto &lookup) -> bool {
      return std::any_of(labels.begin(), labels.end(),
                         [&](const Vertex h) { return lookup[h]; });
    };

    modifyLookups(v, true);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      q[dir].reset();
      seen[dir].reset();

      q[dir].push(v);
      seen[dir].mark(v);

      const auto &toVertex = graph[dir]->toVertex;

      while (!q[dir].isEmpty()) {
        const Vertex u = q[dir].pop();

#pragma GCC unroll 4
        for (std::size_t i = graph[dir]->beginEdge(u);
             i < graph[dir]->endEdge(u); ++i) {
          const Vertex w = toVertex[i];

          bool skip = (seen[dir].isMarked(w) || alreadyProcessed[w]);
          seen[dir].mark(w);

          if (skip || prune(labels[!dir][w].nodes, lookup[dir]))
            continue;
          q[dir].push(w);
        }
      }
    };

    auto addToLabels = [&](const DIRECTION dir) -> void {
#pragma GCC unroll 4
      for (std::size_t i = 0; i < q[dir].read; ++i) {
        if (i + 4 < q[dir].read) {
          PREFETCH(&labels[!dir][q[dir].data[i + 4]]);
        }

        assert(i < q[dir].data.size());
        const Vertex u = q[dir].data[i];

        labels[!dir][u].add(v);
      }
    };

    runOneDirection(FWD);
    addToLabels(FWD);

    runOneDirection(BWD);
    addToLabels(BWD);

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
};
