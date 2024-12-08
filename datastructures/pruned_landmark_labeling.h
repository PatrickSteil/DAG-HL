#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "bfs.h"
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
    bool contains(const Vertex hub) {
      return std::find(nodes.begin(), nodes.end(), hub) != nodes.end();
    };
  };

  std::array<std::vector<Label>, 2> labels;
  std::array<std::vector<uint8_t>, 2> lookup;
  std::vector<uint8_t> alreadyProcessed;
  std::array<const Graph *, 2> graph;
  std::array<bfs::BFS, 2> bfs;

  PLL(const Graph &fwdGraph, const Graph &bwdGraph)
      : labels{std::vector<Label>(fwdGraph.numVertices()),
               std::vector<Label>(fwdGraph.numVertices())},
        lookup{std::vector<uint8_t>(fwdGraph.numVertices(), false),
               std::vector<uint8_t>(fwdGraph.numVertices(), false)},
        alreadyProcessed(fwdGraph.numVertices(), false),
        graph{&fwdGraph, &bwdGraph},
        bfs{bfs::BFS(fwdGraph), bfs::BFS(bwdGraph)} {};

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

  void print() const {
    for (std::size_t v = 0; v < graph[FWD]->numVertices(); ++v) {
      std::cout << " -> " << v << "\n\t";
      for (auto h : labels[FWD][v].nodes) std::cout << h << " ";
      std::cout << "\n <- " << v << "\n\t";
      for (auto h : labels[BWD][v].nodes) std::cout << h << " ";
      std::cout << std::endl;
    }
  }

 private:
  void init(std::size_t numVertices) {
    labels[BWD].assign(numVertices, Label());
    labels[FWD].assign(numVertices, Label());

    lookup[BWD].assign(numVertices, false);
    lookup[FWD].assign(numVertices, false);

    alreadyProcessed.assign(numVertices, false);

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);
  }

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    auto prune = [&](const auto &labels, const auto &lookup) -> bool {
      return std::any_of(labels.begin(), labels.end(),
                         [&](const Vertex h) { return lookup[h]; });
    };

    modifyLookups(v, true);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      bfs[dir].run(
          v, [](const Vertex & /* v */) { return false; },
          [&](const Vertex &w) {
            return alreadyProcessed[w] ||
                   prune(labels[!dir][w].nodes, lookup[dir]);
          });
    };

    runOneDirection(FWD);
    bfs[FWD].doForAllVerticesInQ([&](const Vertex u) {
      assert(!labels[BWD][u].contains(v));
      labels[BWD][u].add(v);
    });

    runOneDirection(BWD);
    bfs[BWD].doForAllVerticesInQ([&](const Vertex u) {
      assert(!labels[FWD][u].contains(v));
      labels[FWD][u].add(v);
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
};
