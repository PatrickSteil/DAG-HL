/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <omp.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "bfs.h"
#include "forest.h"
#include "graph.h"
#include "hub_labels.h"
#include "status_log.h"
#include "utils.h"

template <class LABEL = Label>
struct PLLFrontierBFS {
 public:
  std::array<std::vector<LABEL>, 2> &labels;
  std::vector<std::atomic<bool>> &alreadyProcessed;
  std::array<const Graph *, 2> &graph;
  std::array<std::vector<std::uint8_t>, 2> lookup;
  std::array<bfs::BFSParallelFrontier, 2> bfs;

  PLLFrontierBFS(std::array<std::vector<LABEL>, 2> &labels,
                 std::vector<std::atomic<bool>> &alreadyProcessed,
                 std::array<const Graph *, 2> &graph, const int numThreads = 1)
      : labels(labels),
        alreadyProcessed(alreadyProcessed),
        graph(graph),
        lookup{std::vector<std::uint8_t>(graph[FWD]->numVertices(), 0),
               std::vector<std::uint8_t>(graph[FWD]->numVertices(), 0)},
        bfs{bfs::BFSParallelFrontier(*graph[FWD], numThreads),
            bfs::BFSParallelFrontier(*graph[BWD], numThreads)} {};

  void run(const std::vector<Vertex> &ordering) {
    StatusLog log("Computing HLs");
    assert(ordering.size() == graph[FWD]->numVertices());

    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    init(ordering.size());

    for (std::size_t i = 0; i < ordering.size(); ++i) {
      runPrunedBFS(i, ordering);
    }
  }

  void init(std::size_t numVertices) {
    labels[BWD].assign(numVertices, LABEL());
    labels[FWD].assign(numVertices, LABEL());

    lookup[BWD].assign(numVertices, 0);
    lookup[FWD].assign(numVertices, 0);

    alreadyProcessed = std::vector<std::atomic<bool>>(numVertices, false);

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);
  }

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    setLookup(v);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      bfs[dir].run(
          v,
          [&](const Vertex u) {
            assert(!labels[!dir][u].contains(v));
            labels[!dir][u].add(v);
            return false;
          },
          [&](const Vertex /* u */, const Vertex w) {
            return alreadyProcessed[w].load(std::memory_order_relaxed) ||
                   labels[!dir][w].prune(lookup[dir]);
          });
    };

#pragma omp parallel for num_threads(2)
    for (auto dir : {FWD, BWD}) {
      runOneDirection(dir);
    }

    unsetLookup(v);

    alreadyProcessed[v].store(true, std::memory_order_relaxed);
  }

  void setLookup(const Vertex v) {
    auto forDir = [&](const DIRECTION dir) -> void {
      labels[dir][v].doForAll([&](const Vertex hub) {
        assert(hub < lookup[dir].size());
        lookup[dir][hub] = 1;
      });
    };

    forDir(FWD);
    forDir(BWD);
  }

  void unsetLookup(const Vertex v) {
    auto forDir = [&](const DIRECTION dir) -> void {
      labels[dir][v].doForAll([&](const Vertex hub) {
        assert(hub < lookup[dir].size());
        lookup[dir][hub] = 0;
      });
    };

    forDir(FWD);
    forDir(BWD);
  }
};
