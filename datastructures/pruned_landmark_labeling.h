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
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <thread>
#include <vector>

#include "bfs.h"
#include "graph.h"
#include "hub_labels.h"
#include "status_log.h"

template <class LABEL = Label>
struct PLL {
 public:
  std::array<std::vector<LABEL>, 2> &labels;
  std::array<std::vector<uint8_t>, 2> &lookup;
  std::vector<uint8_t> &alreadyProcessed;
  std::array<const Graph *, 2> &graph;
  std::array<bfs::BFS, 2> bfs;

  PLL(std::array<std::vector<LABEL>, 2> &labels,
      std::array<std::vector<uint8_t>, 2> &lookup,
      std::vector<uint8_t> &alreadyProcessed,
      std::array<const Graph *, 2> &graph)
      : labels(labels),
        lookup(lookup),
        alreadyProcessed(alreadyProcessed),
        graph(graph),
        bfs{bfs::BFS(*graph[FWD]), bfs::BFS(*graph[BWD])} {};

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

  void init(std::size_t numVertices) {
    labels[BWD].assign(numVertices, LABEL());
    labels[FWD].assign(numVertices, LABEL());

    lookup[BWD].assign(numVertices, false);
    lookup[FWD].assign(numVertices, false);

    alreadyProcessed.assign(numVertices, false);

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);
  }

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    modifyLookups(v, true);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      bfs[dir].run(v, bfs::noOp, [&](const Vertex w) {
        return alreadyProcessed[w] ||
               labels[!dir][w].appliesToAny(
                   [&](const Vertex h) { return lookup[dir][h]; });
      });
    };

#pragma omp parallel for num_threads(2)
    for (auto dir : {FWD, BWD}) {
      runOneDirection(dir);
    }

#pragma omp parallel for num_threads(2)
    for (auto dir : {FWD, BWD}) {
      bfs[dir].doForAllVerticesInQ([&](const Vertex u) {
        assert(!labels[!dir][u].contains(v));
        labels[!dir][u].add(v);
      });
    }

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
