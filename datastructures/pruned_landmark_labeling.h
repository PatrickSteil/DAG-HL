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
#include "graph.h"
#include "hub_labels.h"
#include "status_log.h"
#include "utils.h"

template <int WIDTH = 256, class LABEL = Label>
struct PLL {
 public:
  std::array<std::vector<LABEL>, 2> &labels;
  std::array<std::vector<std::bitset<WIDTH>>, 2> &reachability;
  std::vector<std::atomic<bool>> &alreadyProcessed;
  std::array<const Graph *, 2> &graph;
  std::array<std::vector<std::uint16_t>, 2> lookup;
  std::array<bfs::BFS, 2> bfs;
  std::uint16_t generation;

  PLL(std::array<std::vector<LABEL>, 2> &labels,
      std::array<std::vector<std::bitset<WIDTH>>, 2> &reachability,
      std::vector<std::atomic<bool>> &alreadyProcessed,
      std::array<const Graph *, 2> &graph)
      : labels(labels),
        reachability(reachability),
        alreadyProcessed(alreadyProcessed),
        graph(graph),
        lookup{std::vector<std::uint16_t>(graph[FWD]->numVertices(), 0),
               std::vector<std::uint16_t>(graph[FWD]->numVertices(), 0)},
        bfs{bfs::BFS(*graph[FWD]), bfs::BFS(*graph[BWD])},
        generation(0){};

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

    lookup[BWD].assign(numVertices, 0);
    lookup[FWD].assign(numVertices, 0);
    generation = 1;

    alreadyProcessed = std::vector<std::atomic<bool>>(numVertices, false);

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);
  }

  template <bool PRUNE_VIA_BITSET = true>
  void runPrunedBFS(const std::size_t left, const std::size_t i,
                    const std::vector<Vertex> &ordering) {
    assert(left + i < ordering.size());
    assert(i < WIDTH);

    const Vertex v = ordering[left + i];

    setLookup(v);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      bfs[dir].run(v, bfs::noOp, [&](const Vertex w) {
        bool prune = alreadyProcessed[w].load(std::memory_order_relaxed) ||
                     labels[!dir][w].appliesToAny([&](const Vertex h) {
                       return (lookup[dir][h] == generation);
                     });
        if constexpr (PRUNE_VIA_BITSET) {
          assert(reachability[dir][w].any());
          assert(reachability[!dir][v].any());

          auto result =
              findFirstOne(reachability[dir][w], reachability[!dir][v]);
          prune |= (result < i);
        }
        return prune;
      });
    };

    /* #pragma omp parallel for num_threads(2) */
    for (auto dir : {FWD, BWD}) {
      runOneDirection(dir);
    }

    /* #pragma omp parallel for num_threads(2) */
    for (auto dir : {FWD, BWD}) {
      bfs[dir].doForAllVerticesInQ([&](const Vertex u) {
        assert(!labels[!dir][u].contains(v));
        labels[!dir][u].add(v);
      });
    }

    alreadyProcessed[v].store(true, std::memory_order_relaxed);
  }

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    setLookup(v);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      bfs[dir].run(v, bfs::noOp, [&](const Vertex w) {
        return alreadyProcessed[w].load(std::memory_order_relaxed) ||
               labels[!dir][w].appliesToAny([&](const Vertex h) {
                 return (lookup[dir][h] == generation);
               });
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

    alreadyProcessed[v].store(true, std::memory_order_relaxed);
  }

  void setLookup(const Vertex v) {
    generation++;

    if (generation == 0) {
      std::fill(lookup[FWD].begin(), lookup[FWD].end(), 0);
      std::fill(lookup[BWD].begin(), lookup[BWD].end(), 0);

      generation = 1;
    }

    auto forDir = [&](const DIRECTION dir) -> void {
      labels[dir][v].doForAll([&](const Vertex hub) {
        assert(hub < lookup[dir].size());
        lookup[dir][hub] = generation;
      });
    };

    forDir(FWD);
    forDir(BWD);
  }
};
