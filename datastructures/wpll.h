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
#include "ips4o.hpp"
#include "status_log.h"
#include "utils.h"

template <int K = 256>
struct WPLL {
  static constexpr int NUM_NIBBLES = K / 4;

 public:
  std::array<std::vector<Label>, 2> &labels;
  /* std::array<std::vector<SIMDContainer4bit<K>>, 2> &reachability; */
  std::array<const Graph *, 2> &graph;
  std::array<std::vector<Weight>, 2> lookup;
  std::array<std::vector<Weight>, 2> distance;  // TODO maybe timstamped stuff
  std::array<bfs::ZeroOneBFS, 2> bfs;

  WPLL(std::array<std::vector<Label>, 2> &labels,
       /* std::array<std::vector<SIMDContainer4bit<K>>, 2> &reachability, */
       std::array<const Graph *, 2> &graph)
      : labels(labels),
        /* reachability(reachability), */
        graph(graph),
        lookup{std::vector<Weight>(graph[FWD]->numVertices(), 0),
               std::vector<Weight>(graph[FWD]->numVertices(), 0)},
        bfs{bfs::ZeroOneBFS(*graph[FWD]), bfs::ZeroOneBFS(*graph[BWD])} {};

  void run(const std::vector<Vertex> &ordering) {
    StatusLog log("Computing Weighted HLs");
    assert(ordering.size() == graph[FWD]->numVertices());

    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    init(ordering.size());
    /* runPrunedBFS(ordering[0]); */

    for (std::size_t i = 0; i < ordering.size(); ++i) {
      runPrunedBFS(ordering[i]);
    }
  }

  void sortLabels() {
#pragma omp parallel for
    for (Vertex v = 0; v < graph[FWD]->numVertices(); ++v) {
      labels[FWD][v].sort();
      labels[BWD][v].sort();
    }
  }

  void init(std::size_t numVertices) {
    labels[BWD].assign(numVertices, Label());
    labels[FWD].assign(numVertices, Label());

    lookup[BWD].assign(numVertices, 0);
    lookup[FWD].assign(numVertices, 0);

    distance[BWD].assign(numVertices, noWeight);
    distance[FWD].assign(numVertices, noWeight);

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);
  }

  void resetDistance(const DIRECTION dir) {
    std::fill(distance[dir].begin(), distance[dir].end(), noWeight);
  }

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    setLookup(v);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      resetDistance(dir);
      distance[dir][v] = 0;

      bfs[dir].run(
          v,
          [&](const Vertex u) {
            /* assert(!labels[!dir][u].contains(v)); */
            assert(v < distance[dir].size());
            if (distance[dir][u] == noWeight) return true;
            labels[!dir][u].add(v, distance[dir][u]);
            return false;
          },
          [&](const Vertex u, const Vertex w, const Weight dist) {
            bool prune = true;
            assert(distance[dir][u] != noWeight);
            const Weight newDistance = distance[dir][u] + dist;
            if (newDistance < distance[dir][w]) {
              distance[dir][w] = newDistance;
              prune = labels[!dir][w].prune(lookup[dir], newDistance);
            }
            return prune;
          });
    };

    for (auto dir : {FWD, BWD}) {
      runOneDirection(dir);
    }

    unsetLookup(v);
  }

  void setLookup(const Vertex v) {
    auto forDir = [&](const DIRECTION dir) -> void {
      labels[dir][v].doForAll([&](const Vertex hub, const Weight weight) {
        assert(hub < lookup[dir].size());
        lookup[dir][hub] = weight;
      });
    };

    forDir(FWD);
    forDir(BWD);
  }

  void unsetLookup(const Vertex v) {
    auto forDir = [&](const DIRECTION dir) -> void {
      labels[dir][v].doForAll([&](const Vertex hub, const Weight /* weight */) {
        assert(hub < lookup[dir].size());
        lookup[dir][hub] = 0;
      });
    };

    forDir(FWD);
    forDir(BWD);
  }
};
