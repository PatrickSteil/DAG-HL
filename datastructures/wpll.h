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
#include "timestamped_vector.h"
#include "utils.h"

template <int K = 256>
struct WPLL {
  static constexpr int NUM_NIBBLES = K / 4;

 public:
  std::array<std::vector<Label>, 2> &labels;
  /* std::array<std::vector<SIMDContainer4bit<K>>, 2> &reachability; */
  std::array<const Graph *, 2> &graph;
  std::array<std::vector<Weight>, 2> lookup;
  std::array<TimestampedVector<Weight>, 2> distance;
  std::array<bfs::ZeroOneBFS, 2> bfs;

  WPLL(std::array<std::vector<Label>, 2> &labels,
       /* std::array<std::vector<SIMDContainer4bit<K>>, 2> &reachability, */
       std::array<const Graph *, 2> &graph)
      : labels(labels),
        /* reachability(reachability), */
        graph(graph),
        lookup{std::vector<Weight>(graph[FWD]->numVertices(), 0),
               std::vector<Weight>(graph[FWD]->numVertices(), 0)},
        distance{
            TimestampedVector<Weight>(graph[FWD]->numVertices(), noWeight),
            TimestampedVector<Weight>(graph[FWD]->numVertices(), noWeight)},
        bfs{bfs::ZeroOneBFS(*graph[FWD]), bfs::ZeroOneBFS(*graph[BWD])} {};

  void run(const std::vector<Vertex> &ordering) {
    StatusLog log("Computing Weighted HLs");
    assert(ordering.size() == graph[FWD]->numVertices());

    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    init(ordering.size());

    for (std::size_t i = 0; i < ordering.size(); ++i) {
      runPrunedBFS(ordering[i]);
    }
  }

  void init(std::size_t numVertices) {
    labels[BWD].assign(numVertices, Label());
    labels[FWD].assign(numVertices, Label());

    lookup[BWD].assign(numVertices, noWeight);
    lookup[FWD].assign(numVertices, noWeight);

    bfs[FWD].reset(numVertices);
    bfs[BWD].reset(numVertices);
  }

  void resetDistance(const DIRECTION dir) { distance[dir].reset(); }

  void runPrunedBFS(const Vertex v) {
    assert(v < labels[BWD].size());

    setLookup(v);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      resetDistance(dir);
      distance[dir].set(v, 0);

      bfs[dir].run(
          v,
          [&](const Vertex u) {
            if (distance[dir].get(u) == noWeight) return true;
            labels[!dir][u].add(v, distance[dir].get(u));
            return false;
          },
          [&](const Vertex u, const Vertex w, const Weight dist) {
            bool prune = true;
            const Weight newDistance = distance[dir].get(u) + dist;
            if (newDistance < distance[dir].get(w)) {
              distance[dir].set(w, newDistance);
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
        lookup[dir][hub] = noWeight;
      });
    };

    forDir(FWD);
    forDir(BWD);
  }
};
