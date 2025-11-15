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
#include <bitset>
#include <cassert>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <vector>

#include "graph.h"
#include "hub_labels.h"
#include "ips4o.hpp"
#include "pruned_landmark_labeling.h"
#include "simd.h"
#include "status_log.h"
#include "topological_sort.h"
#include "utils.h"

template <int K = 128, class LABEL = Label>
struct WeightedPLL {
  static constexpr int NUM_NIBBLES = K / 4;

 public:
  std::array<std::vector<LABEL>, 2> labels;

  std::array<const Graph *, 2> graph;
  std::vector<std::size_t> rank;

  std::vector<Edge> edges;
  std::array<std::vector<SIMDContainer4bit<K>>, 2> reachability;

  const int numThreads;

  WeightedPLL(const Graph &fwdGraph, const Graph &bwdGraph,
              const std::vector<std::size_t> &rankPar, const int numThreads = 1)
      : labels{std::vector<LABEL>(), std::vector<LABEL>()},
        graph{&fwdGraph, &bwdGraph},
        rank(rankPar),
        edges(),
        reachability{std::vector<SIMDContainer4bit<K>>(fwdGraph.numVertices()),
                     std::vector<SIMDContainer4bit<K>>(fwdGraph.numVertices())},
        numThreads(numThreads) {
    init(fwdGraph.numVertices());
  };

  void run(const std::vector<Vertex> &ordering) {
    StatusLog log("Computing HLs");
    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    assert(isOrdering(ordering, graph[FWD]->numVertices()));

    runSweep(0, ordering);

    // TODO
  }

  void init(std::size_t numVertices) {
    StatusLog log("Init the datastructures");

    parallel_assign(labels[BWD], numVertices, LABEL());
    parallel_assign(labels[FWD], numVertices, LABEL());

    edges.reserve(graph[FWD]->numEdges());

    for (Vertex from = 0; from < numVertices; ++from) {
      std::size_t i = graph[FWD]->beginEdge(from);
      const std::size_t end = graph[FWD]->endEdge(from);
      for (; i < end; ++i) {
        edges.emplace_back(from, graph[FWD]->toVertex[i],
                           graph[FWD]->weight[i]);
      }
    }

    ips4o::parallel::sort(edges.begin(), edges.end(),
                          [&](const auto &left, const auto &right) {
                            return std::tie(rank[left.from], rank[left.to]) <
                                   std::tie(rank[right.from], rank[right.to]);
                          });
    assert(std::is_sorted(edges.begin(), edges.end(),
                          [&](const auto &left, const auto &right) {
                            return std::tie(rank[left.from], rank[left.to]) <
                                   std::tie(rank[right.from], rank[right.to]);
                          }));
  }

  void runSweep(const std::size_t left, const std::vector<Vertex> &ordering) {
    assert(left + NUM_NIBBLES < ordering.size());
    resetReachability();

#pragma GCC unroll(4)
    for (int i = 0; i < NUM_NIBBLES; ++i) {
      const auto vertex = ordering[left + i];

      assert(vertex < reachability[FWD].size());
      assert(vertex < reachability[BWD].size());

      reachability[FWD][vertex].set(i, 0);
      reachability[BWD][vertex].set(i, 0);
    }

    auto fwdSweep = [&]() -> void {
      for (std::size_t i = 0; i < edges.size(); ++i) {
        const auto &edge = edges[i];

        assert(edge.to < reachability[FWD].size());
        assert(edge.from < reachability[FWD].size());
        assert(edge.weight == 0 || edge.weight == 1);

        reachability[FWD][edge.to].increment_clamp(reachability[FWD][edge.from],
                                                   edge.weight);
      }
    };

    auto bwdSweep = [&]() -> void {
      for (std::size_t i = edges.size(); i-- > 0;) {
        const auto &edge = edges[i];

        assert(edge.to < reachability[BWD].size());
        assert(edge.from < reachability[BWD].size());
        assert(edge.weight == 0 || edge.weight == 1);

        reachability[BWD][edge.from].increment_clamp(reachability[BWD][edge.to],
                                                     edge.weight);
      }
    };

    /* fwdSweep(); */
    /* bwdSweep(); */

    std::thread fwdThread(fwdSweep);
    std::thread bwdThread(bwdSweep);

    fwdThread.join();
    bwdThread.join();
  }

  void resetReachability() {
    for (Vertex v = 0; v < graph[FWD]->numVertices(); ++v) {
      assert(v < reachability[FWD].size());
      reachability[FWD][v].reset();
    }
    for (Vertex v = 0; v < graph[FWD]->numVertices(); ++v) {
      assert(v < reachability[BWD].size());
      reachability[BWD][v].reset();
    }
  }
};
