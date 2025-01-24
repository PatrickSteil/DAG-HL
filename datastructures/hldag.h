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
#include "status_log.h"
#include "topological_sort.h"
#include "utils.h"

template <int WIDTH = 256, class LABEL = Label>
struct HLDAG {
 public:
  std::array<std::vector<LABEL>, 2> labels;

  std::array<const Graph *, 2> graph;
  std::vector<std::size_t> rank;

  std::vector<Edge> edges;

  std::array<std::vector<std::bitset<WIDTH>>, 2> reachability;

  std::vector<PLL<WIDTH, LABEL>> plls;

  const int numThreads;

  HLDAG(const Graph &fwdGraph, const Graph &bwdGraph,
        const std::vector<std::size_t> &rankPar, const int numThreads = 1)
      : labels{std::vector<LABEL>(), std::vector<LABEL>()},
        graph{&fwdGraph, &bwdGraph},
        rank(rankPar),
        edges(),
        reachability{std::vector<std::bitset<WIDTH>>(),
                     std::vector<std::bitset<WIDTH>>()},
        plls(numThreads, PLL<WIDTH, LABEL>(labels, reachability, graph)),
        numThreads(numThreads) {
    init(fwdGraph.numVertices());
  };

  template <bool PARALL_TAIL = true>
  void run(const std::string &orderingFileName) {
    StatusLog log("Computing HLs");
    assert(graph[FWD]->numVertices() == graph[BWD]->numVertices());
    assert(graph[FWD]->numEdges() == graph[BWD]->numEdges());

    const std::size_t numVertices = graph[FWD]->numVertices();

    std::vector<Vertex> ordering = getOrdering(orderingFileName, graph);

    assert(ordering.size() == numVertices);
    assert(isOrdering(ordering, numVertices));

    std::size_t i = 0;

    if (numThreads > 1) {
      for (; i + WIDTH < (numVertices >> 7); i += WIDTH) {
        runSweep(i, ordering);

#pragma omp parallel for schedule(dynamic, 1) num_threads(numThreads)
        for (std::size_t step = 0; step < WIDTH; ++step) {
          int threadId = omp_get_thread_num();
          plls[threadId].template runPrunedBFS<true>(i, step, ordering);
        }
      }
    }

    if constexpr (PARALL_TAIL) {
#pragma omp parallel for schedule(dynamic, 8) num_threads(numThreads)
      for (std::size_t j = i; j < numVertices; ++j) {
        int threadId = omp_get_thread_num();
        plls[threadId].template runPrunedBFS<false>(j, 0, ordering);
      }
    } else {
      for (std::size_t j = i; j < numVertices; ++j) {
        plls[0].runPrunedBFS(ordering[j]);
      }
    }
  }

  void runSweep(const std::size_t left, const std::vector<Vertex> &ordering) {
    assert(left + WIDTH < ordering.size());
    resetReachability();

#pragma GCC unroll(4)
    for (int i = 0; i < WIDTH; ++i) {
      const auto vertex = ordering[left + i];

      reachability[FWD][vertex][i] = 1;
      reachability[BWD][vertex][i] = 1;
    }

    auto fwdSweep = [&]() -> void {
      for (std::size_t i = 0; i < edges.size(); ++i) {
        const auto &edge = edges[i];
        reachability[FWD][edge.to] |= reachability[FWD][edge.from];
      }
    };

    auto bwdSweep = [&]() -> void {
      for (std::size_t i = edges.size(); i-- > 0;) {
        const auto &edge = edges[i];
        reachability[BWD][edge.from] |= reachability[BWD][edge.to];
      }
    };

    /* fwdSweep(); */
    /* bwdSweep(); */

    std::thread fwdThread(fwdSweep);
    std::thread bwdThread(bwdSweep);

    fwdThread.join();
    bwdThread.join();
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

  void init(std::size_t numVertices) {
    StatusLog log("Init the datastructures");

    parallel_assign(labels[BWD], numVertices, LABEL());
    parallel_assign(labels[FWD], numVertices, LABEL());

    edges.reserve(graph[FWD]->numEdges());

    for (Vertex from = 0; from < numVertices; ++from) {
      for (std::size_t i = graph[FWD]->beginEdge(from);
           i < graph[FWD]->endEdge(from); ++i) {
        edges.emplace_back(from, graph[FWD]->toVertex[i]);
      }
    }

    /* std::sort(edges.begin(), edges.end(), */
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

  void resetReachability() {
    parallel_assign(reachability[BWD], graph[FWD]->numVertices(),
                    std::bitset<WIDTH>());
    parallel_assign(reachability[FWD], graph[FWD]->numVertices(),
                    std::bitset<WIDTH>());
  }
};
