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

template <int WIDTH = 256, class LABEL = Label>
struct PLL {
 public:
  std::array<std::vector<LABEL>, 2> &labels;
  std::array<std::vector<std::bitset<WIDTH>>, 2> &reachability;
  std::array<const Graph *, 2> &graph;
  std::array<std::vector<std::uint8_t>, 2> lookup;
  std::array<bfs::BFS, 2> bfs;

  PLL(std::array<std::vector<LABEL>, 2> &labels,
      std::array<std::vector<std::bitset<WIDTH>>, 2> &reachability,
      std::array<const Graph *, 2> &graph)
      : labels(labels),
        reachability(reachability),
        graph(graph),
        lookup{std::vector<std::uint8_t>(graph[FWD]->numVertices(), 0),
               std::vector<std::uint8_t>(graph[FWD]->numVertices(), 0)},
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

    lookup[BWD].assign(numVertices, 0);
    lookup[FWD].assign(numVertices, 0);

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
      bfs[dir].run(
          v,
          [&](const Vertex u) {
            assert(!labels[!dir][u].contains(v));
            labels[!dir][u].add(v);
            return false;
          },
          [&](const Vertex /* u */, const Vertex w) -> bool {
            bool prune = false;
            if constexpr (PRUNE_VIA_BITSET) {
              assert(reachability[dir][w].any());
              assert(reachability[!dir][v].any());

              prune |= (findFirstOne(reachability[dir][w],
                                     reachability[!dir][v]) < i);
            }
            prune |= labels[!dir][w].prune(lookup[dir]);

            return prune;
          });
    };

    for (auto dir : {FWD, BWD}) {
      runOneDirection(dir);
    }

    unsetLookup(v);
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
            bool prune = labels[!dir][w].prune(lookup[dir]);
            return prune;
          });
    };

    /* #pragma omp parallel for num_threads(2) */
    for (auto dir : {FWD, BWD}) {
      runOneDirection(dir);
    }

    unsetLookup(v);
  }

  void growTree(const Vertex v, EdgeTree<std::vector<Index>> &tree) {
    assert(v < labels[BWD].size());
    tree.reset(v);

    setLookup(v);

    auto runOneDirection = [&](const DIRECTION dir) -> void {
      bfs[dir].run(v, bfs::noOp, [&](const Vertex u, const Vertex w) {
        bool prune = labels[!dir][w].prune(lookup[dir]);

        if (!prune) {
          tree.addEdge(u, w, dir);
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

  std::vector<Vertex> getOrdering(const std::string &fileName) {
    std::vector<Vertex> ordering;
    ordering.reserve(graph[FWD]->numVertices());

    if (fileName == "") {
      parallel_assign(ordering, graph[FWD]->numVertices(), Vertex(0));

      std::vector<std::size_t> randomNumber;
      parallel_assign_iota(randomNumber, graph[FWD]->numVertices(),
                           static_cast<std::size_t>(0));

      std::mt19937 g(42);

      std::shuffle(randomNumber.begin(), randomNumber.end(), g);

      auto degreeCompRandom = [&](const auto left, const auto right) {
        return std::forward_as_tuple(
                   graph[FWD]->degree(left) + graph[BWD]->degree(left),
                   randomNumber[left]) >
               std::forward_as_tuple(
                   graph[FWD]->degree(right) + graph[BWD]->degree(right),
                   randomNumber[right]);
      };

      parallel_iota(ordering, Vertex(0));
      ips4o::parallel::sort(ordering.begin(), ordering.end(), degreeCompRandom);
      /* std::sort(ordering.begin(), ordering.end(), degreeCompRandom); */
      assert(
          std::is_sorted(ordering.begin(), ordering.end(), degreeCompRandom));
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
};
