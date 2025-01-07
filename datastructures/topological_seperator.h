/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <queue>
#include <random>
#include <vector>

#include "graph.h"

struct TopologicalSeperator {
  std::array<const Graph *, 2> graphs;
  const std::vector<Vertex> &topoOrder;

  TopologicalSeperator(const Graph &fwd, const Graph &bwd,
                       const std::vector<Vertex> &topoOrder)
      : graphs{&fwd, &bwd}, topoOrder(topoOrder){};

  std::vector<Vertex> run(const double imbalance) {
    assert(imbalance >= 0);
    assert(imbalance <= 1);

    std::queue<std::pair<std::size_t, std::size_t>> ranges;
    ranges.push({0, topoOrder.size()});

    std::vector<Vertex> result;
    result.reserve(topoOrder.size());

    auto rng = std::default_random_engine{};
    while (!ranges.empty()) {
      auto [start, end] = ranges.front();
      ranges.pop();

      if (start >= end) continue;

      std::size_t totalSize = end - start;
      std::size_t leftSize =
          static_cast<std::size_t>((totalSize / 2.0) * (1.0 - imbalance));
      std::size_t rightSize = end - leftSize;

      std::size_t leftBound = start + leftSize;
      std::size_t rightBound = rightSize;

      std::vector<Vertex> middleVertices(topoOrder.begin() + leftBound,
                                         topoOrder.begin() + rightBound);

      std::ranges::shuffle(middleVertices, rng);
      /* std::sort(middleVertices.begin(), middleVertices.end(), */
      /*           [&](Vertex a, Vertex b) { */
      /*             return graphs[FWD]->degree(a) + graphs[BWD]->degree(a) > */
      /*                    graphs[FWD]->degree(b) + graphs[BWD]->degree(b); */
      /*           }); */
      result.insert(result.end(), middleVertices.begin(), middleVertices.end());

      ranges.push({start, leftBound});
      ranges.push({rightBound, end});
    };

    return result;
  }
};
