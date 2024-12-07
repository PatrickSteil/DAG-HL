#pragma once

#include "graph.h"
#include <cmath>
#include <functional>
#include <iostream>
#include <vector>

struct TopologicalSeperator {
  const Graph &graph;
  const std::vector<Vertex> &topoOrder;

  TopologicalSeperator(const Graph &graph, const std::vector<Vertex> &topoOrder)
      : graph(graph), topoOrder(topoOrder){};

  std::vector<Vertex> run(const double imbalance) {
    assert(imbalance >= 0);
    assert(imbalance <= 1);

    std::function<void(std::vector<Vertex> &, std::size_t, std::size_t)>
        orderRecursive;
    orderRecursive = [&](std::vector<Vertex> &result, std::size_t start,
                         std::size_t end) {
      if (start >= end)
        return;

      std::size_t totalSize = end - start;
      std::size_t leftSize =
          static_cast<std::size_t>((totalSize / 2.0) * (1.0 - imbalance));
      std::size_t rightSize = totalSize - leftSize;

      std::size_t leftBound = start + leftSize;
      std::size_t rightBound = start + leftSize + rightSize;

      std::vector<Vertex> middleVertices(topoOrder.begin() + leftBound,
                                         topoOrder.begin() + rightBound);
      std::sort(middleVertices.begin(), middleVertices.end(),
                [&](Vertex a, Vertex b) {
                  return graph.degree(a) > graph.degree(b);
                });

      result.insert(result.end(), middleVertices.begin(), middleVertices.end());

      orderRecursive(result, start, leftBound);
      orderRecursive(result, rightBound, end);
    };

    std::vector<Vertex> result;
    orderRecursive(result, 0, topoOrder.size());
    return result;
  }
};
