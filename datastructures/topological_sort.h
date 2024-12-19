/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <queue>
#include <stdexcept>
#include <vector>

#include "graph.h"
#include "status_log.h"

struct TopologicalSort {
  std::vector<Vertex> ordering;

  TopologicalSort(const Graph &graph) {
    StatusLog log("Computing a topological ordering");
    const std::size_t numVertices = graph.numVertices();
    ordering.reserve(numVertices);

    std::vector<std::size_t> inDegree(numVertices, 0);
    for (Vertex v = 0; v < numVertices; ++v) {
      for (std::size_t edge = graph.beginEdge(v); edge < graph.endEdge(v);
           ++edge) {
        ++inDegree[graph.toVertex[edge]];
      }
    }

    std::queue<Vertex> zeroInDegree;
    for (Vertex v = 0; v < numVertices; ++v) {
      if (inDegree[v] == 0) {
        zeroInDegree.push(v);
      }
    }

    while (!zeroInDegree.empty()) {
      Vertex v = zeroInDegree.front();
      zeroInDegree.pop();
      ordering.push_back(v);

      for (std::size_t edge = graph.beginEdge(v); edge < graph.endEdge(v);
           ++edge) {
        Vertex neighbor = graph.toVertex[edge];
        --inDegree[neighbor];
        if (inDegree[neighbor] == 0) {
          zeroInDegree.push(neighbor);
        }
      }
    }

    if (ordering.size() != numVertices) {
      throw std::runtime_error("The graph is not a DAG (cycle detected)");
    }
  }

  std::vector<Vertex> &getOrdering() { return ordering; }
};
