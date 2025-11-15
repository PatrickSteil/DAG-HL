/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include "ips4o.hpp"
#include "status_log.h"
#include "types.h"
#include "utils.h"

struct Edge {
  Vertex from;
  Vertex to;
  Weight weight;

  Edge() = default;
  Edge(Vertex from, Vertex to, Weight weight)
      : from(from), to(to), weight(weight) {}

  auto operator<=>(const Edge &other) const = default;
};

struct Graph {
  std::vector<std::size_t> adjArray;
  std::vector<Vertex> toVertex;
  std::vector<Weight> weight;

  Graph() : adjArray(1), toVertex(), weight(){};

  Graph(const Graph &other)
      : adjArray(other.adjArray),
        toVertex(other.toVertex),
        weight(other.weight){};

  Graph(Graph &&other) noexcept
      : adjArray(std::move(other.adjArray)),
        toVertex(std::move(other.toVertex)),
        weight(std::move(other.weight)) {}

  bool isValid(const Vertex v) const { return v < numVertices(); }

  std::size_t numVertices() const { return adjArray.size() - 1; }
  std::size_t numEdges() const { return toVertex.size(); }

  void print() const {
    std::cout << "NumVertices: " << numVertices() << std::endl;
    std::cout << "NumEdges: " << numEdges() << std::endl;

    for (Vertex v = 0; v < numVertices(); ++v) {
      std::cout << "Edges from " << v << std::endl;

      for (std::size_t i = beginEdge(v); i < endEdge(v); ++i) {
        std::cout << toVertex[i] << " (" << weight[i] << ") ";
      }
      std::cout << std::endl;
    }
  }

  std::size_t degree(const Vertex v) const {
    assert(isValid(v));
    return endEdge(v) - beginEdge(v);
  }

  std::size_t beginEdge(const Vertex v) const {
    assert(isValid(v));
    assert(v < adjArray.size());
    return adjArray[v];
  }

  std::size_t endEdge(const Vertex v) const {
    assert(isValid(v));
    assert(v + 1 < adjArray.size());
    return adjArray[v + 1];
  }

  void clear() {
    adjArray.clear();
    toVertex.clear();
    weight.clear();
  }

  void readFromEdgeList(const std::string &fileName) {
    StatusLog log("Reading weighted graph from edgelist");
    clear();

    std::ifstream file(fileName);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open file: " + fileName);
    }

    std::vector<std::tuple<Vertex, Vertex, Weight>> edges;
    Vertex maxVertex = 0;

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      Vertex u, v;
      if (!(iss >> u >> v)) {
        continue;
      }

      edges.emplace_back(u - 1, v - 1, 0);  // Default edge weight is 0
      maxVertex = std::max({maxVertex, u - 1, v - 1});
    }

    file.close();

    adjArray.resize(maxVertex + 2, 0);

    /* std::sort(edges.begin(), edges.end(), */
    ips4o::parallel::sort(edges.begin(), edges.end());
    for (const auto &[u, v, w] : edges) {
      ++adjArray[u + 1];
    }

    for (std::size_t i = 1; i < adjArray.size(); ++i) {
      adjArray[i] += adjArray[i - 1];
    }

    toVertex.resize(edges.size());
    weight.resize(edges.size());
    std::vector<std::size_t> offset = adjArray;

    for (const auto &[u, v, w] : edges) {
      toVertex[offset[u]] = v;
      weight[offset[u]++] = w;
    }
  }

  void readDimacs(const std::string &fileName) {
    StatusLog log("Reading graph from dimacs");
    clear();

    std::ifstream file(fileName);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open file: " + fileName);
    }

    std::string line;
    std::vector<std::tuple<Vertex, Vertex, Weight>> edges;
    Vertex numVertices = 0, numEdges = 0;

    while (std::getline(file, line)) {
      if (line.empty() || line[0] == 'c') {
        continue;
      }

      if (line[0] == 'p') {
        std::istringstream iss(line);
        std::string tmp;
        if (iss >> tmp >> tmp >> numVertices >> numEdges) {
          parallel_assign(adjArray, numVertices + 1, std::size_t(0));
          parallel_assign(toVertex, numEdges, Vertex(0));
          parallel_assign(weight, numEdges, Weight(0));
          edges.reserve(numEdges);
        }
      } else if (line[0] == 'a') {
        std::istringstream iss(line);
        char a;
        Vertex u, v;
        int w;
        if (iss >> a >> u >> v >> w) {
          edges.emplace_back(u - 1, v - 1, Weight(w));
        }
      }
    }

    file.close();
    std::sort(edges.begin(), edges.end());

    for (const auto &[u, v, w] : edges) {
      ++adjArray[u + 1];
    }

    for (std::size_t i = 1; i < adjArray.size(); ++i) {
      adjArray[i] += adjArray[i - 1];
    }

    adjArray.back() = edges.size();

    toVertex.resize(edges.size());
    weight.resize(edges.size());
    std::vector<std::size_t> offset = adjArray;

    for (const auto &[u, v, w] : edges) {
      toVertex[offset[u]] = v;
      weight[offset[u]++] = w;
    }
  }

  Graph reverseGraph() const {
    StatusLog log("Reversing Graph");

    Graph reversed;
    reversed.adjArray = adjArray;
    reversed.toVertex = toVertex;
    reversed.weight = weight;
    reversed.flip();
    return reversed;
  }

  void flip() {
    std::vector<std::size_t> flippedAdjArray(numVertices() + 1, 0);
    std::vector<Vertex> flippedToVertex(numEdges(), noVertex);
    std::vector<Weight> flippedWeight(numEdges(), Weight(0));

    for (Vertex fromV(0); fromV < numVertices(); ++fromV) {
      for (std::size_t i = adjArray[fromV]; i < adjArray[fromV + 1]; ++i) {
        flippedAdjArray[toVertex[i] + 1]++;
      }
    }

    for (Vertex v = 1; v <= numVertices(); ++v) {
      flippedAdjArray[v] += flippedAdjArray[v - 1];
    }

    std::vector<std::size_t> offset = flippedAdjArray;

    for (Vertex fromV(0); fromV < numVertices(); ++fromV) {
      for (std::size_t i = adjArray[fromV]; i < adjArray[fromV + 1]; ++i) {
        Vertex toV = toVertex[i];
        Weight w = weight[i];
        flippedToVertex[offset[toV]] = fromV;
        flippedWeight[offset[toV]++] = w;
      }
    }

    adjArray = std::move(flippedAdjArray);
    toVertex = std::move(flippedToVertex);
    weight = std::move(flippedWeight);
  }

  void showStats() const {
    if (numVertices() == 0) {
      std::cout << "Graph is empty.\n";
      return;
    }

    std::size_t minDegree = std::numeric_limits<std::size_t>::max();
    std::size_t maxDegree = 0;
    std::size_t totalDegree = 0;

    for (Vertex v = 0; v < numVertices(); ++v) {
      std::size_t deg = degree(v);
      minDegree = std::min(minDegree, deg);
      maxDegree = std::max(maxDegree, deg);
      totalDegree += deg;
    }

    double avgDegree = static_cast<double>(totalDegree) / numVertices();

    std::cout << "Graph Statistics:\n";
    std::cout << "  Number of vertices: " << numVertices() << "\n";
    std::cout << "  Number of edges:    " << numEdges() << "\n";
    std::cout << "  Min degree:         " << minDegree << "\n";
    std::cout << "  Max degree:         " << maxDegree << "\n";
    std::cout << "  Average degree:     " << avgDegree << "\n";
  }
};

struct Tree {
  std::vector<Vertex> parent;

  Tree(const std::size_t numVertices = 0) : parent(numVertices, noVertex){};
  Tree(const Tree &) = default;
  Tree(Tree &&) = default;

  void resize(const std::size_t numVertices) {
    parent.assign(numVertices, noVertex);
  }

  bool isValid(const Vertex v) const {
    return v < parent.size() && v != noVertex;
  };

  void setParent(const Vertex v, const Vertex par) {
    assert(isValid(v));
    assert(isValid(par));

    parent[v] = par;
  }
};
