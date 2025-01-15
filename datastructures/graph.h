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

  Edge() = default;
  Edge(Vertex from, Vertex to) : from(from), to(to) {}

  auto operator<=>(const Edge &other) const = default;
};

struct Graph {
  std::vector<std::size_t> adjArray;
  std::vector<Vertex> toVertex;

  Graph() : adjArray(1), toVertex(){};

  Graph(const Graph &other)
      : adjArray(other.adjArray), toVertex(other.toVertex){};

  Graph(Graph &&other) noexcept
      : adjArray(std::move(other.adjArray)),
        toVertex(std::move(other.toVertex)) {}

  bool isValid(const Vertex v) const { return v < numVertices(); }

  std::size_t numVertices() const { return adjArray.size() - 1; }
  std::size_t numEdges() const { return toVertex.size(); }

  void print() const {
    /* std::locale::global(std::locale("de_DE.UTF-8")); */
    /* std::cout.imbue(std::locale()); */

    std::cout << "NumVertices: " << numVertices() << std::endl;
    std::cout << "NumEdges: " << numEdges() << std::endl;

    for (Vertex v = 0; v < numVertices(); ++v) {
      std::cout << "Edges from " << v << std::endl;

      for (std::size_t i = beginEdge(v); i < endEdge(v); ++i) {
        std::cout << toVertex[i] << " ";
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
  }

  void readFromEdgeList(const std::string &fileName) {
    StatusLog log("Reading graph from edgelist");
    clear();

    std::ifstream file(fileName);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open file: " + fileName);
    }

    std::vector<std::pair<Vertex, Vertex>> edges;
    Vertex maxVertex = 0;

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      Vertex u, v;
      if (!(iss >> u >> v)) {
        continue;
      }

      edges.emplace_back(u - 1, v - 1);
      maxVertex = std::max({maxVertex, u - 1, v - 1});
    }

    file.close();

    adjArray.resize(maxVertex + 2, 0);

    ips4o::parallel::sort(edges.begin(), edges.end(),
                          [](const auto &left, const auto &right) {
                            return std::tie(left.first, left.second) <
                                   std::tie(right.first, right.second);
                          });
    for (const auto &[u, v] : edges) {
      ++adjArray[u + 1];
    }

    for (std::size_t i = 1; i < adjArray.size(); ++i) {
      adjArray[i] += adjArray[i - 1];
    }

    toVertex.resize(edges.size());
    std::vector<std::size_t> offset = adjArray;

    for (const auto &[u, v] : edges) {
      toVertex[offset[u]++] = v;
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
    std::vector<std::pair<Vertex, Vertex>> edges;
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
          edges.reserve(numEdges);
        }
      } else if (line[0] == 'a') {
        std::istringstream iss(line);
        char a;
        Vertex u, v;
        if (iss >> a >> u >> v) {
          edges.emplace_back(u - 1, v - 1);
        }
      }
    }

    file.close();
    std::sort(edges.begin(), edges.end(),
              [](const auto &left, const auto &right) {
                return std::tie(left.first, left.second) <
                       std::tie(right.first, right.second);
              });

    for (const auto &[u, v] : edges) {
      ++adjArray[u + 1];
    }

    for (std::size_t i = 1; i < adjArray.size(); ++i) {
      adjArray[i] += adjArray[i - 1];
    }

    adjArray.back() = edges.size();

    toVertex.resize(edges.size());
    std::vector<std::size_t> offset = adjArray;

    for (const auto &[u, v] : edges) {
      toVertex[offset[u]++] = v;
    }
  }

  void readMetis(const std::string &fileName) {
    StatusLog log("Reading graph from metis");
    std::ifstream inputFile(fileName);
    if (!inputFile.is_open()) {
      throw std::runtime_error("Failed to open file: " + fileName);
    }

    clear();

    std::string line;

    if (!std::getline(inputFile, line)) {
      throw std::runtime_error("Invalid METIS file format: missing header");
    }

    std::istringstream headerStream(line);
    std::size_t numVertices, numEdges;
    if (!(headerStream >> numVertices >> numEdges)) {
      throw std::runtime_error("Invalid METIS file format: invalid header");
    }

    adjArray.assign(numVertices + 1, 0);
    std::vector<std::vector<Vertex>> adjacencyLists(numVertices);

    Vertex vertexId = 0;
    while (std::getline(inputFile, line)) {
      if (line.empty()) {
        continue;
      }

      std::istringstream lineStream(line);
      Vertex neighbor;
      while (lineStream >> neighbor) {
        if (neighbor < 1 || neighbor > numVertices) {
          throw std::runtime_error(
              "Invalid METIS file format: vertex index out of range");
        }
        adjacencyLists[vertexId].push_back(neighbor - 1);
      }
      ++vertexId;
    }

    if (vertexId != numVertices) {
      throw std::runtime_error(
          "Invalid METIS file format: vertex count mismatch");
    }

    std::size_t edgeIndex = 0;
    for (std::size_t v = 0; v < numVertices; ++v) {
      adjArray[v] = edgeIndex;
      toVertex.insert(toVertex.end(), adjacencyLists[v].begin(),
                      adjacencyLists[v].end());
      edgeIndex += adjacencyLists[v].size();
    }
    adjArray[numVertices] = edgeIndex;

    if (toVertex.size() != 2 * numEdges) {
      throw std::runtime_error(
          "Invalid METIS file format: edge count mismatch");
    }
  }

  void readSnap(const std::string &fileName) {
    StatusLog log("Reading graph from .snap format");
    clear();

    std::ifstream file(fileName);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open file: " + fileName);
    }

    std::vector<std::pair<Vertex, Vertex>> edges;
    Vertex maxVertex = 0;

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty() || line[0] == '#') {
        continue;
      }

      std::istringstream iss(line);
      Vertex u, v;
      if (!(iss >> u >> v)) {
        throw std::runtime_error("Invalid line format in .snap file: " + line);
      }

      u = (u < v ? u : v);
      v = (u < v ? v : u);

      if (u == v) [[unlikely]]
        continue;

      edges.emplace_back(u, v);
      maxVertex = std::max(maxVertex, v);
    }

    file.close();

    adjArray.resize(maxVertex + 2, 0);

    ips4o::parallel::sort(edges.begin(), edges.end(),
                          [](const auto &left, const auto &right) {
                            return std::tie(left.first, left.second) <
                                   std::tie(right.first, right.second);
                          });

    auto it = std::unique(edges.begin(), edges.end());
    edges.erase(it, edges.end());

    for (const auto &[u, v] : edges) {
      ++adjArray[u + 1];
    }

    for (std::size_t i = 1; i < adjArray.size(); ++i) {
      adjArray[i] += adjArray[i - 1];
    }

    adjArray.back() = edges.size();

    toVertex.resize(edges.size());
    std::vector<std::size_t> offset = adjArray;

    for (const auto &[u, v] : edges) {
      toVertex[offset[u]++] = v;
    }
  }

  Graph reverseGraph() const {
    StatusLog log("Reversing Graph");

    Graph reversed;
    reversed.adjArray = adjArray;
    reversed.toVertex = toVertex;
    reversed.flip();
    return reversed;
  }

  void flip() {
    std::vector<std::size_t> flippedAdjArray(numVertices() + 1, 0);
    std::vector<Vertex> flippedToVertex(numEdges(), noVertex);

    for (Vertex fromV(0); fromV < numVertices(); ++fromV) {
      for (std::size_t i = adjArray[fromV]; i < adjArray[fromV + 1]; ++i) {
        flippedAdjArray[toVertex[i] + 1]++;
      }
    }

    for (Vertex v = 1; v <= numVertices(); ++v) {
      flippedAdjArray[v] += flippedAdjArray[v - 1];
    }

    std::vector<std::atomic_size_t> offset(flippedAdjArray.size());

#pragma omp parallel for
    for (std::size_t i = 0; i < offset.size(); ++i) {
      offset[i].store(flippedAdjArray[i], std::memory_order_relaxed);
    }

#pragma omp parallel for schedule(static)
    for (Vertex fromV = 0; fromV < numVertices(); ++fromV) {
      for (std::size_t i = adjArray[fromV]; i < adjArray[fromV + 1]; ++i) {
        Vertex toV = toVertex[i];
        flippedToVertex[offset[toV].fetch_add(1)] = fromV;
      }
    }

    adjArray = std::move(flippedAdjArray);
    toVertex = std::move(flippedToVertex);
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
