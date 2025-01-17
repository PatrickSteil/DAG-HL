/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <omp.h>

#include <array>
#include <unordered_map>
#include <vector>

#include "graph.h"
#include "types.h"
#include "utils.h"

/*
 * Supports both `std::vector` (for dense graphs) and `std::unordered_map` (for
 * sparse graphs) to store descendant counts. Provides efficient operations for
 * adding/removing edges, computing descendants, and iterating over vertices.
 * Note that this datastructure was implemented to work efficiently with DAGs,
 * "Undefined behaviour" if the underlying graph (from which we sample the FWD
 * and BWD tree) is not a DAG.
 */
template <typename STORAGE>
class EdgeTree {
 public:
  // Determine storage type (vector or unordered_map)
  static constexpr bool isVectorStorage =
      std::is_same_v<STORAGE, std::vector<Index>>;
  static constexpr bool isMapStorage =
      std::is_same_v<STORAGE, std::unordered_map<Vertex, Index>>;

  static_assert(isVectorStorage || isMapStorage);

  // Edge storage for both directions (FWD and BWD)
  std::array<std::vector<Edge>, 2> edges;

  // Stores the number of descendants per vertex
  STORAGE descendants;

  // Root of the tree
  Vertex root;

  // Constructor - initializes storage if using a vector
  explicit EdgeTree(std::size_t numVertices) : descendants(numVertices, 0) {}

  EdgeTree(EdgeTree&& other) = default;
  EdgeTree& operator=(EdgeTree&& other) = default;

  EdgeTree(const EdgeTree&) = default;
  EdgeTree& operator=(const EdgeTree&) = default;

  // Returns the number of vertices covered by both directions (assuming both
  // FWD and BWD tree are connected)
  std::size_t numVertices() const {
    return edges[FWD].size() + edges[BWD].size() + 1;
  }

  // Returns the capacity, i.e., if vector storage is chosen, how many vertices
  // are present
  std::size_t capacity() const { return descendants.size(); }

  // Returns the number of edges in both directions
  std::size_t numEdges() const { return edges[FWD].size() + edges[BWD].size(); }

  // Get the root vertex
  Vertex getRoot() const { return root; }

  // Clears the tree structure
  void clear() {
    edges[FWD].clear();
    edges[BWD].clear();
    std::fill(descendants.begin(), descendants.end(), 0);
  }

  // Resets the tree, setting a new root
  void reset(const Vertex newRoot = noVertex) {
    clear();
    root = newRoot;
  }

  // Updates the root vertex
  void setRoot(const Vertex newRoot) { root = newRoot; }

  // Adds an edge in the given direction
  void addEdge(Vertex from, Vertex to, const DIRECTION dir) {
    if constexpr (isVectorStorage) {
      assert(from < descendants.size());
      assert(to < descendants.size());
    }
    edges[dir].emplace_back(from, to);
  }

  // Reserves space for edges to reduce reallocations
  void reserveEdges(std::size_t capacity) {
    edges[FWD].reserve(capacity);
    edges[BWD].reserve(capacity);
  }

  // Computes the number of descendants for each node
  void computeDescendants() {
    auto runForDir = [&](const DIRECTION dir) -> void {
      for (std::size_t i = edges[dir].size(); i-- > 0;) {
        const auto& edge = edges[dir][i];
        getOrDefault(edge.from) += 1 + getOrDefault(edge.to);
      }
    };
    runForDir(FWD);
    runForDir(BWD);
  }

  // Removes a subtree rooted at vertex 'v'
  void removeSubtree(Vertex v) {
    // If removing the root, clear everything
    if (v == getRoot()) {
      edges[FWD].clear();
      edges[BWD].clear();
      if constexpr (isVectorStorage) {
        std::fill(descendants.begin(), descendants.end(), 0);
      } else {
        descendants.clear();
      }
      return;
    }

    auto runForDir = [&](const DIRECTION dir) -> void {
      for (std::size_t i = 0; i < edges[dir].size(); ++i) {
        auto& edge = edges[dir][i];

        bool isMatch = (edge.to == v);
        bool remove = (isMatch || getOrDefault(edge.from) == noIndex);

        getOrDefault(edge.from) -= isMatch * (1 + getOrDefault(edge.to));
        getOrDefault(edge.to) = (remove ? noIndex : getOrDefault(edge.to));
      }

      // Remove edges marked for deletion
      edges[dir].erase(std::remove_if(edges[dir].begin(), edges[dir].end(),
                                      [&](const auto& edge) {
                                        return getOrDefault(edge.from) ==
                                                   noIndex ||
                                               getOrDefault(edge.to) == noIndex;
                                      }),
                       edges[dir].end());
    };

    // Process both directions
    runForDir(FWD);
    runForDir(BWD);

    if constexpr (isVectorStorage) {
      for (std::size_t i = 0; i < descendants.size(); ++i) {
        auto& val = descendants[i];
        val = (val == noIndex) ? 0 : val;
      }
    } else {
      std::erase_if(descendants, [](const auto& item) {
        return item.second == noIndex || item.second == 0;
      });
    }
  }

  // Applies a function to all vertices and their descendant counts
  template <typename FUNC>
  void doForAllVerticesAndDescendants(const FUNC&& func) const {
    if constexpr (isVectorStorage) {
      for (std::size_t v = 0; v < descendants.size(); ++v) {
        func(v, descendants[v]);
      }
    } else {
      for (const auto& [v, count] : descendants) {
        func(v, count);
      }
    }
  }

  // Retrieves the descendant count for a vertex, or returns a default value
  Index& getOrDefault(Vertex vertex) {
    if constexpr (isVectorStorage) {
      assert(vertex < descendants.size());
      return descendants[vertex];
    } else {
      return descendants[vertex];
    }
  }
};

/**
 * The forest maintains multiple trees, supporting operations for adding new
 * trees, computing subtree sizes, and removing trees or subtrees dynamically.
 * It provides parallelized methods for efficient batch operations on all trees.
 */
struct Forest {
  std::vector<EdgeTree<std::vector<Index>>> trees;
  std::size_t numVertices;

  Forest(const std::size_t numVerticesNew)
      : trees(), numVertices(numVerticesNew) {
    trees.reserve(32);
  };

  Forest(Forest&& other) noexcept
      : trees(std::move(other.trees)), numVertices(other.numVertices) {}

  std::size_t newTree() {
    assert(numVertices > 0);
    trees.emplace_back(numVertices);
    return trees.size() - 1;
  }

  EdgeTree<std::vector<Index>>& getTree(const std::size_t i) {
    assert(i < trees.size());
    return trees[i];
  }

  std::size_t numberOfTrees() const { return trees.size(); }

  void computeSubtreeSizes(const int numThreads = 1) {
#pragma omp parallel for schedule(dynamic) num_threads(numThreads)
    for (std::size_t t = 0; t < trees.size(); ++t) {
      auto& tree = trees[t];
      tree.computeDescendants();
    }
  }

  void removeSubtreesAtVertex(const Vertex v, const int numThreads = 1) {
#pragma omp parallel for schedule(dynamic) num_threads(numThreads)
    for (std::size_t t = 0; t < trees.size(); ++t) {
      auto& tree = trees[t];
      tree.removeSubtree(v);
    }
  }

  void removeTreesWithNoEdges() {
    trees.erase(
        std::remove_if(trees.begin(), trees.end(),
                       [](const auto& tree) { return tree.numEdges() == 0; }),
        trees.end());
  }

  void clear() { trees.clear(); }

  void reset(std::size_t numVerticesNew) {
    numVertices = numVerticesNew;
    trees.clear();
  }

  void print() const {
    for (const auto& tree : trees) {
      std::cout << "Tree with V: " << tree.numVertices()
                << ", E: " << tree.numEdges() << std::endl;
      tree.doForAllVerticesAndDescendants([](const auto v, const auto val) {
        std::cout << v << ", " << val << std::endl;
      });
    }
  }
};
