#pragma once

#include <immintrin.h>

#include <algorithm>
#include <array>
#include <bitset>
#include <cstdint>
#include <iostream>
#include <queue>
#include <thread>
#include <tuple>
#include <vector>

#include "graph.h"
#include "priority_queue.h"
#include "types.h"
#include "utils.h"

namespace SIMD_MEDIAN {
inline void siftDown(uint32_t *data, std::size_t i) {
  while (i < 8) {
    auto left = (i << 1) + 1;
    auto right = (i << 1) + 2;
    auto smallest = i;

    bool leftIsSmaller = (data[left] < data[right]);
    auto index = left;
    index = branchlessConditional(leftIsSmaller, left, right);
    bool update = (data[index] < data[smallest]);
    smallest = branchlessConditional(update, index, smallest);

    if (smallest == i)
      break;

    std::swap(data[i], data[smallest]);
    i = smallest;
  }
}

void buildHeap(uint32_t *data) {
#pragma GCC unroll(8)
  for (int i = static_cast<int>((8 / 2) - 1); i >= 0; --i) {
    siftDown(data, i);
  }
}

uint32_t pop(uint32_t *data, std::size_t length) {
  std::swap(data[0], data[--length]);
  siftDown(data, 0);
  return data[length];
}
} // namespace SIMD_MEDIAN

union alignas(64) ParentHolder {
  __m256i reg;
  uint32_t values[8];

  // Default constructor
  ParentHolder() : reg(_mm256_setzero_si256()) {}

  // Explicit copy constructor
  ParentHolder(const ParentHolder &other) {
    reg = other.reg; // Safe to copy the `__m256i` type directly
  }

  // Explicit assignment operator
  ParentHolder &operator=(const ParentHolder &other) {
    if (this != &other) {
      reg = other.reg;
    }
    return *this;
  }

  uint32_t &operator[](std::size_t idx) { return values[idx]; }

  const uint32_t &operator[](std::size_t idx) const { return values[idx]; }

  void print() {
    for (int i = 0; i < 8; ++i)
      std::cout << values[i] << ", ";
    std::cout << std::endl;
  }

  void fill(uint32_t val) {
    std::fill(std::begin(values), std::end(values), val);
  }

  std::size_t sum() const {
    std::size_t total = 0;
#pragma GCC unroll(8)
    for (int i = 0; i < 8; ++i) {
      total += values[i];
    }
    return total;
  }

  double average() const { return static_cast<double>(sum()) / 8; }

  uint32_t median() const {
    constexpr std::size_t heapSize = 16;
    uint32_t heap[heapSize];

    std::copy(values, values + 8, std::begin(heap));
    std::fill(heap + 8, heap + heapSize, INF);

    SIMD_MEDIAN::buildHeap(heap);

    SIMD_MEDIAN::pop(heap, 8);
    SIMD_MEDIAN::pop(heap, 7);
    SIMD_MEDIAN::pop(heap, 6);
    uint32_t fourth = SIMD_MEDIAN::pop(heap, 5);
    uint32_t fifth = SIMD_MEDIAN::pop(heap, 4);

    return (fourth + fifth) / 2;
  }

  ParentHolder operator+(const ParentHolder &other) {
    ParentHolder result;
    result.reg = _mm256_add_epi32(this->reg, other.reg);
    return result;
  }

  ParentHolder operator-(const ParentHolder &other) {
    ParentHolder result;
    result.reg = _mm256_sub_epi32(this->reg, other.reg);
    return result;
  }

  ParentHolder operator|(const ParentHolder &other) {
    ParentHolder result;
    result.reg = _mm256_or_si256(this->reg, other.reg);
    return result;
  }

  ParentHolder operator&(const ParentHolder &other) {
    ParentHolder result;
    result.reg = _mm256_and_si256(this->reg, other.reg);
    return result;
  }

  ParentHolder operator^(const ParentHolder &other) {
    ParentHolder result;
    result.reg = _mm256_xor_si256(this->reg, other.reg);
    return result;
  }
};

template <int K = 16> struct TreeSampler {
  std::array<const Graph *, 2> graphs;
  std::size_t numNodes;
  std::array<std::vector<ParentHolder>, 2> parents;
  std::array<std::vector<ParentHolder>, 2> descendants;
  std::array<std::vector<ParentHolder>, 2> currentDescendants;
  std::vector<Edge> edges;

  TreeSampler(const Graph &fwdGraph, const Graph &bwdGraph,
              std::vector<Vertex> &topoOrder)
      : graphs{&fwdGraph, &bwdGraph}, numNodes(fwdGraph.numVertices()),
        parents{std::vector<ParentHolder>(numNodes + 1),
                std::vector<ParentHolder>(numNodes + 1)},
        descendants{std::vector<ParentHolder>(numNodes + 1),
                    std::vector<ParentHolder>(numNodes + 1)},
        currentDescendants{std::vector<ParentHolder>(numNodes + 1),
                           std::vector<ParentHolder>(numNodes + 1)},
        edges() {
    resetDescendants();
    resetCurrentDescendants();
    edges.reserve(fwdGraph.numEdges());
    for (Vertex v = 0; v < fwdGraph.numVertices(); ++v) {
      for (std::size_t i = fwdGraph.beginEdge(v); i < fwdGraph.endEdge(v);
           ++i) {
        edges.emplace_back(v, fwdGraph.toVertex[i]);
      }
    }

    std::sort(edges.begin(), edges.end(),
              [&](const auto &left, const auto &right) {
                return std::tie(topoOrder[left.from], topoOrder[left.to]) <
                       std::tie(topoOrder[right.from], topoOrder[right.to]);
              });
  };

  void resetDescendants() {
    for (auto &vec : descendants) {
      for (auto &desc : vec) {
        desc.fill(0);
      }
    }
  }

  void resetCurrentDescendants() {
    for (auto &vec : currentDescendants) {
      for (auto &desc : vec) {
        desc.fill(0);
      }
    }
  }

  void computeParents(std::vector<uint32_t> &sources,
                      const std::vector<bool> &prune) {
    StatusLog log("[SWEEP] Compute Parents");
    const uint32_t numTrees = 8;
    const std::size_t originalSize = sources.size();
    sources.resize(numTrees, numNodes);

    __m256i empty = _mm256_set1_epi32(numNodes);

    auto fwdWork = [&]() {
      for (std::size_t i = 0; i <= numNodes; ++i) {
        parents[FWD][i].fill(numNodes);
      }

#pragma GCC unroll(4)
      for (std::size_t t = 0; t < originalSize; ++t) {
        parents[FWD][sources[t]][t] = sources[t];
      }

      for (auto it = edges.begin(); it != edges.end(); ++it) {
        uint32_t u = it->from, v = it->to;

        __m256i pruneMask = _mm256_set1_epi32(prune[v] ? 0 : -1);
        __m256i valid_from =
            _mm256_xor_si256(_mm256_cmpeq_epi32(parents[FWD][u].reg, empty),
                             _mm256_set1_epi32(-1));
        __m256i valid_to = _mm256_cmpeq_epi32(parents[FWD][v].reg, empty);

        __m256i mask =
            _mm256_and_si256(_mm256_and_si256(valid_from, valid_to), pruneMask);
        parents[FWD][v].reg =
            _mm256_blendv_epi8(parents[FWD][v].reg, _mm256_set1_epi32(u), mask);
      }
    };

    auto bwdWork = [&]() {
      for (std::size_t i = 0; i <= numNodes; ++i) {
        parents[BWD][i].fill(numNodes);
      }
#pragma GCC unroll(4)
      for (std::size_t t = 0; t < originalSize; ++t) {
        parents[BWD][sources[t]][t] = sources[t];
      }
      for (auto it = edges.rbegin(); it != edges.rend(); ++it) {
        uint32_t u = it->to, v = it->from;

        __m256i pruneMask = _mm256_set1_epi32(prune[v] ? 0 : -1);
        __m256i valid_from =
            _mm256_xor_si256(_mm256_cmpeq_epi32(parents[BWD][u].reg, empty),
                             _mm256_set1_epi32(-1));
        __m256i valid_to = _mm256_cmpeq_epi32(parents[BWD][v].reg, empty);

        __m256i mask =
            _mm256_and_si256(_mm256_and_si256(valid_from, valid_to), pruneMask);
        parents[BWD][v].reg =
            _mm256_blendv_epi8(parents[BWD][v].reg, _mm256_set1_epi32(u), mask);
      }
    };

    std::thread forwardThread(fwdWork);
    std::thread backwardThread(bwdWork);
    forwardThread.join();
    backwardThread.join();
  }

  void computeDescendants() {
    StatusLog log("[SWEEP] Compute Descendants");
    auto fwdWork = [&]() {
      for (std::size_t v = numNodes; v-- > 0;) {
#pragma GCC unroll(8)
        for (int t = 0; t < 8; ++t) {
          uint32_t parent = parents[FWD][v][t];
          uint32_t mask = (parent != v);
          currentDescendants[FWD][parent][t] +=
              mask * (1 + currentDescendants[FWD][v][t]);
        }

        descendants[FWD][v] = descendants[FWD][v] + currentDescendants[FWD][v];
        currentDescendants[FWD][v].fill(0);
      }
    };

    auto bwdWork = [&]() {
      for (std::size_t v = 0; v < numNodes; ++v) {
#pragma GCC unroll(8)
        for (int t = 0; t < 8; ++t) {
          uint32_t parent = parents[BWD][v][t];
          uint32_t mask = (parent != v);
          currentDescendants[BWD][parent][t] +=
              mask * (1 + currentDescendants[BWD][v][t]);
        }

        descendants[BWD][v] = descendants[BWD][v] + currentDescendants[BWD][v];
        currentDescendants[BWD][v].fill(0);
      }
    };

    std::thread forwardThread(fwdWork);
    std::thread backwardThread(bwdWork);
    forwardThread.join();
    backwardThread.join();
  }

  void removeSubtreeAt(const std::vector<uint32_t> &topoOrder,
                       const std::vector<std::size_t> &topoRank,
                       uint32_t root) {
    __m256i empty = _mm256_set1_epi32(topoOrder.size());
    __m256i rootMask = _mm256_set1_epi32(root);

    auto fwdWork = [&]() {
      __m256i removeMask = _mm256_set1_epi32(0);
      for (std::size_t i = topoRank[root]; i < topoOrder.size(); ++i) {
        uint32_t v = topoRank[i];

        removeMask = _mm256_or_si256(
            removeMask, _mm256_cmpeq_epi32(parents[FWD][v].reg, rootMask));

        parents[FWD][v].reg =
            _mm256_blendv_epi8(parents[FWD][v].reg, empty, removeMask);
      }
    };

    auto bwdWork = [&]() {
      __m256i removeMask = _mm256_set1_epi32(0);
      for (std::size_t i = topoRank[root]; i-- > 0;) {
        uint32_t v = topoRank[i];

        removeMask = _mm256_or_si256(
            removeMask, _mm256_cmpeq_epi32(parents[BWD][v].reg, rootMask));

        parents[BWD][v].reg =
            _mm256_blendv_epi8(parents[BWD][v].reg, empty, removeMask);
      }
    };

    std::thread forwardThread(fwdWork);
    std::thread backwardThread(bwdWork);

    forwardThread.join();
    backwardThread.join();
  }

  std::size_t totalDescendants(std::uint32_t v) {
    return (fwdDescendants(v) + bwdDescendants(v));
  }

  std::size_t fwdDescendants(std::uint32_t v) {
    return descendants[FWD][v].average();
  }

  std::size_t bwdDescendants(std::uint32_t v) {
    return descendants[BWD][v].average();
  }
};
