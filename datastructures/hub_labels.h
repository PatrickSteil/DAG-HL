#include <algorithm>
#include <array>
#include <numeric>
#include <vector>

#include "priority_queue.h"
#include "types.h"

enum DIRECTION : bool { FWD, BWD };

struct alignas(64) Label {
  Label(){};

  std::vector<Vertex> nodes;

  Vertex &operator[](std::size_t i) { return nodes[i]; }
  const Vertex &operator[](std::size_t i) const { return nodes[i]; }

  void reserve(const std::size_t size) { nodes.reserve(size); };
  std::size_t size() const { return nodes.size(); };
  void add(const Vertex hub) { nodes.push_back(hub); };
  bool contains(const Vertex hub) {
    return std::find(nodes.begin(), nodes.end(), hub) != nodes.end();
  };

  void applyPermutation(const std::vector<Vertex> &permutation) {
    for (auto &hub : nodes) {
      hub = permutation[hub];
    }

    std::sort(nodes.begin(), nodes.end());
  }

  void setDeltaRepresentation() {
    if (nodes.empty()) return;

    std::vector<Vertex> new_nodes;
    new_nodes.reserve(nodes.size());

    new_nodes.push_back(nodes.front());

    auto prevHub = nodes.front();
    for (std::size_t i = 1; i < nodes.size(); ++i) {
      new_nodes.push_back(nodes[i] - prevHub - 1);
      prevHub = nodes[i];
    }

    nodes = std::move(new_nodes);
  }
};

std::vector<Vertex> computePermutation(
    const std::array<std::vector<Label>, 2> &labels) {
  const std::size_t numVertices = labels[0].size();
  std::vector<Vertex> result(numVertices);
  std::iota(result.begin(), result.end(), 0);

  std::vector<std::uint32_t> freq(numVertices, 0);

  for (DIRECTION dir : {FWD, BWD}) {
    for (auto &lab : labels[dir]) {
      for (const auto h : lab.nodes) {
        freq[h]++;
      }
    }
  }

  PriorityQueue<std::greater<std::uint32_t>> q;
  q.buildFrom(freq);

  Vertex id(0);
  while (!q.empty()) {
    auto [occur, hub] = q.pop();
    assert(hub != noVertex);

    result[hub] = id;
    ++id;
  }

  return result;
}
