#include <algorithm>
#include <vector>

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
