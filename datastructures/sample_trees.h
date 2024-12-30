#pragma once

#include <array>
#include <cassert>
#include <functional>
#include <queue>
#include <random>

#include "bfs.h"
#include "graph.h"
#include "priority_queue.h"
#include "types.h"
#include "utils.h"

// TODO
// remove subtrees

struct TreeIndex {
  Vertex vertex;
  uint16_t treeId;

  TreeIndex(Vertex v, uint16_t tree) : vertex(v), treeId(tree){};

  void clear() {
    vertex = noVertex;
    treeId = 0;
  }
};

struct PQ_Value {
  Vertex vertex;
  std::size_t value;

  PQ_Value(Vertex vertex, std::size_t value) : vertex(vertex), value(value){};
};

struct ComparatorPQ {
  bool operator()(const PQ_Value &a, const PQ_Value &b) {
    return a.value < b.value;
  }
};

template <int K = 16>
struct TreeSampler {
  std::array<const Graph *, 2> graphs;
  std::array<bfs::BFS, 2> bfs;
  std::array<Tree, 2 * K> trees;
  std::array<std::vector<std::size_t>, 2> numberDescendants;
  std::vector<PQ_Value> totalNumberOfDescendants;
  std::array<std::vector<std::vector<TreeIndex>>, 2> treeIndices;
  std::vector<Vertex> topoOrder;
  std::priority_queue<PQ_Value, std::vector<PQ_Value>, ComparatorPQ> pq;
  Drawer<Vertex> drawer;
  std::size_t totalNumberOfEdges;
  std::size_t totalNumberOfVertices;

  TreeSampler(const Graph &fwdGraph, const Graph &bwdGraph,
              std::vector<Vertex> &topoOrder)
      : graphs{&fwdGraph, &bwdGraph},
        bfs{bfs::BFS(*graphs[FWD]), bfs::BFS(*graphs[BWD])},
        numberDescendants{
            std::vector<std::size_t>(fwdGraph.numVertices() + 1, 0),
            std::vector<std::size_t>(fwdGraph.numVertices() + 1, 0)},
        totalNumberOfDescendants(fwdGraph.numVertices(), PQ_Value(0, 0)),
        treeIndices{
            std::vector<std::vector<TreeIndex>>(fwdGraph.numVertices()),
            std::vector<std::vector<TreeIndex>>(fwdGraph.numVertices())},
        topoOrder(topoOrder),
        pq(),
        drawer(fwdGraph.numVertices()) {
    reset();
    std::srand(42);
  };

  void reset() {
    for (auto &tree : trees) {
      tree.resize(graphs[FWD]->numVertices());
    }
    std::vector<Vertex> allVertices(graphs[FWD]->numVertices());
    std::iota(allVertices.begin(), allVertices.end(), 0);
    drawer.init(allVertices);

    for (Vertex v = 0; v < graphs[FWD]->numVertices(); ++v) {
      for (auto &desc : numberDescendants) {
        desc[v] = 0;
      }

      totalNumberOfDescendants[v] = PQ_Value(v, 0);

      for (auto &indexes : treeIndices[FWD][v]) {
        indexes.clear();
      }

      for (auto &indexes : treeIndices[BWD][v]) {
        indexes.clear();
      }
    }

    totalNumberOfEdges = 0;
    totalNumberOfVertices = 0;

    bfs[FWD].reset(graphs[FWD]->numVertices());
    bfs[BWD].reset(graphs[FWD]->numVertices());

    drawer.reset();
  }

  /* void run(std::size_t numberOfEdgesTouchedByLabelConstruction) { */
  /*   while (totalNumberOfEdges < numberOfEdgesTouchedByLabelConstruction && */
  /*          totalNumberOfVertices < 10 * K * graphs[FWD]->numVertices()) { */
  /*   } */
  /* } */

  template <DIRECTION dir = FWD>
  auto &getChildrenInTree(const Vertex v) {
    return treeIndices[dir][v];
  }

  void removeHub(const Vertex hub) { drawer.remove(hub); }

  void sampleTrees() {
    totalNumberOfVertices = 0;
    totalNumberOfEdges = 0;
    int size = std::min(static_cast<int>(drawer.size()), K);

    std::vector<Vertex> samples(size, noVertex);
    fillWithRandomVertices(samples);

    for (std::size_t i = 0; i < static_cast<std::size_t>(size); ++i) {
      growTree<FWD>(i, samples);
      growTree<BWD>(i, samples);
    }

    for (std::size_t t = 0; t < static_cast<std::size_t>(size); ++t) {
      compute_children_of_tree<FWD>(2 * t + FWD);
      compute_children_of_tree<BWD>(2 * t + BWD);
    }

    for (Vertex v = 0; v < graphs[FWD]->numVertices(); ++v) {
      totalNumberOfDescendants[v] =
          PQ_Value(v, numberDescendants[FWD][v] + numberDescendants[BWD][v]);
    }

    pq = std::priority_queue<PQ_Value, std::vector<PQ_Value>, ComparatorPQ>(
        totalNumberOfDescendants.begin(), totalNumberOfDescendants.end());
  }

  template <DIRECTION dir = FWD>
  void compute_children_of_tree(std::size_t treeIndex) {
    assert(treeIndex < trees.size());

    int start = (dir == FWD ? 0 : static_cast<int>(topoOrder.size() - 1));
    int end = (dir == FWD ? static_cast<int>(topoOrder.size()) : -1);

    for (int i = start; i != end; i += (dir == FWD ? 1 : -1)) {
      assert(static_cast<std::size_t>(i) < topoOrder.size());
      const Vertex v = topoOrder[i];

      assert(v < trees[treeIndex].size());
      const Vertex parent = trees[treeIndex][v];

      numberDescendants[dir][std::min(topoOrder.size(),
                                      static_cast<std::size_t>(parent))] +=
          (1 + numberDescendants[dir][v]);
    }
  }

  template <DIRECTION dir = FWD>
  void growTree(const std::size_t i, const std::vector<Vertex> &samples) {
    assert(i < samples.size());
    auto &tree = trees[2 * i + dir];

    tree.resize(graphs[dir]->numVertices());

    const Vertex source = samples[i];
    assert(source != noVertex);
    tree[source] = source;

    bfs[dir].run(
        source,
        [&](const Vertex /* u */) {
          totalNumberOfVertices++;
          return false;
        },
        [&](const Vertex u, const Vertex w) {
          tree[w] = u;
          totalNumberOfEdges++;
          treeIndices[dir][u].emplace_back(w,
                                           static_cast<uint16_t>(2 * i + dir));
          return false;
        });
  }

  void fillWithRandomVertices(std::vector<Vertex> &samples) {
    assert(!drawer.isEmpty());
    std::generate(samples.begin(), samples.end(),
                  [&]() { return drawer.pickRandom(); });
  }
};
