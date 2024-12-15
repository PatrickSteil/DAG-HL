#pragma once

#include <omp.h>

#include "../external/statistics_collecter.h"
#include "bfs_tools.h"
#include "graph.h"

namespace bfs {

auto noOp = [](const Vertex /* v */) { return false; };

enum BFS_METRICS {
  NUM_PUSHED_VERTICES,
  NUM_POPPED_VERTICES,
  NUM_RELAXED_EDGES,
  NUM_PRUNED_VERTICES
};

static std::vector<std::string> BFS_METRICS_NAMES = {
    "\"# of pushed vertices:\"",
    "\"# of popped vertices:\"",
    "\"# of relaxed edges:\"",
    "\"# of pruned vertices:\"",
};

struct BFS {
  const Graph &graph;
  FixedSizedQueue<Vertex> q;
  GenerationChecker<> seen;

  StatisticsCollecter statsCollecter;

  BFS(const Graph &graph)
      : graph(graph),
        q(graph.numVertices()),
        seen(graph.numVertices()),
        statsCollecter(BFS_METRICS_NAMES){};

  void showStats() { statsCollecter.printStats(); }

  void reset(const std::size_t numVertices) {
    q.reset();
    q.resize(numVertices);
    seen.reset();
    seen.resize(numVertices);

    statsCollecter.clear();
  }

  template <typename FUNC>
  void doForAllVerticesInQ(FUNC &&func) {
    for (std::size_t i = 0; i < q.read; ++i) {
      const Vertex u = q.data[i];
      func(u);
    }
  }

  // make sure you do thread safe stuff in the function
  template <typename FUNC>
  void doForAllVerticesInQInParallel(FUNC &&func) {
#pragma omp parallel for schedule(dynamic, 32)
    for (std::size_t i = 0; i < q.read; ++i) {
      const Vertex u = q.data[i];
      func(u);
    }
  }

  template <typename ON_POP = decltype([](const Vertex) { return false; }),
            typename ON_RELAX = decltype([](const Vertex) { return false; })>
  void run(const Vertex root, ON_POP &&onPop, ON_RELAX &&onRelax) {
    q.reset();
    seen.reset();
    statsCollecter.newRound();

    /* statsCollecter.count(NUM_PUSHED_VERTICES); */
    q.push(root);
    seen.mark(root);

    while (!q.isEmpty()) {
      const Vertex u = q.pop();
      /* statsCollecter.count(NUM_POPPED_VERTICES); */

      if (onPop(u)) continue;

      for (std::size_t i = graph.beginEdge(u); i < graph.endEdge(u); ++i) {
        /* statsCollecter.count(NUM_RELAXED_EDGES); */
        const Vertex w = graph.toVertex[i];

        if (seen.isMarked(w)) continue;
        seen.mark(w);

        if (onRelax(w)) {
          /* statsCollecter.count(NUM_PRUNED_VERTICES); */
          continue;
        }

        /* statsCollecter.count(NUM_PUSHED_VERTICES); */
        q.push(w);
      }
    }
  }
};

};  // namespace bfs
