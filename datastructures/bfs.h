/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <omp.h>

#include "../external/statistics_collecter.h"
#include "bfs_tools.h"
#include "graph.h"

namespace bfs {

auto noOp = [](const Vertex /* v */) { return false; };

struct BFS {
  const Graph &graph;
  FixedSizedQueue<Vertex> q;
  GenerationChecker<> seen;

  BFS(const Graph &graph)
      : graph(graph), q(graph.numVertices()), seen(graph.numVertices()) {}

  void reset(const std::size_t numVertices) {
    q.reset();
    q.resize(numVertices);
    seen.reset();
    seen.resize(numVertices);
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
            typename ON_RELAX = decltype([](const Vertex, const Vertex) {
              return false;
            })>
  void run(const Vertex root, ON_POP &&onPop, ON_RELAX &&onRelax) {
    q.reset();
    seen.reset();

    q.push(root);
    seen.mark(root);

    while (!q.isEmpty()) {
      const Vertex u = q.pop();

      if (onPop(u)) continue;

      for (std::size_t i = graph.beginEdge(u), end = graph.endEdge(u); i < end;
           ++i) {
        if (i + 4 < end) {
          PREFETCH(&graph.toVertex[i + 4]);
        }
        const Vertex w = graph.toVertex[i];

        if (seen.isMarked(w)) continue;
        seen.mark(w);

        if (onRelax(u, w)) {
          continue;
        }

        q.push(w);
      }
    }
  }
};

};  // namespace bfs
