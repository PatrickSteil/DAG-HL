/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

// TODO WIP

#pragma once

#include <vector>

#include "graph.h"
#include "priority_queue.h"

namespace dijkstra {
auto noOp = [](const Vertex /* v */) { return false; };

struct Dijkstra {
  const Graph &graph;
  PriorityQueue<> q;
  GenerationChecker<> seen;
  std::vector<std::uint32_t> dist;

  Dijkstra(const Graph &graph)
      : graph(graph),
        q(graph.numVertices()),
        seen(graph.numVertices()),
        dist(graph.numVertices()) {}

  template <typename ON_POP = decltype([](const Vertex, const std::uint32_t) {
              return false;
            }),
            typename ON_RELAX = decltype([](const Vertex, const Vertex,
                                            const std::uint32_t) {
              return false;
            })>
  void run(const Vertex root, ON_POP &&onPop, ON_RELAX &&onRelax) {
    q.reset();
    seen.reset();

    q.push(root);
    seen.mark(root);
    dist[root] = 0;

    while (!q.isEmpty()) {
      const Vertex u = q.pop();
      const std::uint32_t d = dist[u];

      if (onPop(u, d)) continue;

      for (std::size_t i = graph.beginEdge(u), end = graph.endEdge(u); i < end;
           ++i) {
        const Vertex w = graph.toVertex[i];
        const std::uint32_t weight = 1;  // TODO right now

        auto newDist = d + weight;

        bool firstTime = seen.isMarked(w);
        dist[w] =
            firstTime ? std::numeric_limits<std::uint32_t>::max() : dist[w];

        if (newDist >= dist[w]) continue;
        dist[w] = newDist;

        if (onRelax(u, w, dist[w])) {
          continue;
        }

        q.push(w, dist[w]);
      }
    }
  }
};

};  // namespace dijkstra
