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

struct BFSParallelFrontier {
  explicit BFSParallelFrontier(const Graph &graph, const int numThreads)
      : graph(graph),
        seen(graph.numVertices()),
        q(graph.numVertices()),
        local_cache(numThreads),
        read(0),
        write(0) {}

  void reset(const std::size_t numVertices) {
    q.resize(numVertices);
    read.store(0);
    write.store(0);

    seen.reset();
    seen.resize(numVertices);
  }

  void addToQueue(const Vertex v) { q[read.fetch_add(1)] = v; }

  template <typename ON_POP = decltype([](const Vertex) { return false; }),
            typename ON_RELAX = decltype([](const Vertex, const Vertex) {
              return false;
            })>
  void run(const Vertex root, ON_POP &&onPop, ON_RELAX &&onRelax) {
    read.store(0);
    write.store(0);
    seen.reset();

    std::size_t pos = write.fetch_add(1, std::memory_order_release);
    assert(pos < q.size());
    q[pos] = root;
    seen.mark(root);

    while (read.load(std::memory_order_acquire) <
           write.load(std::memory_order_acquire)) {
      processLevel(std::forward<ON_POP>(onPop),
                   std::forward<ON_RELAX>(onRelax));
    }

    assert(read.load() == write.load());
  }

  template <typename ON_POP, typename ON_RELAX>
  void processLevel(ON_POP &&onPop, ON_RELAX &&onRelax) {
    std::size_t left = read.load(std::memory_order_acquire);
    std::size_t right = write.load(std::memory_order_acquire);

    assert(left <= right);
    // if right == q.size(), then we already added all the vertices to the queue
    assert(right <= q.size());

#pragma omp parallel for schedule(dynamic, 4)
    for (std::size_t i = left; i < right; ++i) {
      const Vertex u = q[i];

      int tId = omp_get_thread_num();
      auto &myCache = local_cache[tId];

      if (onPop(u)) continue;

      for (std::size_t i = graph.beginEdge(u), end = graph.endEdge(u); i < end;
           ++i) {
        const Vertex w = graph.toVertex[i];

        if (seen.isMarked(w)) continue;
        seen.mark(w);

        if (onRelax(u, w)) {
          continue;
        }

        myCache.push_back(w);
      }
      if (myCache.empty()) continue;
      std::size_t pos =
          write.fetch_add(myCache.size(), std::memory_order_release);
      assert(pos + myCache.size() <= q.size());
      std::copy(myCache.begin(), myCache.end(), q.begin() + pos);
      myCache.clear();
    }
    read.store(right, std::memory_order_release);
  }

  const Graph &graph;
  GenerationCheckerThreadSafe<> seen;
  std::vector<Vertex> q;

  std::vector<std::vector<Vertex>> local_cache;
  std::atomic_size_t read;
  std::atomic_size_t write;
};

};  // namespace bfs
