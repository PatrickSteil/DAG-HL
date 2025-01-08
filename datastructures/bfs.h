/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

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
      : graph(graph), q(graph.numVertices()), seen(graph.numVertices()),
        statsCollecter(BFS_METRICS_NAMES){};

  void showStats() { statsCollecter.printStats(); }

  void reset(const std::size_t numVertices) {
    q.reset();
    q.resize(numVertices);
    seen.reset();
    seen.resize(numVertices);

    statsCollecter.clear();
  }

  template <typename FUNC> void doForAllVerticesInQ(FUNC &&func) {
    for (std::size_t i = 0; i < q.read; ++i) {
      const Vertex u = q.data[i];
      func(u);
    }
  }

  // make sure you do thread safe stuff in the function
  template <typename FUNC> void doForAllVerticesInQInParallel(FUNC &&func) {
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

      if (onPop(u))
        continue;

      for (std::size_t i = graph.beginEdge(u); i < graph.endEdge(u); ++i) {
        /* statsCollecter.count(NUM_RELAXED_EDGES); */
        const Vertex w = graph.toVertex[i];

        if (!seen.firstOccur(w))
          continue;

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

struct ParallelBFS {
  const Graph &graph;
  FixedSizedQueueThreadSafe<Vertex> q;
  GenerationCheckerThreadSafe<> seen;

  ParallelBFS(const Graph &graph)
      : graph(graph), q(graph.numVertices()), seen(graph.numVertices()) {}

  void reset(const std::size_t numVertices) {
    q.reset();
    q.resize(numVertices);
    seen.reset();
    seen.resize(numVertices);
  }

  template <typename ON_POP = decltype([](const Vertex) { return false; }),
            typename ON_RELAX = decltype([](const Vertex) { return false; })>
  void run(const Vertex root, ON_POP &&onPop = noOp, ON_RELAX &&onRelax = noOp,
           std::size_t numThreads = 4) {
    q.reset();
    seen.reset();

    q.push(root);
    seen.mark(root);

    alignas(64) std::atomic_size_t threads_working_{numThreads};
    alignas(64) std::atomic_size_t threads_stealing_{0};

    // Function to process nodes.
    auto processNode = [&](Vertex u) {
      if (onPop(u))
        return;

      for (std::size_t i = graph.beginEdge(u); i < graph.endEdge(u); ++i) {
        const Vertex w = graph.toVertex[i];

        if (!seen.firstOccur(w))
          continue;

        if (onRelax(w))
          continue;

        q.push(w);
      }
    };

    // Function to find work either from the thread's local queue or through
    // stealing.
    const auto findWork = [&](auto &&findWork,
                              std::size_t thread_id) -> std::optional<Vertex> {
      // Try to find work locally.
      Vertex u = q.pop();
      if (u != noVertex) {
        return u;
      }

      // Wait for others to run out of work.
      for (auto currently_working =
               threads_working_.fetch_sub(1, std::memory_order_relaxed) - 1;
           currently_working > 0; currently_working = threads_working_.load(
                                      std::memory_order_relaxed)) {
        u = q.pop();
        if (u != noVertex) {
          threads_working_.fetch_add(1, std::memory_order_relaxed);
          return u;
        }
      }

      // Do a final check for work.
      u = q.pop();
      if (u != noVertex) {
        threads_working_.fetch_add(1, std::memory_order_relaxed);
        return u;
      }

      // Mark as done if no work is left.
      if (threads_stealing_.fetch_sub(1, std::memory_order_relaxed) == 1) {
        return std::nullopt;
      }

      // Wait for other threads to finish their work or find more tasks.
      for (;;) {
        if (threads_working_.load(std::memory_order_relaxed) > 0) {
          // Someone else found work.
          threads_stealing_.fetch_add(1, std::memory_order_relaxed);
          threads_working_.fetch_add(1, std::memory_order_relaxed);
          return findWork(findWork, thread_id);
        } else if (threads_stealing_.load(std::memory_order_relaxed) == 0) {
          // No one else is working.
          return std::nullopt;
        }
      }
    };

    std::atomic_size_t thread_id(0);

    // Worker function for each thread.
    const auto work = [&]() {
      const auto my_id = thread_id.fetch_add(1, std::memory_order_relaxed);
      for (;;) {
        if (auto u = findWork(findWork, my_id)) {
          processNode(*u); // Process the node if found.
        } else {
          break; // No work found, break the loop.
        }
      }
    };

    std::vector<std::thread> threads;
    for (std::size_t i = 0; i < numThreads; ++i) {
      threads.emplace_back(work);
    }

    for (auto &t : threads) {
      t.join();
    }
  }

  template <typename FUNC> void doForAllVerticesInQ(FUNC &&func) {
    // after the queue is done, read is +1 compared to write
    auto index = q.read.load();

    index = std::min(index, q.write.load());

    assert(index < q.data.size());

    for (std::size_t i = 0; i < index; ++i) {
      const Vertex u = q.data[i];
      func(u);
    }
  }
};

}; // namespace bfs
