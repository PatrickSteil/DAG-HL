#include <benchmark/benchmark.h>

#include <mutex>
#include <thread>
#include <vector>

#include "../datastructures/bfs_tools.h"

// Define Queue Types
template <typename T>
using MutexQueue = bfs::FixedSizedQueueThreadSafe<T>;

template <typename T>
using SpinlockQueue = bfs::FixedSizedQueueThreadSafeSpinlock<T>;

// Benchmark Function Template
template <typename Queue>
static void BenchmarkQueue(benchmark::State& state) {
  std::size_t queue_size = state.range(0);
  std::size_t num_threads = state.range(1);

  auto producer = [&](Queue& queue, std::size_t max_pushes) {
    for (std::size_t i = 0; i < max_pushes; ++i) {
      queue.push(i);
    }
  };

  auto consumer = [&](Queue& queue, std::size_t max_pops) {
    for (std::size_t i = 0; i < max_pops; ++i) {
      while (queue.pop() == static_cast<std::size_t>(-1)) {
        // Busy wait if the queue is empty
        std::this_thread::yield();
      }
    }
  };

  for (auto _ : state) {
    Queue queue(queue_size);

    std::size_t pushes_per_thread = queue_size / (num_threads / 2);
    std::size_t pops_per_thread = pushes_per_thread;

    std::vector<std::thread> threads;

    // Launch producer threads
    for (std::size_t i = 0; i < num_threads / 2; ++i) {
      threads.emplace_back(producer, std::ref(queue), pushes_per_thread);
    }

    // Launch consumer threads
    for (std::size_t i = 0; i < num_threads / 2; ++i) {
      threads.emplace_back(consumer, std::ref(queue), pops_per_thread);
    }

    // Wait for all threads to finish
    for (auto& thread : threads) {
      thread.join();
    }
  }

  state.SetItemsProcessed(queue_size * state.iterations());
}

// Benchmark Registration
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)
    ->Args({1000, 4})      // 100k operations, 4 threads
    ->Args({10000, 8})     // 1M operations, 8 threads
    ->Args({100000, 16});  // 1M operations, 16 threads

BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)
    ->Args({1000, 4})
    ->Args({10000, 8})
    ->Args({100000, 16});

// Benchmark Main Entry
BENCHMARK_MAIN();
