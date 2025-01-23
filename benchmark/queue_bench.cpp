#include <benchmark/benchmark.h>

#include <mutex>
#include <thread>
#include <vector>

#include "../datastructures/bfs_tools.h"

template <typename T>
using MutexQueue = bfs::FixedSizedQueueThreadSafe<T>;

template <typename T>
using SpinlockQueue = bfs::FixedSizedQueueThreadSafeSpinlock<T>;

template <typename Queue>
static void BenchmarkQueue(benchmark::State& state) {
  std::size_t queue_size = state.range(0);
  std::size_t num_threads = state.range(1);

  auto producer = [](Queue& queue, std::size_t max_pushes) {
    for (std::size_t i = 0; i < max_pushes; ++i) {
      queue.push(i);
    }
  };

  auto consumer = [](Queue& queue, std::size_t max_pops) {
    for (std::size_t i = 0; i < max_pops; ++i) {
      while (queue.pop() == static_cast<std::size_t>(-1)) {
        std::this_thread::yield();
      }
    }
  };

  for (auto _ : state) {
    Queue queue(queue_size);

    std::size_t pushes_per_thread = queue_size / (num_threads / 2);
    std::size_t pops_per_thread = pushes_per_thread;

    std::vector<std::thread> threads;

    benchmark::DoNotOptimize(queue);

    for (std::size_t i = 0; i < num_threads / 2; ++i) {
      threads.emplace_back(producer, std::ref(queue), pushes_per_thread);
    }

    for (std::size_t i = 0; i < num_threads / 2; ++i) {
      threads.emplace_back(consumer, std::ref(queue), pops_per_thread);
    }

    for (auto& thread : threads) {
      thread.join();
    }
  }
}

// Register benchmarks explicitly
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 10, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 10, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 10, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 10, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 11, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 11, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 11, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 11, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 12, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 12, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 12, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 12, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 13, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 13, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 13, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 13, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 14, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 14, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 14, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 14, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 15, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 15, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 15, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, MutexQueue<std::size_t>)->Args({1 << 15, 16});

BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 10, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 10, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 10, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 10, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 11, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 11, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 11, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 11, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 12, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 12, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 12, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 12, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 13, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 13, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 13, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 13, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 14, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 14, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 14, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 14, 16});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 15, 2});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 15, 4});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 15, 8});
BENCHMARK_TEMPLATE(BenchmarkQueue, SpinlockQueue<std::size_t>)->Args({1 << 15, 16});

BENCHMARK_MAIN();
