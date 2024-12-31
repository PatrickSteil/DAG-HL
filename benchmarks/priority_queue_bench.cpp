#include "../datastructures/priority_queue.h"

#include <benchmark/benchmark.h>
#include <random>
#include <vector>

std::vector<uint32_t> generateRandomValues(size_t size, uint32_t min = 0,
                                           uint32_t max = 1000000) {
  std::mt19937 rng(42); // Fixed seed for reproducibility.
  std::uniform_int_distribution<uint32_t> dist(min, max);

  std::vector<uint32_t> values(size);
  for (auto &value : values) {
    value = dist(rng);
  }
  return values;
}

static void BM_PriorityQueuePush(benchmark::State &state) {
  size_t size = state.range(0);
  PriorityQueue<> pq(size);
  auto values = generateRandomValues(size);

  for (auto _ : state) {
    for (size_t i = 0; i < size; ++i) {
      pq.push(i, values[i]);
    }
  }
}

static void BM_PriorityQueuePop(benchmark::State &state) {
  size_t size = state.range(0);
  PriorityQueue<> pq(size);
  auto values = generateRandomValues(size);

  for (size_t i = 0; i < size; ++i) {
    pq.push(i, values[i]);
  }

  for (auto _ : state) {
    while (!pq.empty()) {
      pq.pop();
    }
  }
}

static void BM_PriorityQueueBuildFrom(benchmark::State &state) {
  size_t size = state.range(0);
  auto values = generateRandomValues(size);

  for (auto _ : state) {
    PriorityQueue<> pq(size);
    pq.buildFrom(values);
  }
}

BENCHMARK(BM_PriorityQueuePush)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_PriorityQueuePop)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_PriorityQueueBuildFrom)->Range(1 << 10, 1 << 20);

BENCHMARK_MAIN();
