#include <benchmark/benchmark.h>

#include <random>
#include <vector>

#include "../datastructures/bit_vector.h"

constexpr size_t NUM_ELEMENTS = 1 << 20;

static void BM_VectorBool_RandomAccess(benchmark::State& state) {
  std::vector<bool> vec;
  vec.reserve(NUM_ELEMENTS);
  for (size_t i = 0; i < NUM_ELEMENTS; ++i) vec.push_back(i % 2);

  std::mt19937 gen(42);
  std::uniform_int_distribution<size_t> dist(0, NUM_ELEMENTS - 1);

  for (auto _ : state) {
    benchmark::DoNotOptimize(vec[dist(gen)]);
  }
}

static void BM_VectorBool_Iteration(benchmark::State& state) {
  std::vector<bool> vec(NUM_ELEMENTS, true);
  for (auto _ : state) {
    for (bool b : vec) benchmark::DoNotOptimize(b);
  }
}

static void BM_VectorBool_Reserve_PushBack(benchmark::State& state) {
  for (auto _ : state) {
    std::vector<bool> vec;
    vec.reserve(NUM_ELEMENTS);
    for (size_t i = 0; i < NUM_ELEMENTS; ++i) vec.push_back(i % 2);
    benchmark::ClobberMemory();
  }
}

static void BM_VectorBool_PushBack(benchmark::State& state) {
  for (auto _ : state) {
    std::vector<bool> vec;
    for (size_t i = 0; i < NUM_ELEMENTS; ++i) vec.push_back(i % 2);
    benchmark::ClobberMemory();
  }
}

template <typename T>
static void BM_BitVector_RandomAccess(benchmark::State& state) {
  BitVector<T> bv;
  bv.reserve(NUM_ELEMENTS);
  for (size_t i = 0; i < NUM_ELEMENTS; ++i) bv.push_back(i % 2);

  std::mt19937 gen(42);
  std::uniform_int_distribution<size_t> dist(0, NUM_ELEMENTS - 1);

  for (auto _ : state) {
    benchmark::DoNotOptimize(bv[dist(gen)]);
  }
}

template <typename T>
static void BM_BitVector_Iteration(benchmark::State& state) {
  BitVector<T> bv;
  for (size_t i = 0; i < NUM_ELEMENTS; ++i) bv.push_back(true);

  for (auto _ : state) {
    for (bool b : bv) benchmark::DoNotOptimize(b);
  }
}

template <typename T>
static void BM_BitVector_PushBack(benchmark::State& state) {
  for (auto _ : state) {
    BitVector<T> bv;
    for (size_t i = 0; i < NUM_ELEMENTS; ++i) bv.push_back(i % 2);
    benchmark::ClobberMemory();
  }
}

template <typename T>
static void BM_BitVector_Reserve_PushBack(benchmark::State& state) {
  for (auto _ : state) {
    BitVector<T> bv;
    bv.reserve(NUM_ELEMENTS);
    for (size_t i = 0; i < NUM_ELEMENTS; ++i) bv.push_back(i % 2);
    benchmark::ClobberMemory();
  }
}

BENCHMARK(BM_VectorBool_RandomAccess);

BENCHMARK_TEMPLATE(BM_BitVector_RandomAccess, uint8_t);
BENCHMARK_TEMPLATE(BM_BitVector_RandomAccess, uint16_t);
BENCHMARK_TEMPLATE(BM_BitVector_RandomAccess, uint32_t);
BENCHMARK_TEMPLATE(BM_BitVector_RandomAccess, uint64_t);

BENCHMARK(BM_VectorBool_Iteration);

BENCHMARK_TEMPLATE(BM_BitVector_Iteration, uint8_t);
BENCHMARK_TEMPLATE(BM_BitVector_Iteration, uint16_t);
BENCHMARK_TEMPLATE(BM_BitVector_Iteration, uint32_t);
BENCHMARK_TEMPLATE(BM_BitVector_Iteration, uint64_t);

BENCHMARK(BM_VectorBool_PushBack);

BENCHMARK_TEMPLATE(BM_BitVector_PushBack, uint8_t);
BENCHMARK_TEMPLATE(BM_BitVector_PushBack, uint16_t);
BENCHMARK_TEMPLATE(BM_BitVector_PushBack, uint32_t);
BENCHMARK_TEMPLATE(BM_BitVector_PushBack, uint64_t);

BENCHMARK(BM_VectorBool_Reserve_PushBack);

BENCHMARK_TEMPLATE(BM_BitVector_Reserve_PushBack, uint8_t);
BENCHMARK_TEMPLATE(BM_BitVector_Reserve_PushBack, uint16_t);
BENCHMARK_TEMPLATE(BM_BitVector_Reserve_PushBack, uint32_t);
BENCHMARK_TEMPLATE(BM_BitVector_Reserve_PushBack, uint64_t);

BENCHMARK_MAIN();
