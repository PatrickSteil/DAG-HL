#include <benchmark/benchmark.h>

#include <random>
#include <vector>

#include "../datastructures/compressed_vector.h"
#include "../datastructures/utils.h"

std::vector<std::uint32_t> generateRandomNumbers(
    std::size_t size, std::uint32_t max_value = 1'000'000) {
  std::vector<std::uint32_t> numbers(size);
  std::mt19937 rng(42);
  std::uniform_int_distribution<std::uint32_t> dist(0, max_value);

  for (auto& num : numbers) {
    num = dist(rng);
  }
  return numbers;
}

static void BM_CreateCompressedVector(benchmark::State& state) {
  std::vector<std::uint32_t> numbers = generateRandomNumbers(state.range(0));

  for (auto _ : state) {
    CompressedVector compressed(numbers);
    benchmark::DoNotOptimize(compressed);
  }
}

static void BM_IterateCompressedVector(benchmark::State& state) {
  std::vector<std::uint32_t> numbers = generateRandomNumbers(state.range(0));
  CompressedVector compressed(numbers);

  for (auto _ : state) {
    std::uint32_t sum = 0;
    for (auto num : compressed) {
      sum += num;
    }
    benchmark::DoNotOptimize(sum);
  }
}

static void BM_Intersect_Vector(benchmark::State& state) {
  std::size_t N = state.range(0);
  std::vector<std::uint32_t> vec(N);
  std::iota(vec.begin(), vec.end(), 0);  // Fill with [0, 1, 2, ..., N-1]

  std::vector<std::uint32_t> query = {static_cast<std::uint32_t>(N) - 1};

  for (auto _ : state) {
    bool result = intersect(vec.begin(), vec.end(), query.begin(), query.end());
    benchmark::DoNotOptimize(result);
  }
}

static void BM_Intersect_CompressedVector(benchmark::State& state) {
  std::size_t N = state.range(0);
  std::vector<std::uint32_t> numbers(N);
  std::iota(numbers.begin(), numbers.end(), 0);  // Fill with [0, 1, ..., N-1]

  CompressedVector compressed(numbers);
  std::vector<std::uint32_t> query = {static_cast<std::uint32_t>(N) - 1};
  CompressedVector compressed_query(query);

  for (auto _ : state) {
    bool result = intersect(compressed.begin(), compressed.end(),
                            compressed_query.begin(), compressed_query.end());
    benchmark::DoNotOptimize(result);
  }
}

BENCHMARK(BM_CreateCompressedVector)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_IterateCompressedVector)->Range(1 << 10, 1 << 20);

BENCHMARK(BM_Intersect_Vector)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_Intersect_CompressedVector)->Range(1 << 10, 1 << 20);

BENCHMARK_MAIN();
