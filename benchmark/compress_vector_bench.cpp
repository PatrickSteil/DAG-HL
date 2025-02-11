#include <benchmark/benchmark.h>

#include <numeric>
#include <random>
#include <vector>

#include "../datastructures/compressed_vector.h"
#include "../datastructures/utils.h"

std::vector<std::uint32_t> generateEvenNumbers(std::size_t size) {
  std::vector<std::uint32_t> numbers;
  numbers.reserve(size);
  for (std::uint32_t i = 0; i < size - 1; i += 2) {
    numbers.push_back(i);
  }
  numbers.push_back(size - 1);  // Ensure N-1 is included
  return numbers;
}

std::vector<std::uint32_t> generateOddNumbers(std::size_t size) {
  std::vector<std::uint32_t> numbers;
  numbers.reserve(size);
  for (std::uint32_t i = 1; i < size - 1; i += 2) {
    numbers.push_back(i);
  }
  numbers.push_back(size - 1);  // Ensure N-1 is included
  return numbers;
}

static void BM_Intersect_Vector(benchmark::State& state) {
  std::size_t N = state.range(0);
  std::vector<std::uint32_t> vec = generateEvenNumbers(N);
  std::vector<std::uint32_t> query = generateOddNumbers(N);

  for (auto _ : state) {
    bool result = intersect(vec.begin(), vec.end(), query.begin(), query.end());
    assert(result);
    benchmark::DoNotOptimize(result);
  }
}

static void BM_Intersect_CompressedVector(benchmark::State& state) {
  std::size_t N = state.range(0);
  std::vector<std::uint32_t> numbers = generateEvenNumbers(N);
  std::vector<std::uint32_t> query = generateOddNumbers(N);

  CompressedVector compressed(numbers);
  CompressedVector compressed_query(query);

  for (auto _ : state) {
    bool result = intersect(compressed.begin(), compressed.end(),
                            compressed_query.begin(), compressed_query.end());
    assert(result);
    benchmark::DoNotOptimize(result);
  }
}

static void BM_Intersect_DeltaCompressedVector(benchmark::State& state) {
  std::size_t N = state.range(0);
  std::vector<std::uint32_t> numbers = generateEvenNumbers(N);
  std::vector<std::uint32_t> query = generateOddNumbers(N);

  CompressedVector delta_compressed;
  delta_compressed.reserve(N * 5);
  delta_compressed.push_back(numbers.front());
  for (std::size_t i = 1; i < numbers.size(); ++i) {
    delta_compressed.push_back(numbers[i] - numbers[i - 1] - 1);
  }

  CompressedVector delta_query;
  delta_query.reserve(N * 5);
  delta_query.push_back(query.front());
  for (std::size_t i = 1; i < query.size(); ++i) {
    delta_query.push_back(query[i] - query[i - 1] - 1);
  }

  for (auto _ : state) {
    bool result =
        intersect_delta(delta_compressed.begin(), delta_compressed.end(),
                        delta_query.begin(), delta_query.end());
    assert(result);
    benchmark::DoNotOptimize(result);
  }
}

BENCHMARK(BM_Intersect_Vector)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_Intersect_CompressedVector)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_Intersect_DeltaCompressedVector)->Range(1 << 10, 1 << 20);

BENCHMARK_MAIN();
