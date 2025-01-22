#include <benchmark/benchmark.h>

#include <memory>
#include <unordered_map>
#include <vector>

#include "../datastructures/forest.h"

constexpr std::size_t DEFAULT_N = (1 << 14);

std::shared_ptr<std::vector<Index>> createTopoRank(std::size_t N) {
  auto topoRank = std::make_shared<std::vector<Index>>(N);
  for (std::size_t i = 0; i < N; ++i) {
    (*topoRank)[i] = i;
  }
  return topoRank;
}

template <typename T>
std::unique_ptr<T> createTree(std::size_t N) {
  auto topoRank = createTopoRank(N);
  auto tree = std::make_unique<T>(N, topoRank);
  tree->setRoot(0);
  for (std::size_t i = 0; i < N - 1; ++i) {
    tree->addEdge(i, i + 1, FWD);
  }
  return tree;
}

template <typename T>
void BM_ComputeDescendants(benchmark::State& state) {
  std::size_t N = state.range(0);
  auto tree = createTree<T>(N);
  for (auto _ : state) {
    benchmark::DoNotOptimize(tree);
    tree->clearDescendants();
    tree->computeDescendants();
  }
}

template <typename T>
void BM_RemoveSubtree(benchmark::State& state) {
  std::size_t N = state.range(0);
  for (auto _ : state) {
    auto tree = createTree<T>(N);
    tree->clearDescendants();
    tree->computeDescendants();
    benchmark::DoNotOptimize(tree);
    tree->removeSubtree(1);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_TEMPLATE(BM_ComputeDescendants, EdgeTreeVec)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_ComputeDescendants, EdgeTreeMap)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_RemoveSubtree, EdgeTreeVec)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_RemoveSubtree, EdgeTreeMap)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);

BENCHMARK_MAIN();
