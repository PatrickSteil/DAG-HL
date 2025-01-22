#include <benchmark/benchmark.h>

#include <algorithm>
#include <memory>
#include <random>
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
std::unique_ptr<T> createSequentialTree(std::size_t N) {
  auto topoRank = createTopoRank(N);
  auto tree = std::make_unique<T>(N, topoRank);
  tree->setRoot(0);
  for (std::size_t i = 0; i < N - 1; ++i) {
    tree->addEdge(i, i + 1, FWD);
  }
  return tree;
}

template <typename T>
std::unique_ptr<T> createRandomTree(std::size_t N,
                                    std::vector<std::size_t>& nodes) {
  auto topoRank = createTopoRank(N);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    (*topoRank)[nodes[i]] = i;
  }
  auto tree = std::make_unique<T>(N, topoRank);
  tree->setRoot(0);

  for (std::size_t i = 0; i < N - 1; ++i) {
    tree->addEdge(nodes[i], nodes[i + 1], FWD);
  }
  return tree;
}

template <typename T>
void BM_ComputeDescendants_Seq(benchmark::State& state) {
  std::size_t N = state.range(0);
  auto tree = createSequentialTree<T>(N);
  for (auto _ : state) {
    benchmark::DoNotOptimize(tree);
    tree->clearDescendants();
    tree->computeDescendants();
  }
}

template <typename T>
void BM_ComputeDescendants_Rand(benchmark::State& state) {
  std::size_t N = state.range(0);
  std::vector<std::size_t> nodes(N);
  for (std::size_t i = 0; i < N; ++i) {
    nodes[i] = i;
  }
  std::shuffle(nodes.begin(), nodes.end(),
               std::mt19937{std::random_device{}()});
  auto tree = createRandomTree<T>(N, nodes);
  for (auto _ : state) {
    benchmark::DoNotOptimize(tree);
    tree->clearDescendants();
    tree->computeDescendants();
  }
}

template <typename T>
void BM_RemoveSubtree_Seq(benchmark::State& state) {
  std::size_t N = state.range(0);
  for (auto _ : state) {
    auto tree = createSequentialTree<T>(N);
    tree->clearDescendants();
    tree->computeDescendants();
    benchmark::DoNotOptimize(tree);
    tree->removeSubtree(1);
    benchmark::ClobberMemory();
  }
}

template <typename T>
void BM_Remove100Subtree_Rand(benchmark::State& state) {
  std::size_t N = state.range(0);
  for (auto _ : state) {
    std::vector<std::size_t> nodes(N);
    for (std::size_t i = 0; i < N; ++i) {
      nodes[i] = i;
    }
    std::shuffle(nodes.begin(), nodes.end(),
                 std::mt19937{std::random_device{}()});
    auto tree = createRandomTree<T>(N, nodes);
    tree->clearDescendants();
    tree->computeDescendants();
    benchmark::DoNotOptimize(tree);
    for (int i = 0; i < 100 && i < static_cast<int>(N); ++i) {
      tree->removeSubtree(i);
    }
    benchmark::ClobberMemory();
  }
}

BENCHMARK_TEMPLATE(BM_ComputeDescendants_Seq, EdgeTreeVec)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_ComputeDescendants_Seq, EdgeTreeMap)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_ComputeDescendants_Rand, EdgeTreeVec)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_ComputeDescendants_Rand, EdgeTreeMap)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_RemoveSubtree_Seq, EdgeTreeVec)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_RemoveSubtree_Seq, EdgeTreeMap)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_Remove100Subtree_Rand, EdgeTreeVec)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);
BENCHMARK_TEMPLATE(BM_Remove100Subtree_Rand, EdgeTreeMap)
    ->RangeMultiplier(2)
    ->Range(32, DEFAULT_N);

BENCHMARK_MAIN();
