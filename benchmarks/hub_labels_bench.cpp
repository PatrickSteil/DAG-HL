#include <benchmark/benchmark.h>
#include <mutex>
#include <random>
#include <vector>

#include "../datastructures/hub_labels.h"

const Vertex maxVertex = (1 << 18);

std::vector<Vertex> generateRandomVertices(std::size_t count, Vertex maxValue) {
  std::vector<Vertex> vertices(count);
  std::mt19937 rng(42); // Fixed seed for reproducibility
  std::uniform_int_distribution<Vertex> dist(0, maxValue);
  for (auto &vertex : vertices) {
    vertex = dist(rng);
  }
  return vertices;
}

static void BM_Label_Add(benchmark::State &state) {
  auto vertices = generateRandomVertices(state.range(0), maxVertex);
  Label label;
  for (auto _ : state) {
    for (const auto &vertex : vertices) {
      label.add(vertex);
    }
  }
}

static void BM_Label_Contains(benchmark::State &state) {
  auto vertices = generateRandomVertices(state.range(0), maxVertex);
  Label label;
  for (const auto &vertex : vertices) {
    label.add(vertex);
  }
  for (auto _ : state) {
    for (const auto &vertex : vertices) {
      benchmark::DoNotOptimize(label.contains(vertex));
    }
  }
}

static void BM_LabelThreadSafe_Add(benchmark::State &state) {
  auto vertices = generateRandomVertices(state.range(0), maxVertex);
  LabelThreadSafe label;
  for (auto _ : state) {
    for (const auto &vertex : vertices) {
      label.add(vertex);
    }
  }
  state.SetComplexityN(state.range(0));
}

static void BM_LabelThreadSafe_Contains(benchmark::State &state) {
  auto vertices = generateRandomVertices(state.range(0), maxVertex);
  LabelThreadSafe label;
  for (const auto &vertex : vertices) {
    label.add(vertex);
  }
  for (auto _ : state) {
    for (const auto &vertex : vertices) {
      benchmark::DoNotOptimize(label.contains(vertex));
    }
  }
}

BENCHMARK(BM_Label_Add)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_Label_Contains)->Range(1 << 6, 1 << 13);
BENCHMARK(BM_LabelThreadSafe_Add)->Range(1 << 10, 1 << 20);
BENCHMARK(BM_LabelThreadSafe_Contains)->Range(1 << 6, 1 << 13);
