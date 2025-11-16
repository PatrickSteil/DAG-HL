/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#ifdef __GNUC__
#define PREFETCH(addr) __builtin_prefetch(addr)
#else
#define PREFETCH(addr)
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <queue>
#include <ranges>
#include <set>
#include <sstream>
#include <vector>

#include "graph.h"
#include "types.h"
#include "utils.h"

struct HubEntry {
  Vertex node;
  Weight dist;
};

struct Label {
  std::vector<HubEntry> hubs;
  Label(){};

  Label(const Label &other) : hubs(other.hubs) {}

  Label(Label &&other) noexcept : hubs(std::move(other.hubs)) {}

  Label &operator=(const Label &other) {
    if (this != &other) {
      hubs = other.hubs;
    }
    return *this;
  }

  Label &operator=(Label &&other) noexcept {
    if (this != &other) {
      hubs = std::move(other.hubs);
    }
    return *this;
  }

  void reserve(const std::size_t size) { hubs.reserve(size); };

  std::size_t size() const { return hubs.size(); };

  void add(const Vertex hub, const Weight dist) {
    hubs.emplace_back(hub, dist);
  };

  void sort() {
    std::sort(
        hubs.begin(), hubs.end(),
        [](const HubEntry &a, const HubEntry &b) { return a.node < b.node; });
  };

  template <typename FUNC>
  void doForAll(FUNC &&apply) {
    for (std::size_t i = 0; i < hubs.size(); ++i) {
      auto &h = hubs[i];
      apply(h.node, h.dist);
    }
  }

  bool prune(const std::vector<Weight> &lookup, const Weight newDistance) {
    bool result = false;
    doForAll([&](const Vertex h, const Weight dist) {
      assert(h < lookup.size());
      result |= lookup[h] + dist <= newDistance;
    });
    return result;
  }
};

Weight query(std::array<std::vector<Label>, 2> &labels, const Vertex from,
             const Vertex to) {
  assert(from < labels[FWD].size());
  assert(from < labels[BWD].size());
  assert(to < labels[FWD].size());
  assert(to < labels[BWD].size());

  if (from == to) [[unlikely]]
    return 0;

  Weight result = noWeight;
  const auto &fromLabels = labels[FWD][from];
  const auto &toLabels = labels[BWD][to];

  std::size_t i = 0;
  std::size_t j = 0;

  const std::size_t ASize = fromLabels.size();
  const std::size_t BSize = toLabels.size();

  while (i < ASize && j < BSize) {
    if (fromLabels.hubs[i].node == toLabels.hubs[j].node)
      result = min_u8(fromLabels.hubs[i].dist + toLabels.hubs[i].dist, result);

    if (fromLabels.hubs[i].node < toLabels.hubs[j].node)
      ++i;
    else
      ++j;
  }

  return result;
}

template <class LABEL = Label>
std::vector<Vertex> computePermutation(
    const std::array<std::vector<LABEL>, 2> &labels) {
  StatusLog log("Compute Hub permutation");
  const std::size_t numVertices = labels[0].size();

  std::vector<Vertex> result;
  parallel_assign_iota(result, numVertices, Vertex(0));

  std::vector<std::atomic<uint32_t>> freq(numVertices);

#pragma omp parallel for
  for (std::size_t v = 0; v < numVertices; ++v) {
    freq[v].store(0, std::memory_order_relaxed);
  }

  for (DIRECTION dir : {FWD, BWD}) {
#pragma omp for
    for (std::size_t v = 0; v < numVertices; ++v) {
      for (const auto &h : labels[dir][v].hubs) {
        freq[h.node].fetch_add(1, std::memory_order_relaxed);
      }
    }
  }

  using Pair = std::pair<std::uint32_t, Vertex>;
  std::vector<Pair> heap_data(numVertices);

#pragma omp parallel for
  for (Vertex v = 0; v < numVertices; ++v) {
    heap_data[v] = {freq[v].load(std::memory_order_relaxed), v};
  }

  ips4o::parallel::sort(
      heap_data.begin(), heap_data.end(),
      [](const Pair &a, const Pair &b) { return a.first > b.first; });

#pragma omp parallel for
  for (Vertex id = 0; id < numVertices; ++id) {
    result[heap_data[id].second] = id;
  }

  return result;
}

template <class LABEL = Label>
void sortLabels(std::array<std::vector<LABEL>, 2> &labels) {
  StatusLog log("Sort all labels");

#pragma omp parallel for
  for (std::size_t i = 0; i < labels[FWD].size(); ++i) {
    labels[FWD][i].sort();
    labels[BWD][i].sort();
  }
}

void benchmark_hublabels(std::array<std::vector<Label>, 2> &labels,
                         const std::size_t numQueries) {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;

  assert(labels[FWD].size() == labels[BWD].size());

  auto queries =
      generateRandomQueries<Vertex>(numQueries, 0, labels[FWD].size());
  std::size_t counter = 0;
  auto t1 = high_resolution_clock::now();
  for (const std::pair<Vertex, Vertex> &paar : queries) {
    counter += (query(labels, paar.first, paar.second) != noWeight);
  }

  auto t2 = high_resolution_clock::now();
  duration<double, std::nano> nano_double = t2 - t1;
  long double total_ns = nano_double.count();
  std::cout << numQueries << " queries: total " << total_ns << " ns, avg "
            << (total_ns / numQueries) << " ns/query, counter=" << counter
            << "\n";
}

void saveToFile(std::array<std::vector<Label>, 2> &labels,
                const std::string &fileName) {
  std::ofstream outFile(fileName);

  if (!outFile.is_open()) {
    std::cerr << "Error: Unable to open file " << fileName << " for writing.\n";
    return;
  }

  std::size_t N = labels[FWD].size();

  outFile << "V " << N << "\n";

  for (std::size_t v = 0; v < N; ++v) {
    outFile << "o " << v;
    labels[FWD][v].doForAll([&](const Vertex hub, const Weight dist) {
      outFile << " " << hub << " " << (int)dist;
    });
    outFile << "\n";

    outFile << "i " << v;
    labels[BWD][v].doForAll([&](const Vertex hub, const Weight dist) {
      outFile << " " << hub << " " << (int)dist;
    });
    outFile << "\n";
  }

  outFile.close();
  if (outFile.fail()) {
    std::cerr << "Error: Writing to file " << fileName << " failed.\n";
  } else {
    std::cout << "Labels saved successfully to " << fileName << "\n";
  }
}

std::size_t computeTotalBytes(const std::array<std::vector<Label>, 2> &labels) {
  std::size_t totalBytes = 0;

  for (const auto &labelSet : labels) {
    for (const auto &label : labelSet) {
      totalBytes += sizeof(Label);
      totalBytes += label.hubs.capacity() * sizeof(HubEntry);
    }
  }

  return totalBytes;
}

void showStats(const std::array<std::vector<Label>, 2> &labels) {
  auto computeStats = [](const std::vector<Label> &currentLabels) {
    std::size_t minSize = std::numeric_limits<std::size_t>::max();
    std::size_t maxSize = 0;
    std::size_t totalSize = 0;

    for (const auto &label : currentLabels) {
      std::size_t size = label.size();
      minSize = std::min(minSize, size);
      maxSize = std::max(maxSize, size);
      totalSize += size;
    }

    double avgSize = static_cast<double>(totalSize) / currentLabels.size();
    return std::make_tuple(minSize, maxSize, avgSize, totalSize);
  };

  auto [inMin, inMax, inAvg, inTotal] = computeStats(labels[BWD]);
  auto [outMin, outMax, outAvg, outTotal] = computeStats(labels[FWD]);

  std::cout << "Forward Labels Statistics:" << std::endl;
  std::cout << "  Min Size:     " << outMin << std::endl;
  std::cout << "  Max Size:     " << outMax << std::endl;
  std::cout << "  Avg Size:     " << outAvg << std::endl;

  std::cout << "Backward Labels Statistics:" << std::endl;
  std::cout << "  Min Size:     " << inMin << std::endl;
  std::cout << "  Max Size:     " << inMax << std::endl;
  std::cout << "  Avg Size:     " << inAvg << std::endl;

  std::cout << "FWD # count:    " << outTotal << std::endl;
  std::cout << "BWD # count:    " << inTotal << std::endl;
  std::cout << "Both # count:   " << (outTotal + inTotal) << std::endl;

  std::cout << "Total memory consumption [megabytes]:" << std::endl;
  std::cout << "  "
            << static_cast<double>(computeTotalBytes(labels) /
                                   (1024.0 * 1024.0))
            << std::endl;
}

std::vector<Vertex> getOrdering(const std::string &fileName,
                                const std::array<const Graph *, 2> &graph) {
  StatusLog log("Load / Compute Ordering");
  std::vector<Vertex> ordering;
  ordering.reserve(graph[FWD]->numVertices());

  if (fileName == "") {
    parallel_assign(ordering, graph[FWD]->numVertices(), Vertex(0));

    std::vector<std::size_t> randomNumber;
    parallel_assign_iota(randomNumber, graph[FWD]->numVertices(),
                         static_cast<std::size_t>(0));

    std::mt19937 g(42);

    std::shuffle(randomNumber.begin(), randomNumber.end(), g);

    auto degreeCompRandom = [&](const auto left, const auto right) {
      return std::forward_as_tuple(
                 graph[FWD]->degree(left) + graph[BWD]->degree(left),
                 randomNumber[left]) >
             std::forward_as_tuple(
                 graph[FWD]->degree(right) + graph[BWD]->degree(right),
                 randomNumber[right]);
    };

    parallel_iota(ordering, Vertex(0));
    ips4o::parallel::sort(ordering.begin(), ordering.end(), degreeCompRandom);
    assert(std::is_sorted(ordering.begin(), ordering.end(), degreeCompRandom));
  } else {
    std::ifstream file(fileName);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + fileName);
    }

    std::string line;
    while (std::getline(file, line)) {
      std::istringstream iss(line);
      Vertex vertex;
      double centrality;

      if (iss >> vertex >> centrality) {
        ordering.push_back(vertex - 1);
      } else {
        throw std::runtime_error("Failed to parse line: " + line);
      }
    }
  }
  return ordering;
}

bool isOrdering(const std::vector<Vertex> &ordering,
                const std::size_t numVertices) {
  std::set<Vertex> orderedSet(ordering.begin(), ordering.end());

  if (orderedSet.size() != numVertices) {
    std::cout << "The ordering does not contain all vertices!" << std::endl;
    std::cout << "Ordering has " << orderedSet.size() << ", but there are "
              << numVertices << " many vertices!" << std::endl;
    return false;
  }
  if (!orderedSet.contains(0)) {
    std::cout << "The ordering does not contain 0!" << std::endl;
    return false;
  }
  if (!orderedSet.contains(numVertices - 1)) {
    std::cout << "The ordering does not contain the last vertex!" << std::endl;
    return false;
  }

  return true;
};
