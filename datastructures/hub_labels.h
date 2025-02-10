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

#include <omp.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <vector>

#include "compressed_vector.h"
#include "graph.h"
#include "ips4o.hpp"
#include "spinlock.h"
#include "status_log.h"
#include "types.h"
#include "utils.h"

struct Label {
  Label(){};

  Label(const Label &other) : nodes(other.nodes) {}

  Label(Label &&other) noexcept : nodes(std::move(other.nodes)) {}

  Label &operator=(const Label &other) {
    if (this != &other) {
      nodes = other.nodes;
    }
    return *this;
  }

  Label &operator=(Label &&other) noexcept {
    if (this != &other) {
      nodes = std::move(other.nodes);
    }
    return *this;
  }

  std::vector<Vertex> nodes;

  Vertex &operator[](std::size_t i) { return nodes[i]; }
  const Vertex &operator[](std::size_t i) const { return nodes[i]; }

  template <typename FUNC>
  void doForAll(FUNC &&apply) {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      auto &h = nodes[i];
      apply(h);
    }
  }

  // toVerfiy should take (const Vertex h) as argument
  template <typename FUNC>
  bool appliesToAny(FUNC &&toVerfiy) const {
    return std::any_of(nodes.begin(), nodes.end(), toVerfiy);
  }

  bool prune(const std::vector<std::uint8_t> &lookup) {
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (i + 4 < nodes.size()) {
        PREFETCH(&lookup[nodes[i + 4]]);
      }

      if (lookup[nodes[i]]) {
        return true;
      }
    }

    return false;
  }

  void reserve(const std::size_t size) { nodes.reserve(size); };

  std::size_t size() const { return nodes.size(); };

  void add(const Vertex hub) { nodes.push_back(hub); };

  bool contains(const Vertex hub) {
    return std::find(nodes.begin(), nodes.end(), hub) != nodes.end();
  };

  void applyPermutation(const std::vector<Vertex> &permutation) {
    for (auto &hub : nodes) {
      hub = permutation[hub];
    }

    std::sort(nodes.begin(), nodes.end());
  }

  void sort() { std::sort(nodes.begin(), nodes.end()); }

  void setDeltaRepresentation() {
    if (nodes.empty()) return;

    std::vector<Vertex> new_nodes;
    new_nodes.reserve(nodes.size());

    new_nodes.push_back(nodes.front());

    auto prevHub = nodes.front();
    for (std::size_t i = 1; i < nodes.size(); ++i) {
      new_nodes.push_back(nodes[i] - prevHub - 1);
      prevHub = nodes[i];
    }

    nodes = std::move(new_nodes);
  }
};

struct LabelThreadSafe : Label {
  mutable Spinlock mutex;

  LabelThreadSafe() {}

  LabelThreadSafe(const LabelThreadSafe &other) : Label(other) {}

  LabelThreadSafe(LabelThreadSafe &&other) noexcept : Label(std::move(other)) {}

  LabelThreadSafe &operator=(const LabelThreadSafe &other) {
    if (this != &other) {
      std::lock_guard<Spinlock> lock(mutex);
      Label::operator=(other);
    }
    return *this;
  }

  LabelThreadSafe &operator=(LabelThreadSafe &&other) noexcept {
    if (this != &other) {
      std::lock_guard<Spinlock> lock(mutex);
      Label::operator=(std::move(other));
    }
    return *this;
  }

  void add(const Vertex hub) {
    std::lock_guard<Spinlock> lock(mutex);
    nodes.push_back(hub);
  };

  bool contains(const Vertex hub) {
    std::lock_guard<Spinlock> lock(mutex);
    return std::find(nodes.begin(), nodes.end(), hub) != nodes.end();
  };

  template <typename FUNC>
  void doForAll(FUNC &&apply) {
    std::lock_guard<Spinlock> lock(mutex);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      auto &h = nodes[i];
      apply(h);
    }
  }

  template <typename FUNC>
  bool appliesToAny(FUNC &&toVerfiy) const {
    std::lock_guard<Spinlock> lock(mutex);
    return std::any_of(nodes.begin(), nodes.end(), toVerfiy);
  }

  bool prune(const std::vector<std::uint8_t> &lookup) {
    std::lock_guard<Spinlock> lock(mutex);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (i + 4 < nodes.size()) {
        PREFETCH(&lookup[nodes[i + 4]]);
      }

      if (lookup[nodes[i]]) {
        return true;
      }
    }

    return false;
  }

  void sort() {
    std::lock_guard<Spinlock> lock(mutex);
    std::sort(nodes.begin(), nodes.end());
  }
  // all other methods will likely not be called in parallel
};

struct CompressedLabel {
  CompressedVector nodes;

  CompressedLabel(){};
  CompressedLabel(const CompressedLabel &other) : nodes(other.nodes) {}
  CompressedLabel(CompressedLabel &&other) noexcept
      : nodes(std::move(other.nodes)) {}

  CompressedLabel(const Label &other) : nodes(other.nodes) {}

  std::size_t size() const { return nodes.size(); };
};

template <class LABEL = Label>
bool query(std::array<std::vector<LABEL>, 2> &labels, const Vertex from,
           const Vertex to) {
  assert(from < labels[FWD].size());
  assert(from < labels[BWD].size());
  assert(to < labels[FWD].size());
  assert(to < labels[BWD].size());

  if (from == to) [[unlikely]]
    return true;

  const auto &fromLabels = labels[FWD][from];
  const auto &toLabels = labels[BWD][to];

  return intersect(fromLabels.nodes.begin(), fromLabels.nodes.end(),
                   toLabels.nodes.begin(), toLabels.nodes.end());
}

// TODO test this method
bool queryDeltaRepresentation(std::array<std::vector<Label>, 2> &labels,
                              const Vertex from, const Vertex to) {
  if (from == to) [[unlikely]]
    return true;

  const auto &fromLabels = labels[FWD][from];
  const auto &toLabels = labels[BWD][to];

  if (fromLabels.size() == 0 || toLabels.size() == 0) [[unlikely]] {
    return false;
  }

  Vertex fromHub = fromLabels[0];
  Vertex toHub = toLabels[0];

  std::size_t i = 0;
  std::size_t j = 0;

  while (i < fromLabels.size() && j < toLabels.size()) {
    if (fromHub == toHub) {
      return true;
    }

    if (fromHub < toHub) {
      fromHub += 1 + fromLabels[++i];
    } else {
      toHub += 1 + toLabels[++j];
    }
  }

  return false;
}

template <class LABEL = Label>
void saveToFile(std::array<std::vector<LABEL>, 2> &labels,
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
    for (const Vertex hub : labels[FWD][v].nodes) {
      outFile << " " << hub;
    }
    outFile << "\n";

    outFile << "i " << v;
    for (const Vertex hub : labels[BWD][v].nodes) {
      outFile << " " << hub;
    }
    outFile << "\n";
  }

  outFile.close();
  if (outFile.fail()) {
    std::cerr << "Error: Writing to file " << fileName << " failed.\n";
  } else {
    std::cout << "Labels saved successfully to " << fileName << "\n";
  }
}

template <class LABEL = Label>
void readFromFile(std::array<std::vector<LABEL>, 2> &labels,
                  const std::string &fileName) {
  std::ifstream inFile(fileName);

  if (!inFile.is_open()) {
    std::cerr << "Error: Unable to open file " << fileName << " for reading.\n";
    return;
  }

  std::string line;
  std::size_t vertexIndex = 0;

  std::getline(inFile, line);
  if (line.substr(0, 2) == "V ") {
    std::istringstream iss(line.substr(2));
    std::size_t numNodes;
    iss >> numNodes;

    labels[FWD].resize(numNodes);
    labels[BWD].resize(numNodes);
  } else {
    std::cerr << "Error: First line format invalid, expected 'V {num nodes}'\n";
    return;
  }

  while (std::getline(inFile, line)) {
    if (line.empty()) {
      continue;
    }

    char labelType = line[0];

    if (labelType != 'o' && labelType != 'i') {
      std::cerr << "Error: Unexpected line format: " << line << "\n";
      continue;
    }

    std::vector<Vertex> hubs;
    std::istringstream iss(line.substr(1));
    Vertex hub;

    // first element is the vertex itself
    iss >> vertexIndex;
    assert(vertexIndex < labels[FWD].size());
    assert(vertexIndex < labels[BWD].size());

    while (iss >> hub) {
      hubs.push_back(hub);
    }

    if (labelType == 'o') {
      labels[FWD][vertexIndex].nodes = std::move(hubs);
    } else if (labelType == 'i') {
      labels[BWD][vertexIndex].nodes = std::move(hubs);
    }
  }

  inFile.close();
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
      for (const auto h : labels[dir][v].nodes) {
        freq[h].fetch_add(1, std::memory_order_relaxed);
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

template <class LABEL = Label>
void benchmark_hublabels(std::array<std::vector<LABEL>, 2> &labels,
                         const std::size_t numQueries) {
  using std::chrono::duration;
  using std::chrono::duration_cast;
  using std::chrono::high_resolution_clock;
  using std::chrono::milliseconds;

  assert(labels[FWD].size() == labels[BWD].size());

  auto queries =
      generateRandomQueries<Vertex>(numQueries, 0, labels[FWD].size());
  long double totalTime(0);
  for (std::pair<Vertex, Vertex> paar : queries) {
    auto t1 = high_resolution_clock::now();
    query(labels, paar.first, paar.second);
    auto t2 = high_resolution_clock::now();
    duration<double, std::nano> nano_double = t2 - t1;
    totalTime += nano_double.count();
  }

  std::cout << "The " << numQueries << " random queries took in total "
            << totalTime << " [ms] and on average "
            << (double)(totalTime / numQueries) << " [ns]!\n";
}
template <typename LABEL = Label>
std::size_t computeTotalBytes(const std::array<std::vector<LABEL>, 2> &labels) {
  std::size_t totalBytes = 0;

  for (const auto &labelSet : labels) {
    for (const auto &label : labelSet) {
      totalBytes += sizeof(LABEL);
      totalBytes += label.nodes.capacity() * sizeof(Vertex);
    }
  }

  return totalBytes;
}

template <typename LABEL = Label>
void showStats(const std::array<std::vector<LABEL>, 2> &labels) {
  auto computeStats = [](const std::vector<LABEL> &currentLabels) {
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

  /* std::locale::global(std::locale("")); */
  /* std::cout.imbue(std::locale()); */

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
