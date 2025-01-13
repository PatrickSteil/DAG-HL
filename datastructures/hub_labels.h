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
#include <sstream>
#include <vector>

#include "priority_queue.h"
#include "status_log.h"
#include "types.h"
#include "utils.h"

enum DIRECTION : bool { FWD, BWD };

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
  mutable std::mutex mutex;

  LabelThreadSafe() {}

  LabelThreadSafe(const LabelThreadSafe &other) : Label(other) {}

  LabelThreadSafe(LabelThreadSafe &&other) noexcept : Label(std::move(other)) {}

  LabelThreadSafe &operator=(const LabelThreadSafe &other) {
    if (this != &other) {
      std::lock_guard<std::mutex> lock(mutex);
      Label::operator=(other);
    }
    return *this;
  }

  LabelThreadSafe &operator=(LabelThreadSafe &&other) noexcept {
    if (this != &other) {
      std::lock_guard<std::mutex> lock(mutex);
      Label::operator=(std::move(other));
    }
    return *this;
  }

  void add(const Vertex hub) {
    std::lock_guard<std::mutex> lock(mutex);
    nodes.push_back(hub);
  };

  bool contains(const Vertex hub) {
    std::lock_guard<std::mutex> lock(mutex);
    return std::find(nodes.begin(), nodes.end(), hub) != nodes.end();
  };

  template <typename FUNC>
  void doForAll(FUNC &&apply) {
    std::lock_guard<std::mutex> lock(mutex);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      auto &h = nodes[i];
      apply(h);
    }
  }

  template <typename FUNC>
  bool appliesToAny(FUNC &&toVerfiy) const {
    std::lock_guard<std::mutex> lock(mutex);
    return std::any_of(nodes.begin(), nodes.end(), toVerfiy);
  }

  void sort() {
    std::lock_guard<std::mutex> lock(mutex);
    std::sort(nodes.begin(), nodes.end());
  }
  // all other methods will likely not be called in parallel
};

template <class LABEL = Label>
bool query(std::array<std::vector<LABEL>, 2> &labels, const Vertex from,
           const Vertex to) {
  if (from == to) [[unlikely]]
    return true;
  std::size_t i = 0;
  std::size_t j = 0;

  const auto &fromLabels = labels[FWD][from];
  const auto &toLabels = labels[BWD][to];

  while (i < fromLabels.size() && j < toLabels.size()) {
    if (fromLabels[i] == toLabels[j]) return true;

    if (fromLabels[i] < toLabels[j]) {
      i++;
    } else {
      j++;
    }
  }

  return false;
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
    while (iss >> hub) {
      hubs.push_back(hub);
    }

    while (labels[FWD].size() <= vertexIndex) {
      labels[FWD].emplace_back();
      labels[BWD].emplace_back();
    }

    if (labelType == 'o') {
      labels[FWD][vertexIndex].nodes = std::move(hubs);
    } else if (labelType == 'i') {
      labels[BWD][vertexIndex].nodes = std::move(hubs);
      ++vertexIndex;
    }
  }

  inFile.close();
  if (inFile.fail()) {
    std::cerr << "Error: Reading from file " << fileName << " failed.\n";
  } else {
    std::cout << "Labels loaded successfully from " << fileName << "\n";
  }
}

template <class LABEL = Label>
std::vector<Vertex> computePermutation(
    const std::array<std::vector<LABEL>, 2> &labels) {
  StatusLog log("Compute Hub permutation");
  const std::size_t numVertices = labels[0].size();

  std::vector<Vertex> result;
  parallel_assign_iota(result, numVertices, Vertex(0));

  std::vector<std::uint32_t> freq;
  parallel_assign(freq, numVertices, static_cast<std::uint32_t>(0));

  for (DIRECTION dir : {FWD, BWD}) {
    for (auto &lab : labels[dir]) {
      for (const auto h : lab.nodes) {
        freq[h]++;
      }
    }
  }

  PriorityQueue<std::greater<std::uint32_t>> q;
  q.buildFrom(freq);

  Vertex id(0);
  while (!q.empty()) {
    auto [occur, hub] = q.pop();
    assert(hub != noVertex);

    result[hub] = id;
    ++id;
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
void benchmark(std::array<std::vector<LABEL>, 2> &labels,
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
