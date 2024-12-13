#include <algorithm>
#include <array>
#include <fstream>
#include <numeric>
#include <sstream>
#include <vector>

#include "priority_queue.h"
#include "status_log.h"
#include "types.h"

enum DIRECTION : bool { FWD, BWD };

struct Label {
  Label(){};

  std::vector<Vertex> nodes;

  Vertex &operator[](std::size_t i) { return nodes[i]; }
  const Vertex &operator[](std::size_t i) const { return nodes[i]; }

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

void saveToFile(std::array<std::vector<Label>, 2> &labels,
                const std::string &fileName) {
  std::ofstream outFile(fileName);

  if (!outFile.is_open()) {
    std::cerr << "Error: Unable to open file " << fileName << " for writing.\n";
    return;
  }

  std::size_t N = labels[FWD].size();

  for (std::size_t v = 0; v < N; ++v) {
    outFile << "o";
    for (const Vertex hub : labels[FWD][v].nodes) {
      outFile << " " << hub;
    }
    outFile << "\n";

    outFile << "i";
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

void readFromFile(std::array<std::vector<Label>, 2> &labels,
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

std::vector<Vertex> computePermutation(
    const std::array<std::vector<Label>, 2> &labels) {
  StatusLog log("Compute Hub permutation");
  const std::size_t numVertices = labels[0].size();
  std::vector<Vertex> result(numVertices);
  std::iota(result.begin(), result.end(), 0);

  std::vector<std::uint32_t> freq(numVertices, 0);

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

void sortLabels(std::array<std::vector<Label>, 2> &labels) {
  StatusLog log("Sort all labels");
#pragma omp parallel for schedule(dynamic, 64)
  for (std::size_t i = 0; i < labels[FWD].size(); ++i) {
    std::sort(labels[FWD][i].nodes.begin(), labels[FWD][i].nodes.end());
  }

#pragma omp parallel for schedule(dynamic, 64)
  for (std::size_t i = 0; i < labels[BWD].size(); ++i) {
    std::sort(labels[BWD][i].nodes.begin(), labels[BWD][i].nodes.end());
  }
}
