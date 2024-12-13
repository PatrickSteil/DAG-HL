#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include "types.h"

template <typename Comparator = std::less<uint32_t>>
class PriorityQueue {
 private:
  using HeapElement = std::pair<uint32_t, Vertex>;
  std::vector<HeapElement> heap;
  std::vector<Index> mapper;
  Comparator comp;

 public:
  PriorityQueue(size_t size = 0) : heap(), mapper(size, noIndex), comp() {}

  inline size_t size() const { return heap.size(); }
  inline bool empty() const { return heap.empty(); }
  inline size_t capacity() const { return mapper.size(); }

  inline bool isValid(Index i) const { return i < mapper.size(); }

  inline bool heapPropertyFullfilled() const {
    for (Index i = 0; i < size(); ++i) {
      Index left = getLeftChild(i);
      Index right = getRightChild(i);

      if (left < size() && comp(heap[left].first, heap[i].first)) {
        std::cerr << "[FAIL] Heap Property violated from " << int(i) << " to "
                  << int(left) << " (vals: " << heap[i].first << " <-> "
                  << heap[left].first << " )" << std::endl;
        return false;
      }
      if (right < size() && comp(heap[right].first, heap[i].first)) {
        std::cerr << "[FAIL] Heap Property violated from " << int(i) << " to "
                  << int(right) << " (vals: " << heap[i].first << " <-> "
                  << heap[right].first << " )" << std::endl;
        return false;
      }
    }
    return true;
  }

  inline uint32_t get(Vertex v) const {
    if (mapper[v] == noIndex) return uint32_t(-1);
    return heap[mapper[v]].first;
  }

  inline Index getLeftChild(Index i) const { return (i * 2) + 1; }

  inline Index getRightChild(Index i) const { return (i * 2) + 2; }

  inline Index getParentIndex(Index i) const { return (i - 1) / 2; }

  inline void swap(Index i, Index j) {
    assert(isValid(i) && isValid(j));
    mapper[heap[i].second] = j;
    mapper[heap[j].second] = i;
    std::swap(heap[i], heap[j]);
  }

  inline void siftDown(Index i) {
    assert(isValid(i));
    while (true) {
      auto left = getLeftChild(i);
      auto right = getRightChild(i);
      auto smallest = i;

      if (left < size() && comp(heap[left].first, heap[smallest].first)) {
        smallest = left;
      }
      if (right < size() && comp(heap[right].first, heap[smallest].first)) {
        smallest = right;
      }
      if (smallest == i) return;

      swap(i, smallest);
      i = smallest;
    }
  }

  inline void siftUp(Index i) {
    assert(isValid(i));
    while (i > 0) {
      Index parent = getParentIndex(i);
      if (comp(heap[i].first, heap[parent].first)) {
        swap(i, parent);
        i = parent;
      } else {
        return;
      }
    }
  }

  void push(Vertex v, uint32_t value) {
    if (v >= mapper.size()) {
      mapper.resize(v + 1, noIndex);
    }

    if (mapper[v] == noIndex) {
      heap.emplace_back(value, v);
      mapper[v] = size() - 1;
      siftUp(size() - 1);
    } else {
      Index i = mapper[v];
      if (heap[i].first == value) {
        return;
      }

      if (comp(heap[i].first, value)) {
        heap[i].first = value;
        siftDown(i);
      } else {
        heap[i].first = value;
        siftUp(i);
      }
    }
  }

  std::pair<uint32_t, Vertex> pop() {
    if (empty()) {
      return {0, noVertex};
    }

    auto result = heap[0];
    swap(0, size() - 1);
    heap.pop_back();
    mapper[result.second] = noIndex;

    if (!empty()) {
      siftDown(0);
    }

    assert(heapPropertyFullfilled());
    return result;
  }

  const HeapElement &front() const {
    assert(!empty());
    return heap.front();
  }

  void buildFrom(std::vector<uint32_t> &values) {
    mapper.clear();
    heap.clear();

    if (values.size() == 0) return;

    mapper.resize(values.size());
    heap.resize(values.size());

    for (Vertex v = 0; v < values.size(); ++v) {
      heap[v] = std::make_pair(values[v], v);
      mapper[v] = v;
    }
    buildHeapRecursive(0);
    assert(heapPropertyFullfilled());
  }

  void buildHeapRecursive(Index i) {
    if (2 * i + 1 < size()) {
      buildHeapRecursive(getLeftChild(i));
    }
    if (2 * i + 2 < size()) {
      buildHeapRecursive(getRightChild(i));
    }

    siftDown(i);
  }

  void reset() {
    std::fill(mapper.begin(), mapper.end(), noIndex);
    heap.clear();
    heap.reserve(capacity());
  }
};
