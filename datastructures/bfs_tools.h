/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <atomic>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "utils.h"

namespace bfs {
template <typename VertexType = std::uint64_t> class FixedSizedQueue {
public:
  explicit FixedSizedQueue(std::size_t size = 0)
      : data(size), read(0), write(0) {}

  void resize(std::size_t size) {
    parallel_assign(data, size, VertexType(0));
    read = 0;
    write = 0;
  }

  void push(VertexType v) {
    assert(write < data.size());
    data[write++] = v;
  }

  VertexType pop() {
    assert(read < write);
    return data[read++];
  }

  bool isEmpty() const { return read == write; }

  void reset() { read = write = 0; }

  std::vector<VertexType> data;
  std::size_t read;
  std::size_t write;
};

template <typename VertexType = std::uint64_t> class FixedSizedQueueThreadSafe {
public:
  explicit FixedSizedQueueThreadSafe(std::size_t size = 0)
      : data(size), read(0), write(0) {}

  // Resize the queue safely (not thread-safe in this version)
  void resize(std::size_t size) {
    data.resize(size);
    read.store(0, std::memory_order_relaxed);
    write.store(0, std::memory_order_relaxed);
  }

  void push(VertexType v) {
    auto index = write.fetch_add(1, std::memory_order_relaxed);
    assert(index < data.size()); // Ensure no overflows
    data[index] = v;
  }

  VertexType pop() {
    auto current_read = read.load(std::memory_order_relaxed);
    if (current_read >= write.load(std::memory_order_acquire)) {
      return static_cast<VertexType>(-1); // Queue empty
    }
    auto index = read.fetch_add(1, std::memory_order_relaxed);
    assert(index < data.size()); // Ensure valid access
    return data[index];
  }

  bool isEmpty() const {
    return read.load(std::memory_order_acquire) ==
           write.load(std::memory_order_acquire);
  }

  void reset() {
    read.store(0, std::memory_order_relaxed);
    write.store(0, std::memory_order_relaxed);
  }

  std::vector<VertexType> data;
  std::atomic_size_t read;
  std::atomic_size_t write;
};

// class that handles whether we have already seen a vertex
template <std::integral GenerationType = std::uint16_t>
class GenerationChecker {
public:
  explicit GenerationChecker(std::size_t size = 0)
      : seen(size, 0), generation(1) {}

  void resize(std::size_t size) {
    parallel_assign(seen, size, static_cast<GenerationType>(0));
    generation = 1;
  }

  void reset() {
    ++generation;
    if (generation == 0) {
      std::fill(seen.begin(), seen.end(), 0);
      ++generation;
    }
  }

  inline bool isValid(std::size_t i) const { return i < seen.size(); }

  inline bool isMarked(std::size_t i) const {
    assert(isValid(i));
    return seen[i] == generation;
  }

  inline void mark(std::size_t i) {
    assert(isValid(i));
    seen[i] = generation;
  }

  bool firstOccur(std::size_t i) {
    bool seenBefore = (seen[i] == generation);
    seen[i] = generation;
    return !seenBefore;
  }

  std::vector<GenerationType> seen;
  GenerationType generation;
};

template <std::integral GenerationType = std::uint16_t>
class GenerationCheckerThreadSafe {
public:
  explicit GenerationCheckerThreadSafe(std::size_t size = 0)
      : seen(size, 0), generation(1) {}

  void resize(std::size_t size) {
    std::lock_guard<std::mutex> lock(mutex);
    seen.resize(size, 0);
    generation = 1;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex);
    ++generation;
    if (generation == 0) {
      std::fill(seen.begin(), seen.end(), 0);
      generation = 1;
    }
  }

  bool isValid(std::size_t i) const { return i < seen.size(); }

  bool isMarked(std::size_t i) const {
    std::lock_guard<std::mutex> lock(mutex);
    assert(isValid(i));
    return seen[i] == generation;
  }

  void mark(std::size_t i) {
    std::lock_guard<std::mutex> lock(mutex);
    assert(isValid(i));
    seen[i] = generation;
  }

  bool firstOccur(std::size_t i) {
    std::lock_guard<std::mutex> lock(mutex);
    bool seenBefore = (seen[i] == generation);
    seen[i] = generation;
    return !seenBefore;
  }

private:
  mutable std::mutex mutex;
  std::vector<GenerationType> seen;
  GenerationType generation;
};
} // namespace bfs
