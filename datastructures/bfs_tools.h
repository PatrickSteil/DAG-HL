/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <cassert>
#include <concepts>
#include <cstdint>
#include <mutex>
#include <vector>

#include "utils.h"

namespace bfs {
template <typename VertexType = std::uint64_t>
class FixedSizedQueue {
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

template <typename VertexType = std::uint64_t>
class FixedSizedQueueThreadSafe {
 public:
  explicit FixedSizedQueueThreadSafe(std::size_t size = 0)
      : data(size), read(0), write(0) {}

  void resize(std::size_t size) {
    std::lock(mutex_read, mutex_write);

    std::lock_guard<std::mutex> lock_read(mutex_read, std::adopt_lock);
    std::lock_guard<std::mutex> lock_write(mutex_write, std::adopt_lock);

    data.resize(size);
    read = 0;
    write = 0;
  }

  void push(VertexType v) {
    std::lock_guard<std::mutex> lock(mutex_write);
    assert(write < data.size());
    data[write++] = v;
  }

  VertexType pop() {
    std::lock(mutex_read, mutex_write);
    std::lock_guard<std::mutex> lock_read(mutex_read, std::adopt_lock);
    std::lock_guard<std::mutex> lock_write(mutex_write, std::adopt_lock);

    if (read == write) {
      return static_cast<VertexType>(-1);
    }
    /* assert(read < write); */
    return data[read++];
  }

  bool isEmpty() const {
    std::lock(mutex_read, mutex_write);
    std::lock_guard<std::mutex> lock_read(mutex_read, std::adopt_lock);
    std::lock_guard<std::mutex> lock_write(mutex_write, std::adopt_lock);

    return read == write;
  }

  void reset() {
    std::lock(mutex_read, mutex_write);
    std::lock_guard<std::mutex> lock_read(mutex_read, std::adopt_lock);
    std::lock_guard<std::mutex> lock_write(mutex_write, std::adopt_lock);

    read = 0;
    write = 0;
  }

 private:
  mutable std::mutex mutex_read;
  mutable std::mutex mutex_write;
  std::vector<VertexType> data;
  std::size_t read;
  std::size_t write;
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

  std::vector<GenerationType> seen;
  GenerationType generation;
};

// thread safe variant
template <std::integral GenerationType = std::uint16_t>
class GenerationCheckerThreadSafe {
 public:
  explicit GenerationCheckerThreadSafe(std::size_t size = 0)
      : seen(size, 0), generation(1) {}

  void resize(std::size_t size) {
    std::lock_guard<std::mutex> lock(mutex);
    parallel_assign(seen, size, static_cast<GenerationType>(0));
    generation = 1;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex);
    ++generation;
    if (generation == 0) {
      std::fill(seen.begin(), seen.end(), 0);
      ++generation;
    }
  }

  bool isValid(std::size_t i) const {
    std::lock_guard<std::mutex> lock(mutex);
    return i < seen.size();
  }

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

 private:
  mutable std::mutex mutex;
  std::vector<GenerationType> seen;
  GenerationType generation;
};

}  // namespace bfs
