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

    if (read == write) [[unlikely]] {
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

  mutable std::mutex mutex_read;
  mutable std::mutex mutex_write;
  std::vector<VertexType> data;
  std::size_t read;
  std::size_t write;
};

// From exersice 4
template <class T>
class ConcurrentQueue {
  static_assert(std::is_default_constructible<T>::value,
                "T must be default constructible.");

 public:
  explicit ConcurrentQueue(std::size_t capacity)
      : capacity_(capacity),
        data_(std::make_unique<AlignedData<T>[]>(capacity)),
        read_(0),
        write_(0) {}

  bool try_push(const T &value) {
    assert(value != T{});

    std::size_t write_index = write_.load(std::memory_order_relaxed);
    std::size_t next_write_index = (write_index + 1) % capacity_;

    if (next_write_index == read_.load(std::memory_order_acquire)) {
      return false;
    }

    data_[write_index]().store(value, std::memory_order_release);
    write_.store(next_write_index, std::memory_order_release);
    return true;
  }

  bool try_pop(T &value) {
    std::size_t read_index = read_.load(std::memory_order_relaxed);

    if (read_index == write_.load(std::memory_order_acquire)) {
      return false;
    }

    value = data_[read_index]().load(std::memory_order_acquire);
    data_[read_index]().store(T{}, std::memory_order_release);
    read_.store((read_index + 1) % capacity_, std::memory_order_release);
    return true;
  }

 private:
  template <typename L>
  struct alignas(64) AlignedData {
    std::atomic<L> data;

    std::atomic<L> &operator()() { return data; }

    const std::atomic<L> &operator()() const { return data; }
  };

  const std::size_t capacity_;
  std::unique_ptr<AlignedData<T>[]> data_;
  std::atomic<std::size_t> read_;
  std::atomic<std::size_t> write_;
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

  bool tryMark(std::size_t i) {
    std::lock_guard<std::mutex> lock(mutex);
    assert(isValid(i));

    bool overwrite = (seen[i] != generation);
    seen[i] = (overwrite ? generation : seen[i]);
    return overwrite;
  }

 private:
  mutable std::mutex mutex;
  std::vector<GenerationType> seen;
  GenerationType generation;
};
}  // namespace bfs
