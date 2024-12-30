/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <omp.h>

#include <algorithm>
#include <concepts>
#include <iostream>
#include <numeric>
#include <random>
#include <span>
#include <vector>

template <typename T>
void parallel_fill(std::vector<T> &v, const T &value) {
#pragma omp parallel
  {
    auto tid = omp_get_thread_num();
    auto num_threads = omp_get_num_threads();

    auto chunksize = v.size() / num_threads;

    auto begin = v.begin() + chunksize * tid;
    auto end = (tid == num_threads - 1) ? v.end() : begin + chunksize;

    std::fill(begin, end, value);
  }
}

template <typename T>
void parallel_assign(std::vector<T> &v, std::size_t n, const T &value) {
  v.resize(n);
  parallel_fill(v, value);
}

template <typename T>
void parallel_iota(std::vector<T> &v, T start_value) {
#pragma omp parallel
  {
    auto tid = omp_get_thread_num();
    auto num_threads = omp_get_num_threads();

    auto chunksize = v.size() / num_threads;

    auto begin = v.begin() + chunksize * tid;
    auto end = (tid == num_threads - 1) ? v.end() : begin + chunksize;

    T value = start_value + (begin - v.begin());
    for (auto it = begin; it != end; ++it) {
      *it = value++;
    }
  }
}

template <typename T>
void parallel_assign_iota(std::vector<T> &v, std::size_t n, T start_value) {
  v.resize(n);
  parallel_iota(v, start_value);
}

template <std::integral VALUE>
std::vector<std::pair<VALUE, VALUE>> generateRandomQueries(int numQueries,
                                                           int minVertex,
                                                           int maxVertex) {
  std::vector<std::pair<VALUE, VALUE>> queries;
  std::srand(42);

  for (int i = 0; i < numQueries; ++i) {
    VALUE source = minVertex + std::rand() % (maxVertex - minVertex + 1);
    VALUE target = minVertex + std::rand() % (maxVertex - minVertex + 1);

    while (source == target) {
      target = minVertex + std::rand() % (maxVertex - minVertex + 1);
    }

    queries.emplace_back(source, target);
  }

  return queries;
}

template <std::integral ELEMENT_TYPE = uint32_t>
class Drawer {
  std::vector<ELEMENT_TYPE> elements;
  std::vector<std::size_t> index;
  std::size_t left;
  std::mt19937 gen;

 public:
  Drawer(const std::size_t size = 0) : elements(), index(), left(0), gen(42) {
    elements.reserve(size);
    index.reserve(size);
  }

  inline bool isEmpty() const { return left == elements.size(); }

  inline std::size_t size() const { return elements.size() - left; }

  void reset() { left = 0; }

  void init(std::span<const ELEMENT_TYPE> newElements) {
    elements.assign(newElements.begin(), newElements.end());
    if (index.size() != elements.size()) {
      index.resize(elements.size());
    }
    std::iota(index.begin(), index.end(), 0);
    left = 0;
  }

  void swap(std::size_t a, std::size_t b) {
    if (a == b) return;
    std::swap(elements[a], elements[b]);
    index[elements[a]] = a;
    index[elements[b]] = b;
  }

  void remove(ELEMENT_TYPE element) {
    assert(element < index.size() && "Element out of bounds");
    auto i = index[element];
    swap(left, i);
    left += 1;
  }

  ELEMENT_TYPE pickRandom() {
    assert(left < elements.size() && "No elements left to pick");

    std::uniform_int_distribution<std::size_t> dist(left, elements.size() - 1);

    std::size_t i = dist(gen);
    swap(left, i);
    left += 1;
    return elements[left - 1];
  }
};

constexpr uint32_t INF = std::numeric_limits<uint32_t>::max();

inline int branchlessConditional(const bool predicate, const int ifTrue,
                                 const int ifFalse) noexcept {
  int result = 0;
  __asm__ __volatile__(
      "mov    %[ifTrue], %[result]\n\t"
      "test   %[predicate], %[predicate]\n\t"
      "cmovz  %[ifFalse], %[result]\n\t"
      : [result] "=&r"(result)
      : [predicate] "r"(predicate), [ifTrue] "r"(ifTrue), [ifFalse] "r"(ifFalse)
      : "cc");
  return result;
}
