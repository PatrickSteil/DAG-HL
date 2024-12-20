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
