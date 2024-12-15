#pragma once

#include <omp.h>

#include <algorithm>
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
