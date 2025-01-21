/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>

#include "../external/emhash/hash_table5.hpp"

class Drawer {
 public:
  explicit Drawer(std::size_t n, int seed = 42) : n(n), rng(seed) { reset(); }

  void reset() {
    numbers.resize(n);
    index_map.clear();
    index_map.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      numbers[i] = i;
      index_map.emplace_unique(i, i);
    }
  }

  std::size_t size() const { return numbers.size(); }

  bool hasNext() const { return !numbers.empty(); }

  std::size_t draw() {
    if (numbers.empty()) {
      return static_cast<std::size_t>(-1);
    }

    std::size_t index =
        std::uniform_int_distribution<std::size_t>(0, numbers.size() - 1)(rng);
    std::size_t drawn_number = numbers[index];

    assert(index_map.contains(drawn_number));

    remove(drawn_number);
    return drawn_number;
  }

  void add(std::size_t number) {
    if (index_map.contains(number)) return;
    index_map.insert_unique(number, numbers.size());
    numbers.push_back(number);
  }

  bool remove(std::size_t number) {
    if (!index_map.contains(number)) return false;

    std::size_t index = index_map[number];
    std::size_t last_number = numbers.back();

    std::swap(numbers[index], numbers.back());
    numbers.pop_back();

    index_map[last_number] = index;
    index_map.erase(number);

    return true;
  }

 private:
  std::size_t n;
  std::vector<std::uint32_t> numbers;
  emhash5::HashMap<std::uint32_t, std::uint32_t> index_map;
  std::mt19937 rng;
};
