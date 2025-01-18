/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <random>
#include <vector>

/*
 * Simple class to pick random numbers one by one
 */
class Drawer {
 public:
  // Constructor initializes the drawer with n elements and a random number
  // generator
  explicit Drawer(std::size_t n, const int seed = 42) : n(n), rng(seed) {
    reset();
  }

  // Resets the drawer, filling it with numbers from 0 to n-1
  void reset() {
    numbers.resize(n);
    std::iota(numbers.begin(), numbers.end(), 0);
  }

  // Returns the current number of elements in the drawer
  std::size_t size() const { return numbers.size(); }

  // Checks if there are numbers left to draw
  bool hasNext() const { return !numbers.empty(); }

  // Draws a random number from the drawer and removes it
  std::size_t draw() {
    if (numbers.empty()) {
      return static_cast<std::size_t>(-1);
    }

    std::size_t index =
        std::uniform_int_distribution<std::size_t>(0, numbers.size() - 1)(rng);
    std::size_t drawn_number = numbers[index];

    // Swap with last element and remove last element to maintain O(1) removal
    std::swap(numbers[index], numbers.back());
    numbers.pop_back();

    return drawn_number;
  }

  // Adds a number back to the drawer even if it's already present
  void add(std::size_t number) { numbers.push_back(number); }

 private:
  std::size_t n;
  std::vector<std::size_t> numbers;
  std::mt19937 rng;
};
