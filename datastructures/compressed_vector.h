/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#pragma once

#include <cstdint>
#include <iostream>
#include <iterator>
#include <utility>
#include <vector>

/*
 * CompressedVector is a space-efficient data structure that stores a sequence
 * of non-negative integers using LEB128 encoding. It optimizes memory usage by
 * encoding only as many bytes as needed per number.
 * https://en.wikipedia.org/wiki/LEB128
 */
class CompressedVector {
 private:
  std::vector<std::uint8_t> data;
  std::size_t num_elements = 0;

  // Encodes a 32-bit unsigned integer using LEB128 encoding.
  static void encodeLEB128(std::uint32_t value,
                           std::vector<std::uint8_t>& out) {
    do {
      out.push_back(static_cast<std::uint8_t>((value & 0x7F) |
                                              (value >= 0x80 ? 0x80 : 0)));
      value >>= 7;
    } while (value);
  }

 public:
  // Constructs a CompressedVector from an existing vector of 32-bit unsigned
  // integers.
  explicit CompressedVector(const std::vector<std::uint32_t>& numbers) {
    data.reserve(numbers.size() *
                 5);  // Preallocate assuming max LEB128 encoding size.
    for (std::uint32_t num : numbers) push_back(num);
  }

  // Default constructor.
  CompressedVector() = default;

  // Move constructor.
  CompressedVector(CompressedVector&& other) noexcept
      : data(std::move(other.data)), num_elements(other.num_elements) {
    other.num_elements = 0;
  }

  // Move assignment operator.
  CompressedVector& operator=(CompressedVector&& other) noexcept {
    if (this != &other) {
      data = std::move(other.data);
      num_elements = other.num_elements;
      other.num_elements = 0;
    }
    return *this;
  }

  // Copy constructor
  CompressedVector(const CompressedVector& other)
      : data(other.data), num_elements(other.num_elements) {}

  // Copy assignment operator
  CompressedVector& operator=(const CompressedVector& other) {
    if (this != &other) {
      data = other.data;
      num_elements = other.num_elements;
    }
    return *this;
  }

  // Adds a new element to the CompressedVector.
  void push_back(std::uint32_t value) {
    encodeLEB128(value, data);
    ++num_elements;
  }

  // Returns the number of stored elements.
  std::size_t size() const noexcept { return num_elements; }

  // Returns total memory usage (including object overhead).
  std::size_t byteSize() const noexcept { return sizeof(*this) + data.size(); }

  // Clears all elements.
  void clear() noexcept {
    data.clear();
    num_elements = 0;
  }

  // Checks if the vector is empty.
  bool empty() const noexcept { return num_elements == 0; }

  // Reserves storage space.
  void reserve(std::size_t new_cap) { data.reserve(new_cap); }

  // Swaps content with another CompressedVector.
  void swap(CompressedVector& other) noexcept {
    data.swap(other.data);
    std::swap(num_elements, other.num_elements);
  }

  // Returns a pointer to the underlying raw data.
  const std::uint8_t* raw_data() const noexcept { return data.data(); }

  // Iterator for sequential access to elements.
  class Iterator {
   private:
    const std::uint8_t* ptr;
    const std::uint8_t* end;
    std::uint32_t current;
    bool is_end;

    // Decodes the next LEB128-encoded value.
    constexpr void decodeNext() {
      if (ptr >= end) {
        is_end = true;
        return;
      }

      std::uint32_t result = 0;
      std::uint8_t shift = 0;
      while (ptr < end) {
        std::uint8_t byte = *ptr++;
        result |= (byte & 0x7F) << shift;
        shift += 7;
        if (!(byte & 0x80)) break;
      }
      current = result;
    }

   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = std::uint32_t;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::uint32_t*;
    using reference = const std::uint32_t&;

    // Constructs an iterator.
    explicit Iterator(const std::uint8_t* start, const std::uint8_t* stop)
        : ptr(start), end(stop), current(0), is_end(false) {
      decodeNext();
    }

    // Dereference operator.
    std::uint32_t operator*() const { return current; }

    // Prefix increment operator.
    Iterator& operator++() {
      if (!is_end) decodeNext();
      return *this;
    }

    // Equality operator.
    bool operator==(const Iterator& other) const {
      return is_end == other.is_end && ptr == other.ptr;
    }

    // Inequality operator.
    bool operator!=(const Iterator& other) const { return !(*this == other); }
  };

  // Returns an iterator to the first element.
  Iterator begin() const noexcept {
    return Iterator(data.data(), data.data() + data.size());
  }

  // Returns an iterator to the end.
  Iterator end() const noexcept {
    return Iterator(data.data() + data.size(), data.data() + data.size());
  }
};

// Swaps two CompressedVector objects.
inline void swap(CompressedVector& lhs, CompressedVector& rhs) noexcept {
  lhs.swap(rhs);
}
