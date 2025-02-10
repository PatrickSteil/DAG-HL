#pragma once

#include <cstdint>
#include <iostream>
#include <iterator>
#include <vector>

/*
 * CompressedVector is a space-efficient data structure that stores a sequence
 * of non-negative integers using LEB128 encoding, reducing storage by only
 * using as many bytes as needed per number. It provides an iterator for
 * sequential access, decoding numbers on-the-fly while keeping track of the
 * total number of elements and memory usage.
 * https://en.wikipedia.org/wiki/LEB128
 */
class CompressedVector {
 private:
  std::vector<std::uint8_t> data;
  std::size_t num_elements = 0;

  // Encodes a 32-bit unsigned integer using LEB128 encoding and appends it to
  // the output vector.
  static void encodeLEB128(std::uint32_t value,
                           std::vector<std::uint8_t>& out) {
    while (value >= 0x80) {
      out.push_back(static_cast<std::uint8_t>((value & 0x7F) | 0x80));
      value >>= 7;
    }
    out.push_back(static_cast<std::uint8_t>(value));
  }

 public:
  // Constructs a CompressedVector from an existing vector of 32-bit unsigned
  // integers.
  explicit CompressedVector(const std::vector<std::uint32_t>& numbers) {
    reserve(numbers.size() * 3);
    for (std::uint32_t num : numbers) {
      push_back(num);
    }

    data.shrink_to_fit();
  }

  // Default constructor.
  CompressedVector() = default;

  // Adds a new element to the end of the CompressedVector.
  void push_back(std::uint32_t value) {
    encodeLEB128(value, data);
    ++num_elements;
  }

  // Returns the number of elements in the CompressedVector.
  std::size_t size() const { return num_elements; }

  // Returns the total memory usage of the CompressedVector in bytes.
  std::size_t byteSize() const { return sizeof(*this) + data.capacity(); }

  // Clears all elements from the CompressedVector.
  void clear() {
    data.clear();
    num_elements = 0;
  }

  // Checks if the CompressedVector is empty.
  bool empty() const { return num_elements == 0; }

  // Reserves storage for at least the specified number of bytes.
  void reserve(std::size_t new_cap) { data.reserve(new_cap); }

  // Iterator class for sequential access to the elements in the
  // CompressedVector.
  class Iterator {
   private:
    const std::uint8_t* ptr;
    const std::uint8_t* end;
    std::uint32_t current;
    bool is_end;

    // Decodes the next LEB128-encoded value from the data.
    void decodeNext() {
      if (ptr >= end) {
        is_end = true;
        return;
      }

      std::uint32_t result = 0;
      std::uint8_t shift = 0;
      std::uint8_t byte;

      do {
        byte = *ptr++;
        result |= static_cast<std::uint32_t>(byte & 0x7F) << shift;
        shift += 7;
      } while ((byte & 0x80) && ptr < end);

      current = result;
    }

   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = std::uint32_t;
    using difference_type = std::ptrdiff_t;
    using pointer = const std::uint32_t*;
    using reference = const std::uint32_t&;

    // Constructs an Iterator pointing to the start of the data.
    explicit Iterator(const std::uint8_t* start, const std::uint8_t* stop)
        : ptr(start), end(stop), current(0), is_end(false) {
      decodeNext();
    }

    // Dereference operator.
    std::uint32_t operator*() const { return current; }

    // Prefix increment operator.
    Iterator& operator++() {
      if (!is_end) {
        decodeNext();
      }
      return *this;
    }

    // Equality operator.
    bool operator==(const Iterator& other) const {
      // Two iterators are equal if they are both at the end or point to the
      // same location in the data.
      return is_end == other.is_end && ptr == other.ptr;
    }

    // Inequality operator.
    bool operator!=(const Iterator& other) const {
      return is_end != other.is_end || ptr != other.ptr;
    }
  };

  // Returns an iterator to the beginning of the CompressedVector.
  Iterator begin() const {
    return Iterator(data.data(), data.data() + data.size());
  }

  // Returns an iterator to the end of the CompressedVector.
  Iterator end() const {
    return Iterator(data.data() + data.size(), data.data() + data.size());
  }
};
