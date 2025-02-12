#pragma once

#include <cassert>
#include <concepts>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <vector>

template <typename T>
concept UnsignedIntegral = std::is_unsigned_v<T>;

template <UnsignedIntegral TYPE = uint64_t>
class BitVector {
 private:
  std::vector<TYPE> data;
  size_t size_ = 0;

  static constexpr size_t BITS_PER_WORD = sizeof(TYPE) * 8;
  static size_t word_index(size_t bit) { return bit / BITS_PER_WORD; }
  static size_t bit_index(size_t bit) { return bit % BITS_PER_WORD; }

 public:
  BitVector() = default;

  explicit BitVector(const std::vector<bool>& bits) {
    reserve(bits.size());
    for (bool bit : bits) {
      push_back(bit);
    }
  }

  BitVector(const BitVector& other) : data(other.data), size_(other.size_) {}

  BitVector& operator=(const BitVector& other) {
    if (this != &other) {
      data = other.data;
      size_ = other.size_;
    }
    return *this;
  }

  BitVector(BitVector&& other) noexcept
      : data(std::move(other.data)), size_(other.size_) {
    other.size_ = 0;
  }

  BitVector& operator=(BitVector&& other) noexcept {
    if (this != &other) {
      data = std::move(other.data);
      size_ = other.size_;
      other.size_ = 0;
    }
    return *this;
  }

  void push_back(bool value) {
    if (size_ % BITS_PER_WORD == 0) {
      data.push_back(0);
    }
    data[word_index(size_)] |= (TYPE(value) << bit_index(size_));
    ++size_;
  }

  bool operator[](size_t index) const {
    assert(index < size_);
    return (data[word_index(index)] >> bit_index(index)) & 1;
  }

  void clear() {
    data.clear();
    size_ = 0;
  }

  void reserve(size_t bits) {
    data.reserve((bits + BITS_PER_WORD - 1) / BITS_PER_WORD);
  }

  size_t size() const { return size_; }

  size_t capacity() const { return data.capacity() * BITS_PER_WORD; }

  size_t byteSize() const {
    return sizeof(BitVector) + (data.capacity() * sizeof(TYPE));
  }

  bool serialize(const std::string& filename) const {
    std::ofstream file(filename, std::ios::binary);
    if (!file) return false;

    file.write(reinterpret_cast<const char*>(&size_), sizeof(size_));
    size_t data_size = data.size();
    file.write(reinterpret_cast<const char*>(&data_size), sizeof(data_size));
    file.write(reinterpret_cast<const char*>(data.data()),
               data.size() * sizeof(TYPE));

    return file.good();
  }

  bool deserialize(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) return false;

    file.read(reinterpret_cast<char*>(&size_), sizeof(size_));
    size_t data_size;
    file.read(reinterpret_cast<char*>(&data_size), sizeof(data_size));

    data.resize(data_size);
    file.read(reinterpret_cast<char*>(data.data()), data_size * sizeof(TYPE));

    return file.good();
  }

  class Iterator {
   private:
    const BitVector* bv;
    size_t index;

   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = bool;
    using difference_type = std::ptrdiff_t;
    using pointer = void;
    using reference = bool;

    Iterator(const BitVector* bv, size_t index) : bv(bv), index(index) {}

    bool operator*() const { return (*bv)[index]; }
    Iterator& operator++() {
      ++index;
      return *this;
    }
    Iterator operator++(int) {
      Iterator temp = *this;
      ++(*this);
      return temp;
    }
    bool operator==(const Iterator& other) const {
      return index == other.index;
    }
    bool operator!=(const Iterator& other) const { return !(*this == other); }
  };

  Iterator begin() const { return Iterator(this, 0); }
  Iterator end() const { return Iterator(this, size_); }
};
