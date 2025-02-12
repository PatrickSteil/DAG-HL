#pragma once

#include <bit>
#include <cassert>
#include <concepts>
#include <vector>

#include "bit_vector.h"

template <typename T, typename BIT_TYPE>
  requires std::unsigned_integral<T> && std::unsigned_integral<BIT_TYPE>
class GolombRice {
 private:
  T M;

 public:
  explicit GolombRice(T M) : M(M) {
    assert((M & (M - 1)) == 0 && "M must be a power of 2");
  }

  BitVector<BIT_TYPE> encode(const std::vector<T>& values) const {
    BitVector<BIT_TYPE> bv;
    size_t logM = std::countr_zero(M);

    bv.reserve(values.size() * 4);

    for (T value : values) {
      T quotient = value / M;
      T remainder = value % M;

      for (T i = 0; i < quotient; ++i) {
        bv.push_back(1);
      }
      bv.push_back(0);

      for (int i = logM - 1; i >= 0; --i) {
        bv.push_back((remainder >> i) & 1);
      }
    }
    return bv;
  }

  std::vector<T> decode(const BitVector<BIT_TYPE>& bv) const {
    std::vector<T> decodedValues;
    size_t logM = std::countr_zero(M);
    size_t i = 0;

    while (i < bv.size()) {
      T quotient = 0;
      while (i < bv.size() && bv[i] == 1) {
        ++quotient;
        ++i;
      }

      ++i;

      T remainder = 0;
      for (size_t j = 0; j < logM && i < bv.size(); ++j, ++i) {
        remainder = (remainder << 1) | bv[i];
      }

      decodedValues.push_back(quotient * M + remainder);
    }

    return decodedValues;
  }
};
