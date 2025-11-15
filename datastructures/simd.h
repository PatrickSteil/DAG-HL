#pragma once

#include <immintrin.h>

#include <cstdint>
#include <iomanip>
#include <iostream>
#include <type_traits>

// Traits for register type and array size
template <size_t REG_BITS>
struct SIMDTraits;

template <>
struct SIMDTraits<128> {
  using reg_type = __m128i;
  static constexpr size_t BYTES = 16;
};

template <>
struct SIMDTraits<256> {
  using reg_type = __m256i;
  static constexpr size_t BYTES = 32;
};

template <>
struct SIMDTraits<512> {
  using reg_type = __m512i;
  static constexpr size_t BYTES = 64;
};

template <size_t REG_BITS>
struct SIMDContainer4bit {
  static_assert(REG_BITS == 128 || REG_BITS == 256 || REG_BITS == 512,
                "Only 128/256/512-bit supported");

  using traits = SIMDTraits<REG_BITS>;
  using reg_type = typename traits::reg_type;
  static constexpr size_t BYTES = traits::BYTES;
  static constexpr size_t NIBBLES = BYTES * 2;

  union Holder {
    reg_type reg;
    uint8_t arr[BYTES];
  } v;

  SIMDContainer4bit() noexcept = default;

  explicit SIMDContainer4bit(uint8_t scalar4bit) noexcept {
    scalar4bit &= 0x0F;
    uint8_t byte = (scalar4bit << 4) | scalar4bit;
    fill(byte);
  }

  void fill(uint8_t byte) noexcept {
    if constexpr (REG_BITS == 128)
      v.reg = _mm_set1_epi8(byte);
    else if constexpr (REG_BITS == 256)
      v.reg = _mm256_set1_epi8(byte);
    else
      v.reg = _mm512_set1_epi8(byte);
  }

  void reset() noexcept { fill(15); }

  uint8_t get(size_t i) const noexcept {
    uint8_t byte = v.arr[i / 2];
    return (i % 2 == 0) ? (byte & 0x0F) : (byte >> 4);
  }

  void set(size_t i, uint8_t val) noexcept {
    val &= 0x0F;
    uint8_t &byte = v.arr[i / 2];
    if (i % 2 == 0)
      byte = (byte & 0xF0) | val;
    else
      byte = (byte & 0x0F) | (val << 4);
  }

  // Component-wise min
  void min_with(const SIMDContainer4bit &other) noexcept {
    auto mask_lo = create_mask();
    auto a_lo = and_reg(v.reg, mask_lo);
    auto a_hi = and_reg(shr4(v.reg), mask_lo);

    auto b_lo = and_reg(other.v.reg, mask_lo);
    auto b_hi = and_reg(shr4(other.v.reg), mask_lo);

    auto r_lo = min_epu8(a_lo, b_lo);
    auto r_hi = min_epu8(a_hi, b_hi);

    v.reg = or_reg(r_lo, shl4(r_hi));
  }

  // Component-wise max
  void max_with(const SIMDContainer4bit &other) noexcept {
    auto mask_lo = create_mask();
    auto a_lo = and_reg(v.reg, mask_lo);
    auto a_hi = and_reg(shr4(v.reg), mask_lo);

    auto b_lo = and_reg(other.v.reg, mask_lo);
    auto b_hi = and_reg(shr4(other.v.reg), mask_lo);

    auto r_lo = max_epu8(a_lo, b_lo);
    auto r_hi = max_epu8(a_hi, b_hi);

    v.reg = or_reg(r_lo, shl4(r_hi));
  }

  void increment_clamp(const SIMDContainer4bit &B,
                       const uint8_t distance = 1) noexcept {
    auto mask_lo = create_mask();

    auto b_lo = and_reg(B.v.reg, mask_lo);
    auto b_hi = and_reg(shr4(B.v.reg), mask_lo);

    if constexpr (REG_BITS == 128) {
      b_lo = _mm_min_epu8(_mm_add_epi8(b_lo, _mm_set1_epi8(distance)),
                          _mm_set1_epi8(15));
      b_hi = _mm_min_epu8(_mm_add_epi8(b_hi, _mm_set1_epi8(distance)),
                          _mm_set1_epi8(15));
    } else if constexpr (REG_BITS == 256) {
      b_lo = _mm256_min_epu8(_mm256_add_epi8(b_lo, _mm256_set1_epi8(distance)),
                             _mm256_set1_epi8(15));
      b_hi = _mm256_min_epu8(_mm256_add_epi8(b_hi, _mm256_set1_epi8(distance)),
                             _mm256_set1_epi8(15));
    } else {
      b_lo = _mm512_min_epu8(_mm512_add_epi8(b_lo, _mm512_set1_epi8(distance)),
                             _mm512_set1_epi8(15));
      b_hi = _mm512_min_epu8(_mm512_add_epi8(b_hi, _mm512_set1_epi8(distance)),
                             _mm512_set1_epi8(15));
    }

    v.reg = or_reg(b_lo, shl4(b_hi));
  }

 private:
  // Helpers
  static reg_type create_mask() noexcept {
    if constexpr (REG_BITS == 128)
      return _mm_set1_epi8(0x0F);
    else if constexpr (REG_BITS == 256)
      return _mm256_set1_epi8(0x0F);
    else
      return _mm512_set1_epi8(0x0F);
  }

  static reg_type and_reg(reg_type a, reg_type b) noexcept {
    if constexpr (REG_BITS == 128)
      return _mm_and_si128(a, b);
    else if constexpr (REG_BITS == 256)
      return _mm256_and_si256(a, b);
    else
      return _mm512_and_si512(a, b);
  }

  static reg_type or_reg(reg_type a, reg_type b) noexcept {
    if constexpr (REG_BITS == 128)
      return _mm_or_si128(a, b);
    else if constexpr (REG_BITS == 256)
      return _mm256_or_si256(a, b);
    else
      return _mm512_or_si512(a, b);
  }

  static reg_type min_epu8(reg_type a, reg_type b) noexcept {
    if constexpr (REG_BITS == 128)
      return _mm_min_epu8(a, b);
    else if constexpr (REG_BITS == 256)
      return _mm256_min_epu8(a, b);
    else
      return _mm512_min_epu8(a, b);
  }

  static reg_type max_epu8(reg_type a, reg_type b) noexcept {
    if constexpr (REG_BITS == 128)
      return _mm_max_epu8(a, b);
    else if constexpr (REG_BITS == 256)
      return _mm256_max_epu8(a, b);
    else
      return _mm512_max_epu8(a, b);
  }

  static reg_type shl4(reg_type a) noexcept {
    if constexpr (REG_BITS == 128)
      return _mm_slli_epi16(a, 4);
    else if constexpr (REG_BITS == 256)
      return _mm256_slli_epi16(a, 4);
    else
      return _mm512_slli_epi16(a, 4);
  }

  static reg_type shr4(reg_type a) noexcept {
    if constexpr (REG_BITS == 128)
      return _mm_srli_epi16(a, 4);
    else if constexpr (REG_BITS == 256)
      return _mm256_srli_epi16(a, 4);
    else
      return _mm512_srli_epi16(a, 4);
  }
};

// Print function
template <size_t REG_BITS>
inline void printSIMDContainer4bit(const char *name,
                                   const SIMDContainer4bit<REG_BITS> &x) {
  std::cout << std::setw(12) << name << ": [";
  for (size_t i = 0; i < SIMDContainer4bit<REG_BITS>::NIBBLES; ++i) {
    std::cout << int(x.get(i))
              << (i + 1 < SIMDContainer4bit<REG_BITS>::NIBBLES ? ", " : "");
  }
  std::cout << "]\n";
}
