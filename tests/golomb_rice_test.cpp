#include "../datastructures/golomb_rice.h"

#include <gtest/gtest.h>

#include <random>
#include <vector>

TEST(GolombRiceTest, EncodingDecoding) {
  std::vector<uint32_t> values = {3, 7, 15, 31, 63};
  GolombRice<uint32_t, uint8_t> coder(4);  // M = 4

  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_EQ(values, decoded);
}

TEST(GolombRiceTest, EmptyVector) {
  std::vector<uint32_t> values = {};
  GolombRice<uint32_t, uint8_t> coder(4);

  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_TRUE(decoded.empty());
}

TEST(GolombRiceTest, SingleValue) {
  std::vector<uint32_t> values = {42};
  GolombRice<uint32_t, uint8_t> coder(8);  // M = 8

  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_EQ(values, decoded);
}

TEST(GolombRiceTest, LargeSequence) {
  std::vector<uint32_t> values;
  for (uint32_t i = 0; i < 1000; ++i) {
    values.push_back(i * 3 + 7);  // Some arbitrary pattern
  }

  GolombRice<uint32_t, uint8_t> coder(8);  // M = 8
  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_EQ(values, decoded);
}

TEST(GolombRiceTest, LargeRange) {
  std::vector<uint32_t> values(10000);
  for (uint32_t i = 0; i < 10000; ++i) {
    values[i] = i;
  }

  GolombRice<uint32_t, uint8_t> coder(16);  // M = 16
  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_EQ(values, decoded);
}

TEST(GolombRiceTest, RandomValues) {
  std::vector<uint32_t> values(5000);
  std::mt19937 rng(42);  // Fixed seed for reproducibility
  std::uniform_int_distribution<uint32_t> dist(0, 1000000);

  for (uint32_t& val : values) {
    val = dist(rng);
  }

  GolombRice<uint32_t, uint8_t> coder(64);  // M = 64
  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_EQ(values, decoded);
}

TEST(GolombRiceTest, AllZeroes) {
  std::vector<uint32_t> values(1000, 0);  // 1000 zeros

  GolombRice<uint32_t, uint8_t> coder(4);
  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_EQ(values, decoded);
}

TEST(GolombRiceTest, PowersOfTwo) {
  std::vector<uint32_t> values;
  for (uint32_t i = 0; i < 1000; ++i) {
    values.push_back(1 << (i % 16));  // Powers of 2 up to 2^15
  }

  GolombRice<uint32_t, uint8_t> coder(32);  // M = 32
  BitVector<uint8_t> encoded = coder.encode(values);
  std::vector<uint32_t> decoded = coder.decode(encoded);

  EXPECT_EQ(values, decoded);
}
