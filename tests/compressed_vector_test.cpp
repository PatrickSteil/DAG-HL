#include "../datastructures/compressed_vector.h"

#include <gtest/gtest.h>

#include <vector>

// Test default constructor
TEST(CompressedVectorTest, DefaultConstructor) {
  CompressedVector cv;
  EXPECT_EQ(cv.size(), 0);
  EXPECT_TRUE(cv.empty());
}

// Test constructor with initial data
TEST(CompressedVectorTest, ConstructorWithData) {
  std::vector<std::uint32_t> numbers = {1, 2, 3, 128, 300};
  CompressedVector cv(numbers);
  EXPECT_EQ(cv.size(), numbers.size());
  EXPECT_FALSE(cv.empty());
}

// Test push_back method
TEST(CompressedVectorTest, PushBack) {
  CompressedVector cv;
  cv.push_back(42);
  cv.push_back(128);
  cv.push_back(300);

  EXPECT_EQ(cv.size(), 3);
  EXPECT_FALSE(cv.empty());
}

// Test iterator functionality
TEST(CompressedVectorTest, Iterator) {
  std::vector<std::uint32_t> numbers = {1, 2, 3, 128, 300};
  CompressedVector cv(numbers);

  std::vector<std::uint32_t> decoded_numbers;
  for (auto it = cv.begin(); it != cv.end(); ++it) {
    decoded_numbers.push_back(*it);
  }

  EXPECT_EQ(decoded_numbers, numbers);
}

// Test byteSize method
TEST(CompressedVectorTest, ByteSize) {
  std::vector<std::uint32_t> numbers = {1, 2, 3, 128, 300};
  CompressedVector cv(numbers);

  // Ensure byteSize is at least the size of the internal data vector
  EXPECT_GE(cv.byteSize(), cv.size());
}

// Test clear method
TEST(CompressedVectorTest, Clear) {
  std::vector<std::uint32_t> numbers = {1, 2, 3, 128, 300};
  CompressedVector cv(numbers);

  EXPECT_FALSE(cv.empty());
  cv.clear();
  EXPECT_TRUE(cv.empty());
  EXPECT_EQ(cv.size(), 0);
}

// Test reserve method
TEST(CompressedVectorTest, Reserve) {
  CompressedVector cv;
  cv.reserve(100);

  // Ensure the internal data vector has at least the reserved capacity
  EXPECT_GE(cv.byteSize(), 100);
}

// Test edge case: empty vector
TEST(CompressedVectorTest, EmptyVector) {
  std::vector<std::uint32_t> numbers;
  CompressedVector cv(numbers);

  EXPECT_EQ(cv.size(), 0);
  EXPECT_TRUE(cv.empty());

  auto it = cv.begin();
  EXPECT_EQ(it, cv.end());
}

// Test edge case: single element
TEST(CompressedVectorTest, SingleElement) {
  std::vector<std::uint32_t> numbers = {42};
  CompressedVector cv(numbers);

  EXPECT_EQ(cv.size(), 1);
  EXPECT_FALSE(cv.empty());

  auto it = cv.begin();
  EXPECT_NE(it, cv.end());
  EXPECT_EQ(*it, 42);
  ++it;
  EXPECT_EQ(it, cv.end());
}

// Test edge case: large numbers
TEST(CompressedVectorTest, LargeNumbers) {
  std::vector<std::uint32_t> numbers = {0xFFFFFFFF, 0x7FFFFFFF, 0x3FFFFFFF};
  CompressedVector cv(numbers);

  EXPECT_EQ(cv.size(), numbers.size());

  std::vector<std::uint32_t> decoded_numbers;
  for (auto it = cv.begin(); it != cv.end(); ++it) {
    decoded_numbers.push_back(*it);
  }

  EXPECT_EQ(decoded_numbers, numbers);
}
