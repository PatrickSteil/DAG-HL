/*
 * Licensed under MIT License.
 * Author: Patrick Steil
*/

#include "../datastructures/utils.h"

#include <cstdint>
#include <gtest/gtest.h>
#include <numeric>
#include <vector>

class ParallelFillTest : public ::testing::Test {
protected:
  template <typename T>
  void verify_vector(const std::vector<T> &v, const T &expected_value) {
    EXPECT_TRUE(std::all_of(v.begin(), v.end(), [&expected_value](const T &x) {
      return x == expected_value;
    })) << "Vector elements do not match the expected value.";
  }
};

TEST_F(ParallelFillTest, LargeVectorFill) {
  std::vector<int> v(1000000);
  int fill_value = 42;

  parallel_fill(v, fill_value);

  verify_vector(v, fill_value);
}

TEST_F(ParallelFillTest, EmptyVector) {
  std::vector<int> v;
  int fill_value = 0;

  parallel_fill(v, fill_value);

  EXPECT_TRUE(v.empty()) << "Empty vector test failed.";
}

TEST_F(ParallelFillTest, SingleElement) {
  std::vector<int> v(1);
  int fill_value = 99;

  parallel_fill(v, fill_value);

  verify_vector(v, fill_value);
}

TEST_F(ParallelFillTest, DoubleVector) {
  std::vector<double> v(1000);
  double fill_value = 3.14159;

  parallel_fill(v, fill_value);

  verify_vector(v, fill_value);
}

TEST_F(ParallelFillTest, OddSizeVector) {
  std::vector<int> v(1001);
  int fill_value = -1;

  parallel_fill(v, fill_value);

  verify_vector(v, fill_value);
}

class ParallelIotaTest : public ::testing::Test {
protected:
  template <typename T>
  void verify_iota(const std::vector<T> &v, const T &start_value) {
    T value = start_value;
    for (const auto &elem : v) {
      EXPECT_EQ(elem, value++)
          << "Vector element does not match expected value.";
    }
  }
};

TEST_F(ParallelIotaTest, LargeVectorIota) {
  std::vector<int> v(1000000);
  int start_value = 42;

  parallel_iota(v, start_value);

  verify_iota(v, start_value);
}

TEST_F(ParallelIotaTest, EmptyVector) {
  std::vector<int> v;
  int start_value = 0;

  parallel_iota(v, start_value);

  EXPECT_TRUE(v.empty()) << "Empty vector test failed.";
}

TEST_F(ParallelIotaTest, SingleElement) {
  std::vector<int> v(1);
  int start_value = 99;

  parallel_iota(v, start_value);

  verify_iota(v, start_value);
}

TEST_F(ParallelIotaTest, DoubleVector) {
  std::vector<double> v(1000);
  double start_value = 3.14159;

  parallel_iota(v, start_value);

  verify_iota(v, start_value);
}

TEST_F(ParallelIotaTest, OddSizeVector) {
  std::vector<int> v(1001);
  int start_value = -1;

  parallel_iota(v, start_value);

  verify_iota(v, start_value);
}
