/*
 * Licensed under MIT License.
 * Author: Patrick Steil
 */

#include "../datastructures/priority_queue.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <numeric>
#include <vector>

TEST(PriorityQueueTest, LargeInsertionsMinQueue) {
  PriorityQueue<> minQueue(1000);

  EXPECT_EQ(minQueue.capacity(), 1000);
  EXPECT_TRUE(minQueue.empty());

  for (int i = 0; i < 1000; ++i) {
    minQueue.push(i, 1000 - i);
  }

  EXPECT_EQ(minQueue.size(), 1000);
  EXPECT_FALSE(minQueue.empty());

  EXPECT_EQ(minQueue.front().first, 1);
  EXPECT_EQ(minQueue.front().second, 999);

  for (int i = 0; i < 1000; ++i) {
    auto result = minQueue.pop();
    EXPECT_EQ(result.first, i + 1);
    EXPECT_EQ(result.second, 1000 - (i + 1));
  }

  EXPECT_TRUE(minQueue.empty());
}

TEST(PriorityQueueTest, LargeInsertionsMaxQueue) {
  PriorityQueue<std::greater<uint32_t>> maxQueue(1000);

  EXPECT_EQ(maxQueue.capacity(), 1000);
  EXPECT_TRUE(maxQueue.empty());

  for (int i = 0; i < 1000; ++i) {
    maxQueue.push(i, i + 1);
  }

  EXPECT_EQ(maxQueue.size(), 1000);
  EXPECT_FALSE(maxQueue.empty());

  EXPECT_EQ(maxQueue.front().first, 1000);
  EXPECT_EQ(maxQueue.front().second, 999);

  for (int i = 0; i < 1000; ++i) {
    auto result = maxQueue.pop();
    EXPECT_EQ(result.first, 1000 - i);
    EXPECT_EQ(result.second, 999 - i);
  }

  EXPECT_TRUE(maxQueue.empty());
}

TEST(PriorityQueueTest, PushAndUpdateMinQueue) {
  PriorityQueue<> minQueue(5);

  minQueue.push(0, 10);
  minQueue.push(1, 20);
  minQueue.push(2, 30);

  EXPECT_EQ(minQueue.size(), 3);
  EXPECT_EQ(minQueue.front().first, 10);
  EXPECT_EQ(minQueue.front().second, 0);

  minQueue.push(2, 5);
  EXPECT_EQ(minQueue.front().first, 5);
  EXPECT_EQ(minQueue.front().second, 2);

  minQueue.push(0, 15);
  EXPECT_EQ(minQueue.front().first, 5);
  EXPECT_EQ(minQueue.front().second, 2);

  auto result = minQueue.pop();
  EXPECT_EQ(result.first, 5);
  EXPECT_EQ(result.second, 2);
}

TEST(PriorityQueueTest, PushAndUpdateMaxQueue) {
  PriorityQueue<std::greater<uint32_t>> maxQueue(5);

  maxQueue.push(0, 10);
  maxQueue.push(1, 20);
  maxQueue.push(2, 30);

  EXPECT_EQ(maxQueue.size(), 3);
  EXPECT_EQ(maxQueue.front().first, 30);
  EXPECT_EQ(maxQueue.front().second, 2);

  maxQueue.push(2, 40);
  EXPECT_EQ(maxQueue.front().first, 40);
  EXPECT_EQ(maxQueue.front().second, 2);

  maxQueue.push(0, 25);
  EXPECT_EQ(maxQueue.front().first, 40);
  EXPECT_EQ(maxQueue.front().second, 2);

  auto result = maxQueue.pop();
  EXPECT_EQ(result.first, 40);
  EXPECT_EQ(result.second, 2);
}

TEST(PriorityQueueExternalTest, PushAndUpdateMaxQueue) {
  std::vector<std::uint32_t> values = {10, 20, 30};
  PriorityQueueExternal<std::uint32_t, std::greater<uint32_t>> maxQueue(
      values.size());
  maxQueue.buildFrom(values);

  EXPECT_EQ(maxQueue.size(), 3);
  EXPECT_EQ(maxQueue.front(), 2);

  values[2] = 40;
  maxQueue.update(2);
  EXPECT_EQ(maxQueue.front(), 2);

  values[2] = 5;
  maxQueue.update(2);
  EXPECT_EQ(maxQueue.front(), 1);

  auto result = maxQueue.pop();
  EXPECT_EQ(result, 1);
}

TEST(PriorityQueueExternalTest, OtherKeyFunction) {
  std::vector<std::uint32_t> values = {10, 20, 30};
  PriorityQueueExternal<std::uint32_t, MaxHeapComparator> maxQueue(
      values.size());
  maxQueue.buildFrom(values);

  EXPECT_EQ(maxQueue.size(), 3);
  EXPECT_EQ(maxQueue.front(), 0);

  values[2] = 4;
  maxQueue.update(2);
  EXPECT_EQ(maxQueue.front(), 2);

  values[2] = 50;
  maxQueue.update(2);
  EXPECT_EQ(maxQueue.front(), 0);

  auto result = maxQueue.pop();
  EXPECT_EQ(result, 0);
}

struct ArrayFunction {
  bool operator()(const std::array<std::uint32_t, 4>& left,
                  const std::array<std::uint32_t, 4>& right) const {
    return std::accumulate(std::begin(left), std::end(left), 0) <
           std::accumulate(std::begin(right), std::end(right), 0);
  }
};

TEST(PriorityQueueExternalTest, MoreComplexKeyFunction) {
  std::vector<std::array<std::uint32_t, 4>> values = {
      {1, 2, 3, 4},  // Sum = 10
      {2, 3, 4, 5},  // Sum = 14
      {3, 4, 5, 6},  // Sum = 18
      {4, 5, 6, 6}   // Sum = 21
  };

  PriorityQueueExternal<std::array<std::uint32_t, 4>, ArrayFunction> maxQueue(
      values.size());
  maxQueue.buildFrom(values);

  EXPECT_EQ(maxQueue.size(), 4);
  EXPECT_EQ(maxQueue.front(), 0);

  values[2] = {0, 0, 0, 0};
  maxQueue.update(2);
  EXPECT_EQ(maxQueue.front(), 2);

  auto result = maxQueue.pop();
  EXPECT_EQ(result, 2);
}
