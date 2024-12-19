/*
 * Licensed under MIT License.
 * Author: Patrick Steil
*/

#include "../datastructures/priority_queue.h"

#include <gtest/gtest.h>

#include <cstdint>
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
