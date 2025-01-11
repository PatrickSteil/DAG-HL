#include "../datastructures/bfs_tools.h"

#include <gtest/gtest.h>

#include <atomic>
#include <thread>

class FixedSizedQueueThreadSafeTest : public ::testing::Test {
 protected:
  static constexpr std::size_t queueSize = 1000;
  bfs::FixedSizedQueueThreadSafe<int> queue;

  FixedSizedQueueThreadSafeTest() : queue(queueSize) {}
};

TEST_F(FixedSizedQueueThreadSafeTest, ConcurrentPushPop) {
  auto producer = [this](int start, int count) {
    for (int i = start; i < start + count; ++i) {
      queue.push(i);
    }
  };

  auto consumer = [this](int count, std::atomic<int> &sum) {
    for (int i = 0; i < count; ++i) {
      int value = -1;
      do {
        value = queue.pop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      } while (value == -1);

      sum += value;
    }
  };

  std::atomic<int> sum1(0), sum2(0);
  std::thread t1(producer, 0, queueSize / 2);
  std::thread t2(producer, queueSize / 2, queueSize / 2);
  std::thread t3(consumer, queueSize / 2, std::ref(sum1));
  std::thread t4(consumer, queueSize / 2, std::ref(sum2));

  t1.join();
  t2.join();
  t3.join();
  t4.join();

  int expectedSum = (queueSize / 2) * (queueSize / 2 - 1) / 2 +
                    (queueSize / 2) * (queueSize / 2 + queueSize - 1) / 2;
  EXPECT_EQ(sum1 + sum2, expectedSum);
}

TEST(GenerationCheckerThreadSafeTest, SequentialMarking) {
  const static std::size_t SIZE = 1000;
  bfs::GenerationCheckerThreadSafe<> checker(SIZE);

  std::vector<std::size_t> q(SIZE, 0);

  for (std::size_t i = 0; i < SIZE; ++i) {
    if (checker.isMarked(i)) continue;
    checker.mark(i);
    q[i] = i;
  }

  std::size_t totalSum = std::accumulate(q.begin(), q.end(), 0);
  EXPECT_EQ(totalSum, (SIZE * (SIZE - 1)) / 2);
}

TEST(GenerationCheckerThreadSafeTest, ConcurrentMarking) {
  const static std::size_t SIZE = 1000;
  bfs::GenerationCheckerThreadSafe<> checker(SIZE);

  std::vector<std::size_t> q(SIZE, 0);
  auto worker = [&](std::size_t thread_id) {
    for (std::size_t i = thread_id; i < SIZE; ++i) {
      if (checker.isMarked(i)) continue;
      checker.mark(i);
      q[i] = i;
    }
  };

  const static std::size_t NUM_THREADS = 16;
  std::vector<std::thread> threads;
  for (std::size_t t = 0; t < NUM_THREADS; ++t) {
    threads.emplace_back(worker, t);
  }

  for (auto &t : threads) {
    t.join();
  }

  std::size_t totalSum = std::accumulate(q.begin(), q.end(), 0);
  EXPECT_EQ(totalSum, (SIZE * (SIZE - 1)) / 2);
}
