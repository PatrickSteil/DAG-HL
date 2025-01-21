#include "../datastructures/drawer.h"

#include <gtest/gtest.h>

#include <unordered_set>

class DrawerTest : public ::testing::Test {
 protected:
  void SetUp() override { drawer = std::make_unique<Drawer>(10, 42); }

  std::unique_ptr<Drawer> drawer;
};

TEST_F(DrawerTest, Initialization) {
  EXPECT_EQ(drawer->size(), 10);
  EXPECT_TRUE(drawer->hasNext());
}

TEST_F(DrawerTest, Reset) {
  drawer->draw();
  drawer->reset();
  EXPECT_EQ(drawer->size(), 10);
  EXPECT_TRUE(drawer->hasNext());
}

TEST_F(DrawerTest, Draw) {
  std::unordered_set<std::size_t> drawn_numbers;
  for (int i = 0; i < 10; ++i) {
    auto number = drawer->draw();
    EXPECT_NE(number, static_cast<std::size_t>(-1));
    drawn_numbers.insert(number);
  }
  EXPECT_FALSE(drawer->hasNext());
  EXPECT_EQ(drawer->draw(), static_cast<std::size_t>(-1));
  EXPECT_EQ(drawn_numbers.size(), 10);
}

TEST_F(DrawerTest, Add) {
  drawer->add(42);
  EXPECT_EQ(drawer->size(), 11);
}

TEST_F(DrawerTest, Remove) {
  EXPECT_TRUE(drawer->remove(5));
  EXPECT_EQ(drawer->size(), 9);
  EXPECT_FALSE(drawer->remove(5));
}

TEST_F(DrawerTest, DrawAfterRemove) {
  drawer->remove(3);
  for (int i = 0; i < 9; ++i) {
    EXPECT_NE(drawer->draw(), static_cast<std::size_t>(-1));
  }
  EXPECT_FALSE(drawer->hasNext());
}
