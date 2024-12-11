#include "../datastructures/hub_labels.h"
#include <gtest/gtest.h>

TEST(LabelTest, BasicOperations) {
  Label label;

  EXPECT_EQ(label.size(), 0);

  label.add(10);
  label.add(20);
  EXPECT_EQ(label.size(), 2);

  EXPECT_EQ(label[0], 10);
  EXPECT_EQ(label[1], 20);

  label[1] = 30;
  EXPECT_EQ(label[1], 30);
}

TEST(LabelTest, Init) {
  Label label;
  EXPECT_EQ(label.size(), 0);
}

TEST(LabelTest, DeltaRepresentation) {
  Label label;
  label.add(0);
  label.add(16);
  label.add(29);
  label.add(189);
  label.add(299);
  label.add(446);
  label.add(529);

  EXPECT_EQ(label.size(), 7);

  label.setDeltaRepresentation();

  std::vector<Vertex> expected = {0, 15, 12, 159, 109, 146, 82};
  EXPECT_EQ(label.nodes, expected);
}
