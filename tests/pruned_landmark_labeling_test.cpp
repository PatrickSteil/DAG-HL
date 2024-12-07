#include "../datastructures/graph.h"
#include "../datastructures/pruned_landmark_labeling.h"

#include <gtest/gtest.h>

TEST(LabelTest, BasicOperations) {
  PLL::Label label;

  EXPECT_EQ(label.size(), 0);

  label.add(10);
  label.add(20);
  EXPECT_EQ(label.size(), 2);

  EXPECT_EQ(label[0], 10);
  EXPECT_EQ(label[1], 20);

  label[1] = 30;
  EXPECT_EQ(label[1], 30);
}

TEST(PLLTest, Initialization) {
  Graph fwdGraph;
  Graph bwdGraph;
  PLL pll(fwdGraph, bwdGraph);

  EXPECT_EQ(pll.labels[PLL::BWD].size(), fwdGraph.numVertices());
  EXPECT_EQ(pll.labels[PLL::FWD].size(), fwdGraph.numVertices());
  EXPECT_EQ(pll.lookup[PLL::BWD].size(), fwdGraph.numVertices());
  EXPECT_EQ(pll.lookup[PLL::FWD].size(), fwdGraph.numVertices());
}

TEST(PLLTest, RunWithOrdering) {

  Graph fwdGraph;
  fwdGraph.readFromEdgeList("../tests/pll_test_graph.txt");
  Graph bwdGraph = fwdGraph.reverseGraph();

  PLL pll(fwdGraph, bwdGraph);

  std::vector<Vertex> ordering = {1, 0, 2, 3};
  pll.run(ordering);

  EXPECT_EQ(pll.labels[PLL::BWD][3].size(), 3);
  EXPECT_EQ(pll.labels[PLL::FWD][3].size(), 1);

  EXPECT_EQ(pll.labels[PLL::FWD][0].size(), 2);
  EXPECT_EQ(pll.labels[PLL::BWD][0].size(), 1);

  EXPECT_EQ(pll.labels[PLL::FWD][2].size(), 1);
  EXPECT_EQ(pll.labels[PLL::BWD][2].size(), 2);
}
