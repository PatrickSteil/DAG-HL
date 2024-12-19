/*
 * Licensed under MIT License.
 * Author: Patrick Steil
*/

#include "../datastructures/graph.h"
#include "../datastructures/pruned_landmark_labeling.h"

#include <gtest/gtest.h>

TEST(PLLTest, Initialization) {
  Graph fwdGraph;
  Graph bwdGraph;
  PLL pll(fwdGraph, bwdGraph);

  EXPECT_EQ(pll.labels[BWD].size(), fwdGraph.numVertices());
  EXPECT_EQ(pll.labels[FWD].size(), fwdGraph.numVertices());
  EXPECT_EQ(pll.lookup[BWD].size(), fwdGraph.numVertices());
  EXPECT_EQ(pll.lookup[FWD].size(), fwdGraph.numVertices());
}

TEST(PLLTest, RunWithOrdering) {
  Graph fwdGraph;
  fwdGraph.readFromEdgeList("../tests/pll_test_graph.txt");
  Graph bwdGraph = fwdGraph.reverseGraph();

  PLL pll(fwdGraph, bwdGraph);

  std::vector<Vertex> ordering = {1, 0, 2, 3};
  pll.run(ordering);

  EXPECT_EQ(pll.labels[BWD][3].size(), 3);
  EXPECT_EQ(pll.labels[FWD][3].size(), 1);

  EXPECT_EQ(pll.labels[FWD][0].size(), 2);
  EXPECT_EQ(pll.labels[BWD][0].size(), 1);

  EXPECT_EQ(pll.labels[FWD][2].size(), 1);
  EXPECT_EQ(pll.labels[BWD][2].size(), 2);
}
