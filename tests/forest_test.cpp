#include "../datastructures/forest.h"

#include <gtest/gtest.h>

TEST(EdgeTreeTest, BasicOperations) {
  std::vector<Index> topoRank = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  EdgeTree<std::vector<Index>> tree(10, topoRank);
  tree.setRoot(0);
  tree.addEdge(0, 1, FWD);
  tree.addEdge(0, 2, FWD);
  tree.addEdge(1, 3, FWD);
  tree.addEdge(1, 4, FWD);

  tree.computeDescendants();

  EXPECT_EQ(tree.getRoot(), 0);
  EXPECT_EQ(tree.numEdges(), 4);

  tree.removeSubtree(1);
  EXPECT_EQ(tree.numEdges(), 1);
}

TEST(EdgeTreeTest, ComputeDescendants) {
  std::vector<Index> topoRank = {0, 1, 2, 3, 4};
  EdgeTree<std::vector<Index>> tree(5, topoRank);
  tree.setRoot(0);
  tree.addEdge(0, 1, FWD);
  tree.addEdge(0, 2, FWD);
  tree.addEdge(1, 3, FWD);
  tree.addEdge(1, 4, FWD);

  tree.computeDescendants();

  EXPECT_GT(tree.getOrDefault(0), tree.getOrDefault(1));
  EXPECT_GT(tree.getOrDefault(1), tree.getOrDefault(3));
}

TEST(ForestTest, TreeManagement) {
  std::vector<Index> topoRank = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
  Forest forest(10, topoRank);
  EXPECT_EQ(forest.numVertices, 10);

  std::size_t tree1_index = forest.newTree();
  std::size_t tree2_index = forest.newTree();

  auto& tree1 = forest[tree1_index];
  auto& tree2 = forest[tree2_index];

  EXPECT_EQ(tree1.capacity(), 10);
  EXPECT_EQ(tree2.capacity(), 10);

  EXPECT_EQ(forest.numberOfTrees(), 2);

  tree1.setRoot(0);
  tree1.addEdge(0, 1, FWD);
  tree1.addEdge(0, 2, FWD);

  tree2.setRoot(5);
  tree2.addEdge(5, 6, FWD);

  forest.computeSubtreeSizes();
  EXPECT_EQ(tree1.getOrDefault(0), 2);
  EXPECT_EQ(tree2.getOrDefault(5), 1);

  forest.removeSubtreesAtVertex(0);
  EXPECT_EQ(tree1.getOrDefault(0), 0);
  EXPECT_EQ(tree2.getOrDefault(5), 1);

  forest.removeTreesWithNoEdges();
  EXPECT_EQ(forest.numberOfTrees(), 1);

  std::size_t tree3_index = forest.newTree();
  auto& tree3 = forest[tree3_index];

  auto& tree2_new = forest[0];

  tree3.setRoot(2);
  tree3.addEdge(2, 3, FWD);
  tree3.addEdge(2, 5, FWD);
  tree3.addEdge(5, 6, FWD);

  tree3.addEdge(2, 1, BWD);

  forest.computeSubtreeSizes();
  EXPECT_EQ(tree2_new.getOrDefault(5), 1);
  EXPECT_EQ(tree3.getOrDefault(2), 4);
  EXPECT_EQ(tree3.getOrDefault(3), 0);
  EXPECT_EQ(tree3.getOrDefault(4), 0);
  EXPECT_EQ(tree3.getOrDefault(5), 1);
  EXPECT_EQ(tree3.getOrDefault(6), 0);
}
