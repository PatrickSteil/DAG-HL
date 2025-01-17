#include "../datastructures/forest.h"

#include <gtest/gtest.h>

TEST(EdgeTreeTest, BasicOperations) {
  EdgeTree<std::vector<Index>> tree(10);
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
  EdgeTree<std::vector<Index>> tree(5);
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
  Forest forest(10);
  EXPECT_EQ(forest.numVertices, 10);

  std::size_t tree1_index = forest.newTree();
  std::size_t tree2_index = forest.newTree();

  auto& tree1 = forest.getTree(tree1_index);
  auto& tree2 = forest.getTree(tree2_index);

  EXPECT_EQ(tree1.capacity(), 10);
  EXPECT_EQ(tree2.capacity(), 10);

  EXPECT_EQ(forest.numberOfTrees(), 2);

  tree1.setRoot(0);
  tree1.addEdge(0, 1, FWD);
  tree1.addEdge(0, 2, FWD);

  tree2.setRoot(5);
  tree2.addEdge(5, 6, FWD);

  forest.computeSubtreeSizes();
  EXPECT_GT(tree1.getOrDefault(0), 0);
  EXPECT_GT(tree2.getOrDefault(5), 0);

  forest.removeSubtreesAtVertex(0);
  forest.removeTreesWithNoEdges();
  EXPECT_EQ(forest.numberOfTrees(), 1);
}
