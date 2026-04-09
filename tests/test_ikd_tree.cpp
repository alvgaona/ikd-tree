#include <gtest/gtest.h>
#include "ikd_tree.h"

#include <cmath>
#include <vector>

using Point = ikdTree_PointType;
using Tree = KD_TREE<Point>;
using PointVector = Tree::PointVector;

static Point make_point(float x, float y, float z) { return Point(x, y, z); }

static float dist(const Point &a, const Point &b) {
    return std::sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z));
}

static PointVector make_grid(int n) {
    PointVector pts;
    for (int x = 0; x < n; ++x)
        for (int y = 0; y < n; ++y)
            for (int z = 0; z < n; ++z)
                pts.push_back(make_point((float)x, (float)y, (float)z));
    return pts;
}

class IkdTreeTest : public ::testing::Test {
protected:
    Tree tree;

    void SetUp() override {
        PointVector pts = make_grid(5);
        tree.Build(pts);
    }
};

TEST_F(IkdTreeTest, BuildSetsCorrectSize) {
    EXPECT_EQ(tree.size(), 125);
}

TEST_F(IkdTreeTest, ValidnumEqualsSize) {
    EXPECT_EQ(tree.validnum(), tree.size());
}

TEST_F(IkdTreeTest, NearestSearchFindsExactPoint) {
    PointVector results;
    std::vector<float> dists;
    tree.Nearest_Search(make_point(2.0f, 2.0f, 2.0f), 1, results, dists);

    ASSERT_EQ(results.size(), 1u);
    EXPECT_NEAR(results[0].x, 2.0f, 1e-5f);
    EXPECT_NEAR(results[0].y, 2.0f, 1e-5f);
    EXPECT_NEAR(results[0].z, 2.0f, 1e-5f);
    EXPECT_NEAR(dists[0], 0.0f, 1e-5f);
}

TEST_F(IkdTreeTest, NearestSearchResultsAreOrdered) {
    PointVector results;
    std::vector<float> dists;
    tree.Nearest_Search(make_point(0.1f, 0.1f, 0.1f), 5, results, dists);

    ASSERT_EQ(results.size(), 5u);
    for (size_t i = 1; i < dists.size(); ++i)
        EXPECT_LE(dists[i-1], dists[i]);
}

TEST_F(IkdTreeTest, NearestSearchRespectsMaxDist) {
    PointVector results;
    std::vector<float> dists;
    tree.Nearest_Search(make_point(0.0f, 0.0f, 0.0f), 10, results, dists, 0.5);

    for (float d : dists)
        EXPECT_LE(d, 0.5f);
}

TEST_F(IkdTreeTest, NearestSearchKLargerThanTree) {
    PointVector results;
    std::vector<float> dists;
    tree.Nearest_Search(make_point(0.0f, 0.0f, 0.0f), 1000, results, dists);

    EXPECT_EQ((int)results.size(), tree.size());
}

TEST_F(IkdTreeTest, BoxSearchReturnsPointsInBox) {
    // Box search uses half-open intervals [min, max), so vertex_max
    // must be strictly greater than the largest coordinate we want.
    BoxPointType box;
    box.vertex_min[0] = 1.0f; box.vertex_max[0] = 3.1f;
    box.vertex_min[1] = 1.0f; box.vertex_max[1] = 3.1f;
    box.vertex_min[2] = 1.0f; box.vertex_max[2] = 3.1f;

    PointVector results;
    tree.Box_Search(box, results);

    EXPECT_EQ(results.size(), 27u);
    for (const auto &p : results) {
        EXPECT_GE(p.x, 1.0f); EXPECT_LT(p.x, 3.1f);
        EXPECT_GE(p.y, 1.0f); EXPECT_LT(p.y, 3.1f);
        EXPECT_GE(p.z, 1.0f); EXPECT_LT(p.z, 3.1f);
    }
}

TEST_F(IkdTreeTest, RadiusSearchReturnsPointsWithinRadius) {
    Point center = make_point(2.0f, 2.0f, 2.0f);
    float radius = 1.5f;

    PointVector results;
    tree.Radius_Search(center, radius, results);

    for (const auto &p : results)
        EXPECT_LE(dist(p, center), radius + 1e-5f);
}

TEST_F(IkdTreeTest, RadiusSearchDoesNotMissPoints) {
    Point center = make_point(2.0f, 2.0f, 2.0f);
    float radius = 1.0f;

    PointVector results;
    tree.Radius_Search(center, radius, results);

    EXPECT_TRUE(results.size() >= 7u);
}

TEST_F(IkdTreeTest, AddPointsIncreasesSize) {
    int before = tree.size();
    PointVector new_pts = { make_point(10.0f, 10.0f, 10.0f),
                            make_point(11.0f, 11.0f, 11.0f) };
    tree.Add_Points(new_pts, false);

    EXPECT_EQ(tree.size(), before + 2);
}

TEST_F(IkdTreeTest, AddedPointIsSearchable) {
    PointVector new_pts = { make_point(99.0f, 99.0f, 99.0f) };
    tree.Add_Points(new_pts, false);

    PointVector results;
    std::vector<float> dists;
    tree.Nearest_Search(make_point(99.0f, 99.0f, 99.0f), 1, results, dists);

    ASSERT_EQ(results.size(), 1u);
    EXPECT_NEAR(results[0].x, 99.0f, 1e-5f);
    EXPECT_NEAR(dists[0], 0.0f, 1e-5f);
}

TEST_F(IkdTreeTest, DeletePointsDecreasesValidnum) {
    // Add a unique point outside the grid so the add-then-delete
    // traversal finds the exact node (avoids ambiguity when multiple
    // points share a coordinate with a pivot).
    PointVector to_add = { make_point(100.0f, 100.0f, 100.0f) };
    tree.Add_Points(to_add, false);
    int after_add = tree.validnum();

    tree.Delete_Points(to_add);
    EXPECT_EQ(tree.validnum(), after_add - 1);
}

TEST_F(IkdTreeTest, DeletedPointNotReturnedInNearestSearch) {
    PointVector to_add = { make_point(100.0f, 100.0f, 100.0f) };
    tree.Add_Points(to_add, false);

    tree.Delete_Points(to_add);

    PointVector results;
    std::vector<float> dists;
    tree.Nearest_Search(make_point(100.0f, 100.0f, 100.0f), 1, results, dists);

    ASSERT_EQ(results.size(), 1u);
    EXPECT_GT(dists[0], 0.0f);
}

// Regression test: before the fix, Delete_by_point used a strict-< traversal
// that could miss points whose coordinates equal a pivot along the path,
// because nth_element may place equal-valued points on either subtree.
// The original grid contains (0,0,0) and many points sharing each coord with
// internal pivots, so deleting any of them must succeed.
TEST_F(IkdTreeTest, DeletePointsHandlesPivotEqualCoordinates) {
    PointVector to_del = {
        make_point(0.0f, 0.0f, 0.0f),
        make_point(2.0f, 2.0f, 2.0f),
        make_point(4.0f, 4.0f, 4.0f),
        make_point(2.0f, 0.0f, 4.0f),
    };
    int before = tree.validnum();
    tree.Delete_Points(to_del);
    EXPECT_EQ(tree.validnum(), before - (int)to_del.size());

    // Each deleted point should no longer be the nearest match.
    for (const auto &p : to_del) {
        PointVector results;
        std::vector<float> dists;
        tree.Nearest_Search(p, 1, results, dists);
        ASSERT_EQ(results.size(), 1u);
        EXPECT_GT(dists[0], 0.0f);
    }
}

TEST_F(IkdTreeTest, DeleteBoxRemovesPointsInRegion) {
    int before = tree.validnum();

    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = -0.1f; boxes[0].vertex_max[0] = 0.1f;
    boxes[0].vertex_min[1] = -0.1f; boxes[0].vertex_max[1] = 0.1f;
    boxes[0].vertex_min[2] = -0.1f; boxes[0].vertex_max[2] = 0.1f;
    tree.Delete_Point_Boxes(boxes);

    EXPECT_LT(tree.validnum(), before);
}

TEST_F(IkdTreeTest, TreeRangeMatchesGridBounds) {
    BoxPointType range = tree.tree_range();
    EXPECT_NEAR(range.vertex_min[0], 0.0f, 1e-5f);
    EXPECT_NEAR(range.vertex_min[1], 0.0f, 1e-5f);
    EXPECT_NEAR(range.vertex_min[2], 0.0f, 1e-5f);
    EXPECT_NEAR(range.vertex_max[0], 4.0f, 1e-5f);
    EXPECT_NEAR(range.vertex_max[1], 4.0f, 1e-5f);
    EXPECT_NEAR(range.vertex_max[2], 4.0f, 1e-5f);
}

TEST_F(IkdTreeTest, RootAlphaReturnsValidRatios) {
    float alpha_bal = -1.0f, alpha_del = -1.0f;
    tree.root_alpha(alpha_bal, alpha_del);
    EXPECT_GE(alpha_bal, 0.0f);
    EXPECT_LE(alpha_bal, 1.0f);
    EXPECT_GE(alpha_del, 0.0f);
    EXPECT_LE(alpha_del, 1.0f);
}

TEST_F(IkdTreeTest, AcquireRemovedPointsAfterBoxDelete) {
    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = -0.1f; boxes[0].vertex_max[0] = 1.1f;
    boxes[0].vertex_min[1] = -0.1f; boxes[0].vertex_max[1] = 1.1f;
    boxes[0].vertex_min[2] = -0.1f; boxes[0].vertex_max[2] = 1.1f;
    tree.Delete_Point_Boxes(boxes);

    PointVector removed;
    tree.acquire_removed_points(removed);
    EXPECT_FALSE(removed.empty());
    for (const auto &p : removed) {
        EXPECT_GE(p.x, -0.1f); EXPECT_LT(p.x, 1.1f);
        EXPECT_GE(p.y, -0.1f); EXPECT_LT(p.y, 1.1f);
        EXPECT_GE(p.z, -0.1f); EXPECT_LT(p.z, 1.1f);
    }
}

TEST_F(IkdTreeTest, AcquireRemovedPointsClearsBuffer) {
    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = -0.1f; boxes[0].vertex_max[0] = 1.1f;
    boxes[0].vertex_min[1] = -0.1f; boxes[0].vertex_max[1] = 1.1f;
    boxes[0].vertex_min[2] = -0.1f; boxes[0].vertex_max[2] = 1.1f;
    tree.Delete_Point_Boxes(boxes);

    PointVector first;
    tree.acquire_removed_points(first);
    PointVector second;
    tree.acquire_removed_points(second);
    EXPECT_TRUE(second.empty());
}

TEST_F(IkdTreeTest, AddPointBoxesDoesNotCrash) {
    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = -0.1f; boxes[0].vertex_max[0] = 2.1f;
    boxes[0].vertex_min[1] = -0.1f; boxes[0].vertex_max[1] = 2.1f;
    boxes[0].vertex_min[2] = -0.1f; boxes[0].vertex_max[2] = 2.1f;
    tree.Delete_Point_Boxes(boxes);
    tree.Add_Point_Boxes(boxes);
    EXPECT_GT(tree.size(), 0);
}

TEST_F(IkdTreeTest, AddPointsWithDownsamplingLimitsDensity) {
    // Start from an existing built tree, then add a cluster of close points
    // inside a large downsample voxel. With downsample_on=true, only one
    // representative per voxel should be kept.
    int before = tree.size();
    tree.set_downsample_param(10.0f);
    PointVector cluster;
    for (int i = 0; i < 10; ++i)
        cluster.push_back(make_point(50.0f + 0.1f * i, 50.0f, 50.0f));
    tree.Add_Points(cluster, true);
    EXPECT_LT(tree.size() - before, 10);
}

TEST(IkdTreeConfigTest, ConstructorWithCustomParams) {
    Tree custom_tree(0.3f, 0.7f, 0.5f);
    PointVector pts = make_grid(3);
    custom_tree.Build(pts);
    EXPECT_EQ(custom_tree.size(), 27);
}

TEST(IkdTreeConfigTest, InitializeKDTreeSetsParams) {
    Tree t;
    t.InitializeKDTree(0.4f, 0.8f, 0.3f);
    PointVector pts = make_grid(3);
    t.Build(pts);
    EXPECT_EQ(t.size(), 27);
}

TEST(IkdTreeConfigTest, SettersDoNotCrashOnEmptyTree) {
    Tree t;
    t.Set_delete_criterion_param(0.3f);
    t.Set_balance_criterion_param(0.7f);
    t.set_downsample_param(0.5f);
    EXPECT_EQ(t.size(), 0);
}

TEST(IkdTreeEmptyTest, BuildEmptyCloud) {
    Tree tree;
    PointVector empty;
    tree.Build(empty);
    EXPECT_EQ(tree.size(), 0);
}

TEST(IkdTreeEmptyTest, SearchOnEmptyTree) {
    Tree tree;
    PointVector results;
    std::vector<float> dists;
    tree.Nearest_Search(make_point(0.0f, 0.0f, 0.0f), 5, results, dists);
    EXPECT_TRUE(results.empty());
}

TEST(IkdTreeEmptyTest, TreeRangeOnEmptyTreeIsZero) {
    Tree tree;
    BoxPointType range = tree.tree_range();
    EXPECT_EQ(range.vertex_min[0], 0.0f);
    EXPECT_EQ(range.vertex_max[0], 0.0f);
}
