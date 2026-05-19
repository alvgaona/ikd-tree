#include <gtest/gtest.h>
#include <ikd_tree/ikd_tree.h>

#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

using Point = ikd_tree::ikdTree_PointType;
using Tree = ikd_tree::KdTree<Point>;
using PointVector = Tree::PointVector;
using BoxPointType = ikd_tree::BoxPointType;

static Point make_point(float x, float y, float z) {
    return Point(x, y, z);
}

static float dist(const Point &a, const Point &b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

static PointVector make_grid(int n) {
    PointVector pts;
    for (int x = 0; x < n; ++x)
        for (int y = 0; y < n; ++y)
            for (int z = 0; z < n; ++z)
                pts.push_back(make_point((float) x, (float) y, (float) z));
    return pts;
}

class IkdTreeTest : public ::testing::Test {
  protected:
    Tree tree;

    void SetUp() override {
        PointVector pts = make_grid(5);
        tree.build(pts);
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
    tree.nearest_search(make_point(2.0f, 2.0f, 2.0f), 1, results, dists);

    ASSERT_EQ(results.size(), 1u);
    EXPECT_NEAR(results[0].x, 2.0f, 1e-5f);
    EXPECT_NEAR(results[0].y, 2.0f, 1e-5f);
    EXPECT_NEAR(results[0].z, 2.0f, 1e-5f);
    EXPECT_NEAR(dists[0], 0.0f, 1e-5f);
}

TEST_F(IkdTreeTest, NearestSearchResultsAreOrdered) {
    PointVector results;
    std::vector<float> dists;
    tree.nearest_search(make_point(0.1f, 0.1f, 0.1f), 5, results, dists);

    ASSERT_EQ(results.size(), 5u);
    for (size_t i = 1; i < dists.size(); ++i)
        EXPECT_LE(dists[i - 1], dists[i]);
}

TEST_F(IkdTreeTest, NearestSearchRespectsMaxDist) {
    PointVector results;
    std::vector<float> dists;
    tree.nearest_search(make_point(0.0f, 0.0f, 0.0f), 10, results, dists, 0.5);

    for (float d : dists)
        EXPECT_LE(d, 0.5f);
}

TEST_F(IkdTreeTest, NearestSearchKLargerThanTree) {
    PointVector results;
    std::vector<float> dists;
    tree.nearest_search(make_point(0.0f, 0.0f, 0.0f), 1000, results, dists);

    EXPECT_EQ((int) results.size(), tree.size());
}

TEST_F(IkdTreeTest, BoxSearchReturnsPointsInBox) {
    // Box search uses half-open intervals [min, max), so vertex_max
    // must be strictly greater than the largest coordinate we want.
    BoxPointType box;
    box.vertex_min[0] = 1.0f;
    box.vertex_max[0] = 3.1f;
    box.vertex_min[1] = 1.0f;
    box.vertex_max[1] = 3.1f;
    box.vertex_min[2] = 1.0f;
    box.vertex_max[2] = 3.1f;

    PointVector results;
    tree.box_search(box, results);

    EXPECT_EQ(results.size(), 27u);
    for (const auto &p : results) {
        EXPECT_GE(p.x, 1.0f);
        EXPECT_LT(p.x, 3.1f);
        EXPECT_GE(p.y, 1.0f);
        EXPECT_LT(p.y, 3.1f);
        EXPECT_GE(p.z, 1.0f);
        EXPECT_LT(p.z, 3.1f);
    }
}

TEST_F(IkdTreeTest, RadiusSearchReturnsPointsWithinRadius) {
    Point center = make_point(2.0f, 2.0f, 2.0f);
    float radius = 1.5f;

    PointVector results;
    tree.radius_search(center, radius, results);

    for (const auto &p : results)
        EXPECT_LE(dist(p, center), radius + 1e-5f);
}

TEST_F(IkdTreeTest, RadiusSearchDoesNotMissPoints) {
    Point center = make_point(2.0f, 2.0f, 2.0f);
    float radius = 1.0f;

    PointVector results;
    tree.radius_search(center, radius, results);

    EXPECT_TRUE(results.size() >= 7u);
}

TEST_F(IkdTreeTest, AddPointsIncreasesSize) {
    int before = tree.size();
    PointVector new_pts = {make_point(10.0f, 10.0f, 10.0f), make_point(11.0f, 11.0f, 11.0f)};
    tree.add_points(new_pts, false);

    EXPECT_EQ(tree.size(), before + 2);
}

TEST_F(IkdTreeTest, AddedPointIsSearchable) {
    PointVector new_pts = {make_point(99.0f, 99.0f, 99.0f)};
    tree.add_points(new_pts, false);

    PointVector results;
    std::vector<float> dists;
    tree.nearest_search(make_point(99.0f, 99.0f, 99.0f), 1, results, dists);

    ASSERT_EQ(results.size(), 1u);
    EXPECT_NEAR(results[0].x, 99.0f, 1e-5f);
    EXPECT_NEAR(dists[0], 0.0f, 1e-5f);
}

TEST_F(IkdTreeTest, DeletePointsDecreasesValidnum) {
    // Add a unique point outside the grid so the add-then-delete
    // traversal finds the exact node (avoids ambiguity when multiple
    // points share a coordinate with a pivot).
    PointVector to_add = {make_point(100.0f, 100.0f, 100.0f)};
    tree.add_points(to_add, false);
    int after_add = tree.validnum();

    tree.delete_points(to_add);
    EXPECT_EQ(tree.validnum(), after_add - 1);
}

TEST_F(IkdTreeTest, DeletedPointNotReturnedInNearestSearch) {
    PointVector to_add = {make_point(100.0f, 100.0f, 100.0f)};
    tree.add_points(to_add, false);

    tree.delete_points(to_add);

    PointVector results;
    std::vector<float> dists;
    tree.nearest_search(make_point(100.0f, 100.0f, 100.0f), 1, results, dists);

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
    tree.delete_points(to_del);
    EXPECT_EQ(tree.validnum(), before - (int) to_del.size());

    // Each deleted point should no longer be the nearest match.
    for (const auto &p : to_del) {
        PointVector results;
        std::vector<float> dists;
        tree.nearest_search(p, 1, results, dists);
        ASSERT_EQ(results.size(), 1u);
        EXPECT_GT(dists[0], 0.0f);
    }
}

TEST_F(IkdTreeTest, DeleteBoxRemovesPointsInRegion) {
    int before = tree.validnum();

    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = -0.1f;
    boxes[0].vertex_max[0] = 0.1f;
    boxes[0].vertex_min[1] = -0.1f;
    boxes[0].vertex_max[1] = 0.1f;
    boxes[0].vertex_min[2] = -0.1f;
    boxes[0].vertex_max[2] = 0.1f;
    tree.delete_point_boxes(boxes);

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
    boxes[0].vertex_min[0] = -0.1f;
    boxes[0].vertex_max[0] = 1.1f;
    boxes[0].vertex_min[1] = -0.1f;
    boxes[0].vertex_max[1] = 1.1f;
    boxes[0].vertex_min[2] = -0.1f;
    boxes[0].vertex_max[2] = 1.1f;
    tree.delete_point_boxes(boxes);

    PointVector removed;
    tree.acquire_removed_points(removed);
    EXPECT_FALSE(removed.empty());
    for (const auto &p : removed) {
        EXPECT_GE(p.x, -0.1f);
        EXPECT_LT(p.x, 1.1f);
        EXPECT_GE(p.y, -0.1f);
        EXPECT_LT(p.y, 1.1f);
        EXPECT_GE(p.z, -0.1f);
        EXPECT_LT(p.z, 1.1f);
    }
}

TEST_F(IkdTreeTest, AcquireRemovedPointsClearsBuffer) {
    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = -0.1f;
    boxes[0].vertex_max[0] = 1.1f;
    boxes[0].vertex_min[1] = -0.1f;
    boxes[0].vertex_max[1] = 1.1f;
    boxes[0].vertex_min[2] = -0.1f;
    boxes[0].vertex_max[2] = 1.1f;
    tree.delete_point_boxes(boxes);

    PointVector first;
    tree.acquire_removed_points(first);
    PointVector second;
    tree.acquire_removed_points(second);
    EXPECT_TRUE(second.empty());
}

// add_point_boxes re-validates soft-deleted points within a box region. Three
// semantic properties:
//
//   1. Single-point revival after delete_points.
//   2. Whole-region revival after delete_point_boxes (regression test for the
//      node-range shrinkage bug — Update() now preserves the full bbox of
//      every subtree so the recursion can still find revivable nodes).
//   3. No-op on empty regions.

TEST(IkdTreeAddBoxes, RevivesWholeBoxAfterBoxDelete) {
    Tree t(0.99f, 0.99f, 0.2f);
    PointVector pts = make_grid(5);
    t.build(pts);
    int before = t.validnum();

    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = -0.1f;
    boxes[0].vertex_max[0] = 2.1f;
    boxes[0].vertex_min[1] = -0.1f;
    boxes[0].vertex_max[1] = 2.1f;
    boxes[0].vertex_min[2] = -0.1f;
    boxes[0].vertex_max[2] = 2.1f;

    t.delete_point_boxes(boxes);
    EXPECT_EQ(t.validnum(), before - 27);

    t.add_point_boxes(boxes);
    EXPECT_EQ(t.validnum(), before);

    PointVector revived;
    t.box_search(boxes[0], revived);
    EXPECT_EQ(revived.size(), 27u);
}

TEST(IkdTreeAddBoxes, RevivesSinglePointAfterPointDelete) {
    // 0.99 thresholds keep the rebuild from physically pruning the deleted
    // node, so add_point_boxes can still find and revive it.
    Tree t(0.99f, 0.99f, 0.2f);
    PointVector pts = make_grid(5);
    t.build(pts);
    int before = t.validnum();

    PointVector to_del = {make_point(2.0f, 2.0f, 2.0f)};
    t.delete_points(to_del);
    EXPECT_EQ(t.validnum(), before - 1);

    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = 1.9f;
    boxes[0].vertex_max[0] = 2.1f;
    boxes[0].vertex_min[1] = 1.9f;
    boxes[0].vertex_max[1] = 2.1f;
    boxes[0].vertex_min[2] = 1.9f;
    boxes[0].vertex_max[2] = 2.1f;
    t.add_point_boxes(boxes);

    EXPECT_EQ(t.validnum(), before);
    PointVector found;
    t.box_search(boxes[0], found);
    ASSERT_EQ(found.size(), 1u);
    EXPECT_NEAR(found[0].x, 2.0f, 1e-5f);
    EXPECT_NEAR(found[0].y, 2.0f, 1e-5f);
    EXPECT_NEAR(found[0].z, 2.0f, 1e-5f);
}

TEST_F(IkdTreeTest, AddPointBoxesNoOpOnEmptyTreeRegion) {
    int before = tree.validnum();
    std::vector<BoxPointType> boxes(1);
    boxes[0].vertex_min[0] = 100.0f;
    boxes[0].vertex_max[0] = 200.0f;
    boxes[0].vertex_min[1] = 100.0f;
    boxes[0].vertex_max[1] = 200.0f;
    boxes[0].vertex_min[2] = 100.0f;
    boxes[0].vertex_max[2] = 200.0f;
    tree.add_point_boxes(boxes);
    EXPECT_EQ(tree.validnum(), before);
    EXPECT_EQ(tree.size(), before);
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
    tree.add_points(cluster, true);
    EXPECT_LT(tree.size() - before, 10);
}

TEST(IkdTreeConfigTest, ConstructorWithCustomParams) {
    Tree custom_tree(0.3f, 0.7f, 0.5f);
    PointVector pts = make_grid(3);
    custom_tree.build(pts);
    EXPECT_EQ(custom_tree.size(), 27);
}

TEST(IkdTreeConfigTest, InitializeKDTreeSetsParams) {
    Tree t;
    t.initialize(0.4f, 0.8f, 0.3f);
    PointVector pts = make_grid(3);
    t.build(pts);
    EXPECT_EQ(t.size(), 27);
}

TEST(IkdTreeConfigTest, SettersDoNotCrashOnEmptyTree) {
    Tree t;
    t.set_delete_criterion_param(0.3f);
    t.set_balance_criterion_param(0.7f);
    t.set_downsample_param(0.5f);
    EXPECT_EQ(t.size(), 0);
}

TEST(IkdTreeEmptyTest, BuildEmptyCloud) {
    Tree tree;
    PointVector empty;
    tree.build(empty);
    EXPECT_EQ(tree.size(), 0);
}

TEST(IkdTreeEmptyTest, SearchOnEmptyTree) {
    Tree tree;
    PointVector results;
    std::vector<float> dists;
    tree.nearest_search(make_point(0.0f, 0.0f, 0.0f), 5, results, dists);
    EXPECT_TRUE(results.empty());
}

TEST(IkdTreeEmptyTest, TreeRangeOnEmptyTreeIsZero) {
    Tree tree;
    BoxPointType range = tree.tree_range();
    EXPECT_EQ(range.vertex_min[0], 0.0f);
    EXPECT_EQ(range.vertex_max[0], 0.0f);
}

// --- kNN correctness oracle: brute-force reference ---

static std::vector<std::pair<float, int>> brute_force_knn(const PointVector &cloud, const Point &query, int k,
                                                          double max_dist = INFINITY) {
    std::vector<std::pair<float, int>> dists;
    dists.reserve(cloud.size());
    double max_dist_sq = max_dist * max_dist;
    for (int i = 0; i < (int) cloud.size(); ++i) {
        float dx = cloud[i].x - query.x;
        float dy = cloud[i].y - query.y;
        float dz = cloud[i].z - query.z;
        float d = dx * dx + dy * dy + dz * dz;
        if (d <= max_dist_sq)
            dists.emplace_back(d, i);
    }
    int kk = std::min<int>(k, (int) dists.size());
    std::partial_sort(dists.begin(), dists.begin() + kk, dists.end(),
                      [](const auto &a, const auto &b) { return a.first < b.first; });
    dists.resize(kk);
    return dists;
}

static PointVector make_random_cloud(int n, uint32_t seed, float spread = 100.0f) {
    PointVector pts;
    pts.reserve(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(-spread, spread);
    for (int i = 0; i < n; ++i)
        pts.push_back(Point(u(rng), u(rng), u(rng)));
    return pts;
}

class IkdTreeKnnReferenceTest : public ::testing::TestWithParam<int> {};

TEST_P(IkdTreeKnnReferenceTest, MatchesBruteForceOnRandomCloud) {
    int k = GetParam();
    PointVector cloud = make_random_cloud(2000, 42);
    Tree tree;
    tree.build(cloud);

    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-150.0f, 150.0f);

    for (int q = 0; q < 50; ++q) {
        Point query(u(rng), u(rng), u(rng));

        PointVector knn_pts;
        std::vector<float> knn_d;
        tree.nearest_search(query, k, knn_pts, knn_d);

        auto truth = brute_force_knn(cloud, query, k);

        ASSERT_EQ(knn_pts.size(), truth.size()) << "k=" << k << " query=" << q;
        for (size_t i = 0; i < truth.size(); ++i) {
            EXPECT_NEAR(knn_d[i], truth[i].first, 1e-3f) << "k=" << k << " query=" << q << " i=" << i;
        }
    }
}

INSTANTIATE_TEST_SUITE_P(VariousK, IkdTreeKnnReferenceTest, ::testing::Values(1, 5, 16, 50, 200));

TEST(IkdTreeKnnReference, MatchesBruteForceWithMaxDist) {
    PointVector cloud = make_random_cloud(1000, 99);
    Tree tree;
    tree.build(cloud);

    std::mt19937 rng(123);
    std::uniform_real_distribution<float> u(-150.0f, 150.0f);

    for (int q = 0; q < 30; ++q) {
        Point query(u(rng), u(rng), u(rng));
        double max_dist = 25.0;

        PointVector knn_pts;
        std::vector<float> knn_d;
        tree.nearest_search(query, 20, knn_pts, knn_d, max_dist);

        auto truth = brute_force_knn(cloud, query, 20, max_dist);

        ASSERT_EQ(knn_pts.size(), truth.size()) << "query=" << q;
        for (size_t i = 0; i < truth.size(); ++i) {
            EXPECT_NEAR(knn_d[i], truth[i].first, 1e-3f);
        }
    }
}

TEST(IkdTreeKnnReference, MatchesBruteForceAfterIncrementalAddDelete) {
    PointVector cloud = make_random_cloud(500, 11);
    Tree tree;
    tree.build(cloud);

    PointVector to_add = make_random_cloud(500, 22, 80.0f);
    tree.add_points(to_add, false);
    PointVector full = cloud;
    full.insert(full.end(), to_add.begin(), to_add.end());

    std::mt19937 rng(33);
    std::uniform_real_distribution<float> u(-100.0f, 100.0f);

    for (int q = 0; q < 30; ++q) {
        Point query(u(rng), u(rng), u(rng));
        PointVector knn_pts;
        std::vector<float> knn_d;
        tree.nearest_search(query, 10, knn_pts, knn_d);

        auto truth = brute_force_knn(full, query, 10);

        ASSERT_EQ(knn_pts.size(), truth.size());
        for (size_t i = 0; i < truth.size(); ++i) {
            EXPECT_NEAR(knn_d[i], truth[i].first, 1e-3f);
        }
    }
}

// --- flatten() ---

static auto point_key = [](const Point &p) { return std::make_tuple(p.x, p.y, p.z); };

TEST_F(IkdTreeTest, FlattenReturnsEveryLivePoint) {
    PointVector out;
    tree.flatten(out);

    EXPECT_EQ((int) out.size(), tree.validnum());
    EXPECT_EQ((int) out.size(), tree.size());

    PointVector expected = make_grid(5);
    std::vector<std::tuple<float, float, float>> got, want;
    for (const auto &p : out)
        got.push_back(point_key(p));
    for (const auto &p : expected)
        want.push_back(point_key(p));
    std::sort(got.begin(), got.end());
    std::sort(want.begin(), want.end());
    EXPECT_EQ(got, want);
}

TEST_F(IkdTreeTest, FlattenSkipsDeletedPoints) {
    PointVector to_del = {make_point(0.0f, 0.0f, 0.0f), make_point(4.0f, 4.0f, 4.0f)};
    tree.delete_points(to_del);

    PointVector out;
    tree.flatten(out);

    EXPECT_EQ((int) out.size(), tree.validnum());
    for (const auto &p : out) {
        EXPECT_FALSE(p.x == 0.0f && p.y == 0.0f && p.z == 0.0f);
        EXPECT_FALSE(p.x == 4.0f && p.y == 4.0f && p.z == 4.0f);
    }
}

TEST_F(IkdTreeTest, FlattenOverwritesStorage) {
    PointVector out = {make_point(-1.0f, -1.0f, -1.0f)};
    tree.flatten(out);
    EXPECT_EQ((int) out.size(), tree.size());
    for (const auto &p : out)
        EXPECT_GE(p.x, 0.0f);
}

TEST(IkdTreeEmptyTest, FlattenEmptyTreeYieldsEmpty) {
    Tree empty_tree;
    PointVector out = {make_point(1.0f, 2.0f, 3.0f)};
    empty_tree.flatten(out);
    EXPECT_TRUE(out.empty());
}

// --- async rebuild correctness ---
//
// Multi_Thread_Rebuild_Point_Num is 1500 inside ikd_tree.cpp; subtrees above
// that threshold are rebuilt on the background pthread. The test grows the
// tree well past the threshold in batches, runs nearest_search against a
// brute-force oracle between batches, and re-checks once the rebuild thread
// has had time to settle. This exercises the concurrent search path that
// runs while Rebuild_Ptr is non-null.

TEST(IkdTreeAsyncRebuild, NearestSearchRemainsCorrectAcrossRebuilds) {
    PointVector cloud = make_random_cloud(500, 1);
    Tree tree;
    tree.build(cloud);

    PointVector full = cloud;
    std::mt19937 rng(2);
    std::uniform_real_distribution<float> u(-150.0f, 150.0f);

    const int batches = 6;
    const int batch_size = 800;
    for (int b = 0; b < batches; ++b) {
        PointVector batch = make_random_cloud(batch_size, 100 + b, 120.0f);
        tree.add_points(batch, false);
        full.insert(full.end(), batch.begin(), batch.end());

        for (int q = 0; q < 5; ++q) {
            Point query(u(rng), u(rng), u(rng));
            PointVector knn_pts;
            std::vector<float> knn_d;
            tree.nearest_search(query, 10, knn_pts, knn_d);

            auto truth = brute_force_knn(full, query, 10);
            ASSERT_EQ(knn_pts.size(), truth.size()) << "batch=" << b << " query=" << q;
            for (size_t i = 0; i < truth.size(); ++i) {
                EXPECT_NEAR(knn_d[i], truth[i].first, 1e-3f) << "batch=" << b << " query=" << q << " i=" << i;
            }
        }
    }

    EXPECT_GT(tree.size(), 1500);

    // Give the background rebuild thread time to finish any in-flight work.
    usleep(200000);

    for (int q = 0; q < 20; ++q) {
        Point query(u(rng), u(rng), u(rng));
        PointVector knn_pts;
        std::vector<float> knn_d;
        tree.nearest_search(query, 10, knn_pts, knn_d);

        auto truth = brute_force_knn(full, query, 10);
        ASSERT_EQ(knn_pts.size(), truth.size()) << "settled query=" << q;
        for (size_t i = 0; i < truth.size(); ++i) {
            EXPECT_NEAR(knn_d[i], truth[i].first, 1e-3f) << "settled query=" << q << " i=" << i;
        }
    }
}
