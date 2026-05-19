#include <benchmark/benchmark.h>
#include <ikd_tree/ikd_tree.h>

#include <algorithm>
#include <random>

using Point = ikd_tree::ikdTree_PointType;
using Tree = ikd_tree::KdTree<Point>;
using PointVector = Tree::PointVector;

static PointVector make_random_cloud(int n, uint32_t seed, float spread = 100.0f) {
    PointVector pts;
    pts.reserve(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(-spread, spread);
    for (int i = 0; i < n; ++i)
        pts.push_back(Point(u(rng), u(rng), u(rng)));
    return pts;
}

// kNN search throughput across cloud size N (state.range(0)) and k (state.range(1)).
static void BM_NearestSearch(benchmark::State &state) {
    const int n = state.range(0);
    const int k = state.range(1);

    PointVector cloud = make_random_cloud(n, 42);
    Tree tree;
    tree.build(cloud);

    // Pre-generate query points so the loop measures only Search work.
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-150.0f, 150.0f);
    constexpr int Q = 256;
    std::vector<Point> queries;
    queries.reserve(Q);
    for (int i = 0; i < Q; ++i)
        queries.push_back(Point(u(rng), u(rng), u(rng)));

    PointVector results;
    std::vector<float> dists;
    int qi = 0;
    for (auto _ : state) {
        tree.nearest_search(queries[qi], k, results, dists);
        benchmark::DoNotOptimize(results);
        benchmark::DoNotOptimize(dists);
        qi = (qi + 1) % Q;
    }
    state.SetItemsProcessed(state.iterations());
    state.counters["k"] = k;
    state.counters["N"] = n;
}

// Cloud sizes from small to large; k from typical robotics values.
BENCHMARK(BM_NearestSearch)->ArgsProduct({{100, 1000, 10000, 100000}, {1, 5, 10, 30}})->Unit(benchmark::kMicrosecond);

// build benchmark across cloud sizes.
static void BM_Build(benchmark::State &state) {
    const int n = state.range(0);
    PointVector cloud = make_random_cloud(n, 42);
    for (auto _ : state) {
        Tree tree;
        tree.build(cloud);
        benchmark::DoNotOptimize(tree);
    }
    state.SetItemsProcessed(state.iterations() * (int64_t) n);
    state.counters["N"] = n;
}

BENCHMARK(BM_Build)->Arg(100)->Arg(1000)->Arg(10000)->Arg(100000)->Unit(benchmark::kMicrosecond);

// Search throughput on a tree where ~30% of points are soft-deleted.
// Exercises the looser node_range bounds introduced when Update() started
// preserving deleted children's extents — search pruning is less aggressive
// here than in the all-alive case.
static void BM_NearestSearchAfterDeletes(benchmark::State &state) {
    const int n = state.range(0);
    const int k = state.range(1);

    PointVector cloud = make_random_cloud(n, 42);
    // High thresholds keep Criterion_Check from physically rebuilding away
    // the deleted nodes; otherwise the scenario degrades to the all-alive case.
    Tree tree(0.99f, 0.99f, 0.2f);
    tree.build(cloud);

    std::mt19937 rng(7);
    PointVector shuffled = cloud;
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    PointVector to_del(shuffled.begin(), shuffled.begin() + n / 3);
    tree.delete_points(to_del);

    std::uniform_real_distribution<float> u(-150.0f, 150.0f);
    constexpr int Q = 256;
    std::vector<Point> queries;
    queries.reserve(Q);
    for (int i = 0; i < Q; ++i)
        queries.push_back(Point(u(rng), u(rng), u(rng)));

    PointVector results;
    std::vector<float> dists;
    int qi = 0;
    for (auto _ : state) {
        tree.nearest_search(queries[qi], k, results, dists);
        benchmark::DoNotOptimize(results);
        benchmark::DoNotOptimize(dists);
        qi = (qi + 1) % Q;
    }
    state.SetItemsProcessed(state.iterations());
    state.counters["k"] = k;
    state.counters["N"] = n;
}
BENCHMARK(BM_NearestSearchAfterDeletes)
    ->ArgsProduct({{1000, 10000, 100000}, {1, 10, 30}})
    ->Unit(benchmark::kMicrosecond);

// Insert a batch of points into a pre-built tree large enough to trigger the
// async rebuild path (subtrees > Multi_Thread_Rebuild_Point_Num = 1500).
// Hits the pthread_mutex_lock(&threads_->...) sites on the insert path —
// the indirection the pimpl pass introduced.
static void BM_AddBatch(benchmark::State &state) {
    const int base_n = state.range(0);
    const int batch_n = state.range(1);
    PointVector base_cloud = make_random_cloud(base_n, 42);
    PointVector batch = make_random_cloud(batch_n, 99, 120.0f);

    for (auto _ : state) {
        state.PauseTiming();
        Tree tree;
        tree.build(base_cloud);
        state.ResumeTiming();
        tree.add_points(batch, false);
        benchmark::DoNotOptimize(tree);
    }
    state.SetItemsProcessed(state.iterations() * (int64_t) batch_n);
    state.counters["base"] = base_n;
    state.counters["batch"] = batch_n;
}
BENCHMARK(BM_AddBatch)->ArgsProduct({{10000}, {100, 1000, 5000}})->Unit(benchmark::kMicrosecond);
