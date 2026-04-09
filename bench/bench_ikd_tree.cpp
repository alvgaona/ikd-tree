#include <benchmark/benchmark.h>
#include "ikd_tree.h"

#include <random>

using Point = ikdTree_PointType;
using Tree = KD_TREE<Point>;
using PointVector = Tree::PointVector;

static PointVector make_random_cloud(int n, uint32_t seed, float spread = 100.0f) {
    PointVector pts;
    pts.reserve(n);
    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> u(-spread, spread);
    for (int i = 0; i < n; ++i) pts.push_back(Point(u(rng), u(rng), u(rng)));
    return pts;
}

// kNN search throughput across cloud size N (state.range(0)) and k (state.range(1)).
static void BM_NearestSearch(benchmark::State &state) {
    const int n = state.range(0);
    const int k = state.range(1);

    PointVector cloud = make_random_cloud(n, 42);
    Tree tree;
    tree.Build(cloud);

    // Pre-generate query points so the loop measures only Search work.
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> u(-150.0f, 150.0f);
    constexpr int Q = 256;
    std::vector<Point> queries;
    queries.reserve(Q);
    for (int i = 0; i < Q; ++i) queries.push_back(Point(u(rng), u(rng), u(rng)));

    PointVector results;
    std::vector<float> dists;
    int qi = 0;
    for (auto _ : state) {
        tree.Nearest_Search(queries[qi], k, results, dists);
        benchmark::DoNotOptimize(results);
        benchmark::DoNotOptimize(dists);
        qi = (qi + 1) % Q;
    }
    state.SetItemsProcessed(state.iterations());
    state.counters["k"] = k;
    state.counters["N"] = n;
}

// Cloud sizes from small to large; k from typical robotics values.
BENCHMARK(BM_NearestSearch)
    ->ArgsProduct({{100, 1000, 10000, 100000}, {1, 5, 10, 30}})
    ->Unit(benchmark::kMicrosecond);

// Build benchmark across cloud sizes.
static void BM_Build(benchmark::State &state) {
    const int n = state.range(0);
    PointVector cloud = make_random_cloud(n, 42);
    for (auto _ : state) {
        Tree tree;
        tree.Build(cloud);
        benchmark::DoNotOptimize(tree);
    }
    state.SetItemsProcessed(state.iterations() * (int64_t)n);
    state.counters["N"] = n;
}

BENCHMARK(BM_Build)->Arg(100)->Arg(1000)->Arg(10000)->Arg(100000)->Unit(benchmark::kMicrosecond);
