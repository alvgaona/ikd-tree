# ikd-Tree

[![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

An incremental k-d tree for 3D point clouds. Insert, delete, k-NN, box and radius search, with
unbalanced subtrees rebuilt lazily on a background thread.

> [!NOTE]
> Fork of [hku-mars/ikd-Tree](https://github.com/hku-mars/ikd-Tree), the reference implementation
> from the paper *ikd-Tree: An Incremental K-D Tree for Robotic Applications* (Cai, Ren, Zhang,
> HKU-MARS, 2021). This fork adds tests, sanitisers, CI, a namespaced API, and an installable
> CMake package.

## Example

```cpp
#include <ikd_tree/ikd_tree.h>

using Tree = ikd_tree::KdTree<ikd_tree::ikdTree_PointType>;
Tree tree;
tree.build(cloud);

Tree::PointVector knn;
std::vector<float> dists;
tree.nearest_search({1, 2, 3}, /*k=*/10, knn, dists);
```

## Build

```bash
pixi run build   # compile lib + tests + bench + examples
pixi run test    # run the test suite
```

[pixi](https://pixi.sh) pins the full toolchain — PCL, Eigen, GoogleTest, Google Benchmark. Other
tasks: `bench`, `test-asan`, `test-tsan`, `format`, `format-check`.

## Use in your project

```cmake
find_package(ikd_tree REQUIRED)
target_link_libraries(your_target PRIVATE ikd_tree::ikd_tree)
```
