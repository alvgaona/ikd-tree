# ikd-Tree

An incremental k-d tree for 3D point clouds, designed for online robotic perception (LiDAR SLAM,
mapping, localisation). Supports incremental insert/delete, box-wise add/delete, k-nearest-neighbour
and radius/box search, voxel down-sampling, and lazy background rebuild of unbalanced subtrees on a
dedicated worker thread.

Based on the paper *ikd-Tree: An Incremental K-D Tree for Robotic Applications* (Cai, Ren, Zhang,
HKU-MARS, 2021).

## Requirements

- C++17
- CMake >= 3.5
- PCL >= 1.8
- pthreads
- Eigen (transitively via PCL)

## build

### With pixi (recommended)

[pixi](https://pixi.sh) brings the full toolchain — PCL, Eigen, GoogleTest, Google Benchmark.

```bash
pixi run test        # build + run tests
pixi run bench       # build + run kNN benchmarks
pixi build           # produce a conda package
```

### Manually

```bash
cmake -B build -S . -DBUILD_TESTING=ON -DBUILD_EXAMPLES=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

build options:

| Option              | Default | What it enables                       |
| ------------------- | ------- | ------------------------------------- |
| `BUILD_TESTING`     | OFF     | GoogleTest suite (`ikd_tree_tests`)   |
| `BUILD_EXAMPLES`    | OFF     | The three demo executables            |
| `BUILD_BENCHMARKS`  | OFF     | Google Benchmark target               |

## Minimal usage

```cpp
#include <ikd_tree.h>

using Point = ikd_tree::ikdTree_PointType;
using Tree  = ikd_tree::KdTree<Point>;

Tree tree(/*delete_param=*/0.5f, /*balance_param=*/0.7f, /*downsample=*/0.2f);

Tree::PointVector cloud = /* ... fill in ... */;
tree.build(cloud);

Tree::PointVector knn;
std::vector<float> dists;
tree.nearest_search(Point(1.0f, 2.0f, 3.0f), /*k=*/10, knn, dists);

Tree::PointVector to_add = /* ... */;
tree.add_points(to_add, /*downsample_on=*/true);

Tree::PointVector to_del = /* ... */;
tree.delete_points(to_del);
```

The class is templated; explicit instantiations are provided for `ikdTree_PointType`,
`pcl::PointXYZ`, `pcl::PointXYZI`, and `pcl::PointXYZINormal`.

## Public API summary

| Method                                                     | Purpose                                   |
| ---------------------------------------------------------- | ----------------------------------------- |
| `build(cloud)`                                             | build the tree from a point cloud         |
| `add_points(pts, downsample_on)`                           | Incrementally insert points               |
| `add_point_boxes(boxes)`                                   | Re-validate soft-deleted points in region |
| `delete_points(pts)`                                       | Soft-delete points by coordinate          |
| `delete_point_boxes(boxes)`                                | Soft-delete all points in box regions     |
| `nearest_search(p, k, out_pts, out_dists, max_dist?)`      | k-NN search with optional distance cap    |
| `box_search(box, out_pts)`                                 | Points inside a box                       |
| `radius_search(p, r, out_pts)`                             | Points within radius `r` of `p`           |
| `flatten(out_pts)`                                         | All currently-live points                 |
| `acquire_removed_points(out_pts)`                          | Drains the deleted-points buffer          |
| `size()` / `validnum()`                                    | Total nodes / live point count            |
| `tree_range()`                                             | AABB of all live points                   |

## Examples

`examples/` contains three demos (built with `-DBUILD_EXAMPLES=ON`):

- `ikd_tree_demo` — speed benchmark with synthetic data
- `ikd_tree_search_demo` — box & radius search on a PCD point cloud
- `ikd_tree_async_demo` — visual demo of the background rebuild thread

The last two need
[HKU_demo_pointcloud.pcd](https://drive.google.com/file/d/1tMYiBIFn-fcjisaoIrmIKA09NICGG9KJ/view?usp=sharing)
in `materials/`.

## License

GPL-2.0. See [LICENSE](LICENSE).
