# Performance

Comparison of this fork against the [upstream HKU-MARS implementation][upstream] at the point this
fork's publish-readiness work began (commit `80ac978`).

[upstream]: https://github.com/hku-mars/ikd-Tree

Benchmark setup: Apple Silicon (M-series) macOS host, Google Benchmark, 3 repetitions,
`--benchmark_min_time=1s`, all reported runs with CV < 4 %. Reproducible via `pixi run bench`.

## Headlines

| Path | Δ vs upstream | Source of the change |
| --- | --- | --- |
| `KdTree` construction | **−6 % to −95 %** | Replaced 1 M-slot custom ring buffer with `std::deque` |
| `add_points` (large batches) | **−11 % to −37 %** | Simpler `Update()` after `Add_Point_Boxes` bug fix |
| `nearest_search` (steady-state) | ±5 % noise | No change |
| `nearest_search` (30 % deleted) | **+2 % to +11 %** | Cost of `Add_Point_Boxes` bug fix |

## Detailed numbers

### `build()` — constructor + initial build

| N | Upstream | This fork | Δ |
| ---: | ---: | ---: | ---: |
| 100 | 620 µs | 32 µs | **−95 %** |
| 1 000 | 923 | 350 | **−62 %** |
| 10 000 | 2 101 | 1 554 | **−26 %** |
| 100 000 | 18 781 | 17 640 | −6 % |

Upstream allocated a fixed 1 M-slot `Operation_Logger_Type` array per `KdTree` instance (~50 MB) in
the constructor. The fork uses a `std::deque` that starts empty and grows on demand. At small N the
constructor was 95 % allocator cost; that goes away.

### `nearest_search()` — steady state

Flat within the noise floor (±5 %). No regression at any measured (N, k):

| N \ k | 1 | 5 | 10 | 30 |
| ---: | ---: | ---: | ---: | ---: |
| 1 000 | 0.14 µs | 0.32 | 0.56 | 1.75 |
| 10 000 | 0.20 | 0.44 | 0.75 | 2.24 |
| 100 000 | 0.25 | 0.55 | 0.93 | 2.86 |

### `nearest_search()` — 30 % of points soft-deleted

Small regression (+2 % to +11 %): `Update()` now preserves the bounding box of deleted subtrees so
the recursion can still find them for revival via `add_point_boxes()`. Search pruning consequently
operates on slightly looser bounds.

| N \ k | 1 | 10 | 30 |
| ---: | ---: | ---: | ---: |
| 1 000 | +7 % | +0 % | −3 % |
| 10 000 | +6 % | +5 % | **+11 %** |
| 100 000 | +6 % | +5 % | +1 % |

This is the cost of the [`Add_Point_Boxes` correctness fix](include/ikd_tree/ikd_tree.h) — upstream's
recursion early-exited on the shrunken bounding box and never revived the deleted region.

### `add_points()` — batch insert into a 10 k-point tree

| Batch | Upstream | This fork | Δ |
| ---: | ---: | ---: | ---: |
| 100 | 378 µs | 376 µs | flat |
| 1 000 | 659 | 598 | −9 % |
| 5 000 | 2 668 | 1 810 | **−32 %** |

Larger batches trigger the async rebuild path, where the simplified `Update()` (no longer branching
on `tree_deleted` to decide whether to include children) saves enough work to dominate.

## Caveats

- All deltas are run-to-run measurements on a single machine. Different microarchitectures or
  compilers may shift them; the structural changes (no 50 MB up-front allocation, simpler `Update`)
  should generalize but the magnitudes won't.
- The "30 % deleted" benchmark configures the tree with high rebuild thresholds (`0.99 / 0.99`)
  to keep `Criterion_Check` from physically pruning the deleted nodes — otherwise the scenario
  degrades to the all-alive case.
- `nearest_search()` already received a [pruning-correctness fix and tighter
  bound](https://github.com/alvgaona/ikd-tree/commit/f81d07e) ahead of the publish-readiness work;
  those gains are baked into the "upstream" baseline above for an apples-to-apples comparison.

## Running the benchmarks yourself

```bash
pixi run bench
```

Source: [`bench/bench_ikd_tree.cpp`](bench/bench_ikd_tree.cpp).
