# Assignment 3 Implementation Roadmap

Implement and benchmark one stage at a time. Keep the sequential baseline
correct and easy to compare against while adding parallelism.

## 1. Lock Down Baseline Correctness

- Use the keypoint `Wire` representation:
  - `Point pts[MAX_PTS_PER_WIRE]`
  - `num_pts`
  - `to_validate_format()` converts to the checker/output format.
- Initialize each wire to a legal route:
  - straight route if endpoints share a row or column
  - otherwise horizontal-first one-bend route
- Build the occupancy matrix by adding every initial wire once.
- Generate all legal candidate routes for a wire:
  - straight route for collinear endpoints
  - horizontal-first and vertical-first routes with at most two bends
  - exactly three-bend routes through strictly interior points
- For each reroute decision:
  - remove the current wire from occupancy
  - score current and candidate routes using `(occupancy[y][x] + 1)^2`
  - keep the lowest-cost route, preserving the old route on ties
  - add the selected route back to occupancy
- For now, both `-m W` and `-m A` may call the same sequential baseline.

Expected result: `Validate Passed: no mismatches.`

Before moving on, also test `-m A` with the same inputs to confirm both modes
still share the correct baseline behavior.

## 2. Implement Within-Wires Parallelism First

Plan:

- For one wire, generate all candidate routes.
- If the number of routes is small, score sequentially to avoid OpenMP overhead.
- If the number of routes is large, score candidates in parallel:

```cpp
#pragma omp parallel for schedule(dynamic, chunk_size)
for (int r = 0; r < routes.size(); r++) {
  costs[r] = calculate_wire_cost_minimal(routes[r], occupancy);
}
```

- Reduce to the best route after scoring.
- Apply one occupancy update for the selected route.

Things to experiment with:

- Threshold for switching from sequential to parallel route scoring.
- `schedule(static)`.
- `schedule(dynamic, 1)`.
- `schedule(dynamic, 8)`, `16`, or `32`.
- `schedule(guided)`.

Design goal:

- Expose more independent route-scoring tasks than the target thread count.
- On GHC, the required measurements use 1, 2, 4, and 8 threads, so the first
  practical goal is enough route work to keep 8 threads busy.
- On PSC, the required scaling reaches 128 threads, so later experiments need
  much more available work.

Cache/locality notes:

- Horizontal occupancy access is more cache-friendly because `occupancy[y][x]`
  stores each row contiguously.
- Vertical segments jump between rows, so they may have worse locality.
- Candidate routes for the same wire touch the same bounding box, so
  within-wires may have better locality than arbitrary across-wires scheduling.
- Later possible optimization: use a flat occupancy array
  `occupancy[y * dim_x + x]` if cache miss data suggests row-vector overhead or
  locality problems.

## 3. Measure Within-Wires

Collect correctness and timing before making the design more complex.

Inputs to start with:

```bash
inputs/timeinput/few_wires.txt
inputs/timeinput/medium_wires.txt
inputs/timeinput/abundant_wires.txt
```

Measurements needed on GHC:

- 1, 2, 4, 8 threads
- total time
- computation time
- final maximum occupancy
- final total cost
- cache misses with `perf stat -e cache-misses`

Watch for:

- small wires producing too few route tasks
- OpenMP overhead dominating on small inputs
- route generation cost becoming significant
- non-ideal speedup from memory bandwidth/cache misses

## 4. Implement Across-Wires Baseline

Across-wires parallelism is less independent because route choices mutate the
shared occupancy matrix. Expect more tradeoffs between speed, synchronization,
staleness, and routing quality.

Required shape:

- Tasks are batches of wires.
- Batch size comes from `-b`.
- A worker processes wires in its batch sequentially.
- Occupancy updates for the batch happen according to the assignment's batching
  model.

Initial simple design:

```cpp
#pragma omp parallel for schedule(dynamic, 1)
for (int batch_start = 0; batch_start < num_wires; batch_start += batch_size) {
  process one batch
}
```

Start with coarse correctness, then refine synchronization.

## 5. Across-Wires Synchronization Experiments

Main knobs:

- Batch size `B`.
- OpenMP schedule and chunk size.
- Lock granularity for occupancy updates.
- Whether route decisions physically remove wires early or logically factor out
  the current wire during scoring.

Potential lock/update strategies:

- One global lock around occupancy updates.
  - simplest
  - likely high contention
- Row-level locks.
  - moderate complexity
  - may improve concurrency for wires in different rows
- Tile/block-level locks.
  - more complex
  - could reduce contention if wires are spatially separated
- Per-cell locking is probably too fine-grained unless measurements suggest a
  clear need.

Use increment/decrement updates rather than overwriting occupancy entries,
because other threads may update nearby or overlapping cells.

## 6. Across-Wires Scheduling Experiments

Candidate experiments:

- Dynamic scheduling over batches to reduce stragglers.
- Longest-task-first ordering based on bounding-box area or candidate-route
  count.
- Compare input order vs sorted-by-route-count order.
- Compare small `B` for freshness/load balance vs larger `B` for lower update
  overhead.

Potential locality experiment, not first implementation:

- Group wires by bounding-box midpoint or quadrant before batching.
- Treat long cross-region wires as large tasks and schedule them early.
- Hypothesis: spatial grouping may improve cache locality in occupancy reads
  and updates.
- Risk: nearby wires are also more likely to overlap, which can increase
  contention and reduce routing quality.
- Only pursue if cache miss data or synchronization behavior suggests locality
  is worth the extra work.

## 7. Report and Write-Up Data

Keep a running log of:

- design changes
- correctness status
- timing results
- cache miss results
- cost/max-occupancy results
- failed ideas and why they were rejected

For each final plot, preserve:

- exact input file
- command line
- thread count
- mode
- batch size
- annealing probability
- iteration count
- machine name

## Intentionally Deferred

Do not start with these:

- custom work-stealing deques
- lock-free queues
- spatial batching
- flat occupancy conversion
- complicated OpenMP task graphs
- PSC 128-thread tuning

These may become useful later, but the next useful step is a measured
within-wires implementation against the correct sequential baseline.

