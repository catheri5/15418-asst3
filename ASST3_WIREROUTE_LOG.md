# Assignment 3 Wireroute Log

Facts and implementation notes useful for partner handoff and the final
write-up.

## Environment

- Partners: Jason (`jasont2`) and Catherine (`catheri5`)
- Assignment: 15-418 Assignment 3, Parallel VLSI Wire Routing
- Primary debug target: GHC machines
- GHC experiment thread counts: 1, 2, 4, 8
- PSC experiment thread counts later: 1, 2, 4, 8, 16, 32, 64, 128

## Current Baseline Status

- Replaced the starter compressed route representation with a keypoint-based
  `Wire`:
  - `Point pts[MAX_PTS_PER_WIRE]`
  - `int num_pts`
- `to_validate_format()` copies keypoints into the checker/output
  `validate_wire_t` format.
- Input reading initializes every wire to a legal route:
  - straight route for collinear endpoints
  - otherwise horizontal-first one-bend route
- The occupancy matrix is filled during initialization by adding each initial
  wire route.
- Current deterministic baseline loop:
  - remove current wire from occupancy
  - generate all legal candidate routes
  - score current and candidate routes using `(occupancy + 1)^2`
  - choose the lowest-cost route, preserving current route on ties
  - update `wires[w]`
  - add the selected route back into occupancy
- For now, `-m W` and `-m A` both run the same sequential rerouting baseline.

## Route Generation

- `make_wire_from_points(...)` builds a `Wire` from keypoints and removes
  consecutive duplicate points.
- `generate_routes(start, end)` covers:
  - the only straight route when endpoints share a row or column
  - horizontal-first routes with at most two bends
  - vertical-first routes with at most two bends
  - exactly three-bend routes through strictly interior points
- Three-bend routes should not collapse to two-bend routes because the
  intermediate `(x, y)` is chosen strictly inside the endpoint bounding box.
- Some at-most-two-bend routes intentionally collapse to one-bend routes;
  duplicate-point cleanup handles this.

## Correctness Notes

- Occupancy is indexed as `occupancy[y][x]`.
- Route scoring assumes the current wire has already been removed from
  occupancy.
- Route cost uses `occupancy[y][x] + 1` because it is scoring the candidate as
  if this wire were added back.
- Segment traversal includes each bend point once and includes the final
  endpoint only on the final segment.
- Recommended debug assertions:
  - generated route has `2 <= num_pts <= MAX_PTS_PER_WIRE`
  - each segment is horizontal or vertical
  - occupancy does not become negative during removal

## Within-Wires

Within-wires is the current implementation target because route scoring for a
single wire is mostly read-only and naturally parallel across candidate routes.

Current approach:

- Keep the same sequential outer loop over wires.
- For each wire, generate candidate routes.
- Score routes in parallel only when there are enough candidates to amortize
  OpenMP overhead.
- Each OpenMP worker keeps a local best route/cost, then the main thread reduces
  the per-thread bests after the parallel loop.
- Current threshold experiment uses route count compared against
  `threshold_multiplier * num_threads`.
- Current schedule experiments focus on static vs dynamic, matching the lecture
  discussion.

Main questions to measure:

- How many candidate routes are needed before parallel scoring helps?
- Does `dynamic` scheduling help with route-cost imbalance?
- Does OpenMP overhead dominate small wires?
- How do cache misses change as thread count increases?

### Timing Data Note

Early static/dynamic timing tables were removed after code inspection showed
that `-n` was parsed but not yet passed to OpenMP with
`omp_set_num_threads(num_threads)`. Those runs are useful only as a diagnostic
and should not be used for speedup or schedule conclusions.

Rerun within-wires timing after rebuilding with the thread-count fix. Suggested
first rerun:

```bash
./wireroute -f inputs/timeinput/few_wires.txt -n 1 -p 0 -i 5 -m W -b 1
./wireroute -f inputs/timeinput/few_wires.txt -n 8 -p 0 -i 5 -m W -b 1
./wireroute -f inputs/timeinput/medium_wires.txt -n 1 -p 0 -i 5 -m W -b 1
./wireroute -f inputs/timeinput/medium_wires.txt -n 8 -p 0 -i 5 -m W -b 1
```

Record schedule, chunk size, threshold multiplier, total/computation time,
max occupancy, total cost, and validation status.

### Corrected W-Mode Static vs Dynamic Comparison

These runs were collected after adding `omp_set_num_threads(num_threads)`.
Computation time still excludes initial route placement/occupancy
initialization.

Run settings:

- Mode: `-m W`
- Threads: `-n 8`
- Probability: `-p 0`
- Iterations: `-i 5`
- Batch size: `-b 1`
- Threshold multiplier: 8
- Chunk size: 8

| Input | Schedule | Init time (s) | Compute time (s) | Max occ | Total cost | Validation |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `few_wires.txt` | static | 0.0217492720 | 1.2637013040 | 2 | 29236 | passed |
| `medium_wires.txt` | static | 0.0276752920 | 4.6609008170 | 2 | 266415 | passed |
| `few_wires.txt` | dynamic | 0.0221345980 | 1.2671972500 | 2 | 29236 | passed |
| `medium_wires.txt` | dynamic | 0.0277004300 | 4.7129189950 | 2 | 266429 | passed |

Interpretation:

- Static is slightly faster in this corrected comparison:
  - `few_wires.txt`: about 0.3 percent faster.
  - `medium_wires.txt`: about 1.1 percent faster.
- The difference is small, so schedule choice alone is not the dominant factor
  for within-wires performance at this setting.
- The small cost difference on `medium_wires.txt` with `P=0` is likely caused
  by schedule-dependent tie-breaking among equal-cost routes. If needed for
  cleaner comparisons, make the parallel reduction break ties by route index.

## Across-Wires

Across-wires is more complex because different threads update the shared
occupancy matrix.

Initial plan:

- Use assignment-required batches of wires.
- Start with dynamic scheduling over batches.
- Add synchronization around occupancy updates.
- Compare batch sizes for performance and routing quality.

Potential experiments:

- Global lock vs row locks vs tile locks.
- Batch size sensitivity.
- Longest-task-first ordering by bounding-box area or candidate-route count.
- Locality-aware batching by bounding-box midpoint/quadrant.

Locality-aware batching is only a candidate experiment. It may improve cache
locality, but nearby wires may contend more and produce worse routing quality.

## Lecture Themes to Reuse in Write-Up

- Implement the simplest correct version first, then measure.
- Expose more tasks than cores:
  - at least enough work for 8 GHC threads
  - eventually enough work for 128 PSC threads
- Balance workload while controlling scheduling overhead.
- Increase arithmetic intensity where possible.
- Reduce communication and contention.
- Use finer-grained locks only if coarse locking is measured as a bottleneck.
- Dynamic scheduling can help when tasks have uneven cost.
- Long tasks should not be left as final stragglers.

## Open Items

- Confirm baseline validation on small debug inputs.
- Add deterministic correctness results to this log.
- Implement within-wires route scoring.
- Collect first timing table for W mode on 1, 2, 4, 8 GHC threads.
- Decide whether simulated annealing random reroutes are needed before or after
  the first within-wires implementation.
