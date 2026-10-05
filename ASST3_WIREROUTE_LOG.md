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
- Local bests are a manual reduction pattern: each thread accumulates its own
  best candidate independently, similar in spirit to per-thread partial sums,
  then the main thread combines the partial bests. This avoids locking or
  serializing on a shared current-best route during candidate scoring.
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

For early within-wires tuning, we temporarily measured only the iterative
rerouting phase, excluding initial route placement and occupancy initialization.
This narrower timing made it easier to isolate the effect of scheduling, chunk
size, and threshold choices. Final benchmark results should use the assignment
timing definition, where computation time includes initial route placement.

Early static/dynamic timing tables were removed because they used an outdated
timing/run setup. Those runs are useful only as a diagnostic and should not be
used for speedup or schedule conclusions.

Suggested first rerun for clean within-wires timing:

```bash
./wireroute -f inputs/timeinput/few_wires.txt -n 1 -p 0 -i 5 -m W -b 1
./wireroute -f inputs/timeinput/few_wires.txt -n 8 -p 0 -i 5 -m W -b 1
./wireroute -f inputs/timeinput/medium_wires.txt -n 1 -p 0 -i 5 -m W -b 1
./wireroute -f inputs/timeinput/medium_wires.txt -n 8 -p 0 -i 5 -m W -b 1
```

Record schedule, chunk size, threshold multiplier, total/computation time,
max occupancy, total cost, and validation status.

### Corrected W-Mode Static vs Dynamic Comparison

These runs were collected with the corrected timing/run setup.
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

### W-Mode Threshold Multiplier Sweep

Run settings:

- Input: `medium_wires.txt`
- Schedule: static
- Threads: `-n 8`
- Probability: `-p 0`
- Iterations: `-i 5`
- Batch size: `-b 1`

| Threshold multiplier | Init time (s) | Compute time (s) | Max occ | Total cost | Validation |
| ---: | ---: | ---: | ---: | ---: | --- |
| 2 | 0.0277333960 | 4.6140986040 | 2 | 266415 | passed |
| 4 | 0.0261910380 | 4.6017672650 | 2 | 266415 | passed |
| 8 | 0.0277792250 | 4.6910999480 | 2 | 266415 | passed |
| 16 | 0.0278667380 | 4.7385649950 | 2 | 266415 | passed |
| 32 | 0.0276994070 | 4.6931010690 | 2 | 266415 | passed |

Interpretation:

- Multiplier 4 was fastest in this run, but the total spread is small.
- Threshold multiplier does not appear to be a dominant knob for
  `medium_wires.txt` at 8 threads.
- Lower thresholds may help slightly by parallelizing more wires, but the effect
  is modest. This suggests many wires either already have enough routes to pass
  all thresholds or that OpenMP region overhead/memory behavior is dominating.
- Next useful test: check scaling across `-n 1,2,4,8` using a fixed multiplier
  such as 4, then compare against multiplier 8 only if scaling is poor.

### W-Mode Static Chunk Size Sweep

Run settings:

- Input: `medium_wires.txt`
- Schedule: static
- Threshold multiplier: 4
- Threads: `-n 8`
- Probability: `-p 0`
- Iterations: `-i 5`
- Batch size: `-b 1`

First pass:

| Chunk size | Init time (s) | Compute time (s) | Max occ | Total cost | Validation |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 0.0279142950 | 4.7243530490 | 2 | 266407 | passed |
| 4 | 0.0253644780 | 4.6544068280 | 2 | 266407 | passed |
| 8 | 0.0256665890 | 4.7059855210 | 2 | 266415 | passed |
| 16 | 0.0278095560 | 4.6888941160 | 2 | 266427 | passed |

Rerun after closing extra SSH/session noise:

| Chunk size | Init time (s) | Compute time (s) | Max occ | Total cost | Validation |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 0.0263368050 | 4.6403938510 | 2 | 266407 | passed |
| 4 | 0.0256396570 | 4.8198056440 | 2 | 266407 | passed |
| 8 | 0.0277449480 | 4.7673641550 | 2 | 266415 | passed |
| 16 | 0.0257447940 | 4.7818548620 | 2 | 266427 | passed |
| 32 | 0.0251177820 | 4.7497977470 | 2 | 266429 | passed |

Interpretation:

- Chunk size does not appear to be a major performance lever; most runs are
  within a few percent.
- Measurements are somewhat noisy.
- We decided to keep chunk size 8 for now because it is a stable middle-ground
  value and avoids overfitting to noisy single runs.
- The small cost differences are likely due to schedule/chunk-dependent
  tie-breaking among equal-cost routes.


### W-Mode Medium Scaling Snapshot

This table is for tuning/debugging only: it uses `P=0` and the narrower timing
region that excludes initial route placement/occupancy initialization. Final
report benchmark tables should use `P=0.1` and the assignment timing definition.

Run settings:

- Input: `medium_wires.txt`
- Schedule: static
- Threshold multiplier: 4
- Chunk size: 8
- Probability: `-p 0`
- Iterations: `-i 5`
- Batch size: `-b 1`

| Threads | Init time (s) | Compute time (s) | Speedup vs 1 thread | Max occ | Total cost | Validation |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 0.0253442170 | 22.3493856930 | 1.00x | 2 | 266471 | passed |
| 2 | 0.0251552650 | 12.4094447330 | 1.80x | 2 | 266423 | passed |
| 4 | 0.0281204470 | 7.3029963770 | 3.06x | 2 | 266441 | passed |
| 8 | 0.0278557520 | 4.6172252720 | 4.84x | 2 | 266429 | passed |

Interpretation:

- With the corrected timing/run setup (included everything specified in computation time), within-wires shows real speedup on the
  medium input.
- Speedup is meaningful but sublinear, likely due to repeated parallel-region
  overhead, sequential work outside route scoring, memory/cache behavior, and
  varying available route-level parallelism across wires.
- Costs differ slightly across thread counts, likely because parallel local-best
  reduction can choose different equal-cost routes. Deterministic tie-breaking
  by route index would make tuning comparisons cleaner.

Current run with benchmark settings yields ~6.6 s computation time (vs handout 2.528s), consider implementing further improvements specified at the end of log. 

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
- Collect first timing table for W mode on 1, 2, 4, 8 GHC threads.

## Potential Improvements To Resume Later (To Meet Good Performance Solution Benchmark)

Allowed by instructor guidance, because each candidate route would still be fully
evaluated:

1. Reserve route-vector capacity in `generate_routes`.
   - Use `dx + dy + 2 * (dx - 1) * (dy - 1)` for non-collinear endpoints.
   - This reduces repeated `std::vector` reallocations while preserving the same
     exhaustive route set.

2. Avoid storing all candidate routes.
   - Generate each candidate route and score it immediately.
   - For within-wires parallelism, a stronger version is `route_from_index(...)`:
     map route id -> candidate wire, score it fully, and keep a local best.
   - This removes route-vector allocation/storage overhead without reusing costs
     or skipping route evaluation.

3. Consider flat occupancy storage.
   - Instructor says internal occupancy representation may change as long as
     final output format is correct.
   - Replace `occupancy[y][x]` with a flat `occupancy[y * dim_x + x]` internally
     to reduce row-vector indirection and improve locality.
   - This is more invasive because validation/output currently expect
     `std::vector<std::vector<int>>`.

4. Add temporary instrumentation before more tuning.
   - Count total route searches, parallel route searches, sequential route
     searches, average routes per search, and max routes per search.
   - This will show whether threshold changes matter and how often the parallel
     path is actually used.

5. Re-run final within-wires GHC experiments.
   - Inputs: `few_wires.txt`, `medium_wires.txt`, `abundant_wires.txt`.
   - Threads: 1, 2, 4, 8.
   - Settings: `-p 0.1 -i 5 -m W -b 1`.
   - Record initialization time, computation time, max occupancy, total cost,
     validation status, and cache misses.

Not allowed (according to Ed):

- no early exit while scoring a route
- no prefix sums/range-query structures
- no reused route-cost computations
- no changing `P` across iterations
- no sampling candidate routes except for the specified random route branch
