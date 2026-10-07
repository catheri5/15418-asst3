# Assignment 3 Partner Log

## Wire Representation and Baseline

- Replaced the starter representation with named `Point` keypoints

```cpp
struct Point {
  int x;
  int y;
};

struct Wire {
  Point pts[MAX_PTS_PER_WIRE];
  int num_pts;
  validate_wire_t to_validate_format() const;
};
```

- Up to five points represent the start, up to three bends, and the end
- `make_wire_from_points()` removes consecutive duplicate points when bends
  coincide with endpoints
- `to_validate_format()` copies the keypoints into the checker/output format
- Input reading stores endpoints; timed computation selects straight routes for
  collinear endpoints or horizontal-first one-bend routes otherwise, then fills occupancy
- Each iteration visits all wires: remove the old route, choose a random route
  with probability `P` or exhaustively find the best, then add the chosen route
- `-m W` has within-wires parallelism; `-m A` remains the sequential baseline

## Route Generation and Correctness

- Routes include straight paths, horizontal-first and vertical-first paths with
  at most two bends, and both three-bend orientations
- Three-bend intermediate points are strictly inside the endpoint bounding box
  so these candidates do not collapse into one- or two-bend routes
- For non-collinear endpoints, the candidate count is
  `dx + dy + 2 * (dx - 1) * (dy - 1)`, using absolute endpoint differences
- Occupancy is `occupancy[y][x]`; remove the current wire before scoring
- Each candidate cell contributes `(occupancy[y][x] + 1)^2`
- Segment traversal excludes its end so the next segment counts the shared
  bend; the final wire endpoint is included once
- Endpoints may run in either direction, so reversed segments must work too
- Keep the current route unless a strictly cheaper candidate is found
- `P` is fixed and random routes are chosen uniformly, not from a normal distribution
- Instructor clarification forbids early cutoff, prefix sums, and reused
  route-cost computations

## Recent Baseline Changes and Why

These changes currently apply only to `-m A`; W mode was preserved.

- **Immediate scoring:** `try_routes_sequential()` generates each candidate and
  passes it directly to `try_candidate_route()`, avoiding candidate-vector
  allocation and storage
- **Best-route update:** `try_candidate_route()` fully scores one candidate and
  updates the best wire and cost together through references
- **Simpler traversal:** `calculate_wire_cost_baseline()` separates horizontal
  and vertical segments and changes only one coordinate per inner loop
  Horizontal segments use contiguous row access; vertical segments still cross
  rows but have simpler loop control
- **Random selection:** `count_routes()` and `choose_random_route()` construct
  only the uniformly selected route without generating all candidates
- W still uses `generate_routes()` and `calculate_wire_cost_minimal()`

## Current Within-Wires Approach

- Keep iterations and the outer wire loop sequential
- Remove the current wire, then generate its candidates into a vector
- Score sequentially below `threshold_multiplier * num_threads`; otherwise
  distribute candidate scoring with OpenMP
- Each thread accumulates a private best route and cost, then the main thread
  combines the per-thread results
- This is a manual minimum reduction, similar to accumulating partial sums:
  compare locally during scoring and synchronize only at the end
- Occupancy stays read-only during scoring; removal and insertion occur outside
  the parallel region
- Current settings: `schedule(static, 8)`, threshold multiplier 4
- Static/dynamic, threshold, and chunk-size tests showed only modest differences
  These settings are provisional, not proven optimal
- Scaling can be limited by sequential route generation, repeated parallel
  regions, available candidates per wire, and occupancy memory access
- Parallel tie-breaking is not deterministic across schedules/thread counts;
  equal-cost choices can lead to different later routes and final costs

## Experiment History

The tables below are preliminary within-wires measurements with `P=0`,
five iterations, and batch size 1. They timed only iterative rerouting,
excluding initial route placement and occupancy filling. They are useful for
tuning, but are not final benchmark results.

### Static vs Dynamic

Eight threads, threshold multiplier 8, chunk size 8.

| Input | Schedule | Init (s) | Compute (s) | Max occ | Cost | Validation |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| few | static | 0.0217492720 | 1.2637013040 | 2 | 29236 | passed |
| medium | static | 0.0276752920 | 4.6609008170 | 2 | 266415 | passed |
| few | dynamic | 0.0221345980 | 1.2671972500 | 2 | 29236 | passed |
| medium | dynamic | 0.0277004300 | 4.7129189950 | 2 | 266429 | passed |

Static was slightly faster, but the small difference does not establish a
strong scheduling advantage. Kept static for subsequent tuning.

### Threshold Multiplier

Medium input, eight threads, static scheduling, chunk size 8.

| Multiplier | Init (s) | Compute (s) | Max occ | Cost | Validation |
| ---: | ---: | ---: | ---: | ---: | --- |
| 2 | 0.0277333960 | 4.6140986040 | 2 | 266415 | passed |
| 4 | 0.0261910380 | 4.6017672650 | 2 | 266415 | passed |
| 8 | 0.0277792250 | 4.6910999480 | 2 | 266415 | passed |
| 16 | 0.0278667380 | 4.7385649950 | 2 | 266415 | passed |
| 32 | 0.0276994070 | 4.6931010690 | 2 | 266415 | passed |

Multiplier 4 was fastest in this sweep, but the spread was small. Route-count
instrumentation would help establish how often these thresholds actually change
which searches run in parallel.

### Static Chunk Size

Medium input, eight threads, threshold multiplier 4.

First pass:

| Chunk | Init (s) | Compute (s) | Max occ | Cost | Validation |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 0.0279142950 | 4.7243530490 | 2 | 266407 | passed |
| 4 | 0.0253644780 | 4.6544068280 | 2 | 266407 | passed |
| 8 | 0.0256665890 | 4.7059855210 | 2 | 266415 | passed |
| 16 | 0.0278095560 | 4.6888941160 | 2 | 266427 | passed |

Rerun:

| Chunk | Init (s) | Compute (s) | Max occ | Cost | Validation |
| ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 0.0263368050 | 4.6403938510 | 2 | 266407 | passed |
| 4 | 0.0256396570 | 4.8198056440 | 2 | 266407 | passed |
| 8 | 0.0277449480 | 4.7673641550 | -- | -- | not recorded |
| 16 | 0.0257447940 | 4.7818548620 | 2 | 266427 | passed |
| 32 | 0.0251177820 | 4.7497977470 | 2 | 266429 | passed |

The chunk-8 rerun output stopped after computation time, so its remaining
results are not recorded. Rankings changed between passes and no chunk was a
clear winner. Kept chunk size 8 as a reasonable middle-ground choice.

### Medium Scaling Snapshot

Static scheduling, threshold multiplier 4, chunk size 8.

| Threads | Init (s) | Compute (s) | Speedup | Max occ | Cost | Validation |
| ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 0.0253442170 | 22.3493856930 | 1.00x | 2 | 266471 | passed |
| 2 | 0.0251552650 | 12.4094447330 | 1.80x | 2 | 266423 | passed |
| 4 | 0.0281204470 | 7.3029963770 | 3.06x | 2 | 266441 | passed |
| 8 | 0.0278557520 | 4.6172252720 | 4.84x | 2 | 266429 | passed |

This shows useful but sublinear speedup. Sequential work, synchronization,
memory access, and workload size are possible limits, not yet separately measured.

### Baseline Performance Follow-Up

After immediate candidate scoring but before the new cost loop, the one-thread
A baseline took approximately 5.89 s on `few_wires.txt` with
`-p 0.1 -i 5 -b 1`. The handout's one-thread across-wires reference is 2.483 s.
This is a single exploratory timing. Timing after both optimizations is pending.

## Testing Notes

- All complete outputs in the tuning tables passed the occupancy checker
- Validation confirms occupancy matches the chosen wires; it does not establish
  optimal routing cost or good performance
- Check straight, one-, two-, and three-bend routes, reversed endpoints, and
  adjacent endpoints where three-bend candidates disappear
- Test `P=0`, `P=0.1`, and `P=1` to cover exhaustive and random selection
- Candidate counts and duplicate-free cell traversal help catch missing routes
  and double-counted bends
- Record initialization/computation time, validation, maximum occupancy, and cost
- Repeat timings before concluding that small differences are significant
- Final computation timing includes initial route placement and occupancy filling;
  validation and output remain outside the starter computation timer
- For final results, use `-p 0.1 -i 5 -b 1` on few, medium, and abundant inputs
  with 1, 2, 4, and 8 threads for both modes

## Across-Wires Plan

Across-wires parallelism is still pending; the current A branch is sequential.

- Follow the assignment's batch model, with batch size supplied by `-b`
- Start with a simple correct synchronization strategy for occupancy access and updates
- Then compare batch size, scheduling, and global vs row/tile lock granularity
- Longest-task-first ordering may improve balance when wire searches differ greatly
- Locality-aware batching by midpoint or quadrant is a potential experiment
  Nearby wires may reuse data but also overlap and contend more, so sorting is
  worth trying only if its benefit outweighs overhead

## Useful Lecture Ideas

- Expose at least as many independent tasks as threads, preferably more
- Balance long tasks while keeping scheduling overhead under control
- Generate work in parallel if sequential work creation becomes a bottleneck
- Favor nearby data access where possible and measure cache behavior
- Improve useful work per load when permitted by the assignment
- Refine lock granularity only when contention is measured
- Custom distributed queues, work stealing, and lock-free deques are later options,
  not prerequisites for the initial OpenMP implementation

## Potential Improvements

1. Port the measured A baseline improvements to W mode as a separate change
   An indexed candidate loop could generate and score routes in parallel without
   storing them, while retaining per-thread bests
2. Reserve capacity in W's existing route vector as a smaller interim change
   Use the candidate-count formula above
3. Consider flat occupancy storage to remove row-vector indirection
   Output/checker conversion would be needed, and vertical access would still be strided
4. Add deterministic tie-breaking by candidate index for cleaner comparisons
   across schedules and thread counts
5. Instrument route counts, sequential/parallel search frequency, and time spent
   generating, scoring, and updating before further tuning
6. Evaluate across-wires batch size, lock granularity, and locality-aware ordering
   after implementing a correct parallel baseline

## Next Steps

1. Rebuild and benchmark the updated sequential A baseline
2. Revisit W only after checking the baseline changes
3. Implement and measure across-wires parallelism

```bash
make -B
./wireroute -f inputs/timeinput/few_wires.txt -n 1 -p 0.1 -i 5 -m A -b 1
```
