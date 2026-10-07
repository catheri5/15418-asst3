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
- Remove the current wire before scoring; each candidate cell contributes
  `(occupancy[y][x] + 1)^2`
- Segment traversal excludes its end so the next segment counts the shared
  bend; the final wire endpoint is included once
- Endpoints may run in either direction, so reversed segments must work too
- Keep the current route unless a strictly cheaper candidate is found
- `P` is fixed and random routes are chosen uniformly, not from a normal distribution
- Instructor clarification forbids early cutoff, prefix sums, and reused
  route-cost computations

## Current Baseline Optimizations

- **Immediate scoring:** construct and fully score each candidate without storing
  the full candidate vector
- **A mode:** `try_routes_sequential()` generates candidates directly in loops
  and calls `try_candidate_route()` to update the best wire and cost
- **W mode:** `route_from_index()` maps an index to one candidate, preserving the
  original enumeration order and allowing generation and scoring in parallel
- **Exact route count:** `count_routes()` supplies the loop bounds and threshold
  decision without generating any candidates first
- **Axis-specific traversal:** `calculate_wire_cost_baseline()` handles horizontal
  and vertical segments separately, changing only one coordinate per inner loop
- **Contiguous access on both axes:** maintain `occupancy[y][x]` and a transposed
  `occupancy_columns[x][y]`; horizontal scoring reads a row, vertical scoring
  reads a column, and each insertion/removal updates both views
- The column view stores raw occupancy, not cached costs or partial sums
  It adds memory and update work in exchange for contiguous vertical reads
- **Random selection:** uniformly choose an index and construct only that route
- Candidates are still constructed as small `Wire` objects before scoring
  Deferring construction until a candidate became best was tried and was slower
- Legacy vector-generation and cost helpers remain in the file but are not used
  by the current W/A computation paths

## Current Within-Wires Approach

- Keep iterations and the outer wire loop sequential
- Remove the current wire from both occupancy views before searching
- Below `threshold_multiplier * num_threads`, generate and score sequentially
  Otherwise distribute indexed candidate generation and scoring with OpenMP
- Each thread accumulates a private best route and cost, then the main thread
  combines the per-thread results
- This is a manual minimum reduction, similar to accumulating partial sums:
  compare locally during scoring instead of synchronizing after every candidate
- Occupancy stays read-only during scoring; removal and insertion occur outside
  the parallel region
- Current settings: `schedule(static, 8)`, threshold multiplier 4
- These settings were carried over and need retesting with the new implementation
- Equal-cost tie-breaking can vary across schedules/thread counts, affecting
  later choices and final routing costs even when validation passes

## Current Experiment History

Old implementation timing and tuning tables have been removed. The following
measurements start the new implementation's experiment history.

### Step 1: Medium Scalability

User-reported GHC medians for `medium_wires.txt`: 2048 x 2048 grid, 2048 wires,
`-m W -p 0.1 -i 5 -b 1`, static chunk size 8, threshold multiplier 4.
Computation timing includes initial route placement and occupancy filling.
The initialization times and routing statistics below are the supplied outputs.

| Threads | Init (s) | Compute (s) | Total (s) | Compute speedup | Total speedup | Compute efficiency | Max occ | Cost | Validation |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 1 | 0.0456695970 | 9.5112553890 | 9.5569249860 | 1.00x | 1.00x | 100.0% | 3 | 267723 | passed |
| 2 | 0.0434853090 | 4.9418258640 | 4.9853111730 | 1.92x | 1.92x | 96.2% | 3 | 267701 | passed |
| 4 | 0.0417537090 | 2.6645043410 | 2.7062580500 | 3.57x | 3.53x | 89.2% | 3 | 267753 | passed |
| 8 | 0.0428917550 | 1.5063211130 | 1.5492128680 | 6.31x | 6.17x | 78.9% | 3 | 267645 | passed |

- Speedup is this implementation's one-thread time divided by its N-thread time
  Compute efficiency is compute speedup divided by thread count
- All four supplied outputs passed validation, with maximum occupancy 3
- Computation nearly halves from one to two threads; returns diminish at four
  and eight threads, but performance continues improving
- The handout's one-thread GHC within-wires medium reference is 8.240 s
  Our 9.511 s is about 15.4% slower; this is an absolute-time comparison,
  separate from scaling relative to our own one-thread run
- Sequential wire updates, repeated parallel regions/barriers, per-wire result
  allocation, and memory/cache effects are possible limits, not yet isolated
- Different final costs mean these are not perfectly identical routing trajectories
  Deterministic tie-breaking could make future tuning comparisons cleaner

## Testing Notes

- Validation confirms occupancy matches the chosen wires; it does not establish
  optimal routing cost or good performance
- Check straight, one-, two-, and three-bend routes, reversed endpoints, and
  adjacent endpoints where three-bend candidates disappear
- Test `P=0`, `P=0.1`, and `P=1` to cover exhaustive and random selection
- Check indexed enumeration against the original candidate order and verify that
  both occupancy views stay consistent after insertions/removals
- Record initialization/computation time, validation, maximum occupancy, and cost
- Repeat timings and compare medians before interpreting small differences
- Computation includes initial route placement and occupancy filling;
  validation and output remain outside the starter computation timer
- Final GHC coverage: `-p 0.1 -i 5 -b 1` on few, medium, and abundant inputs
  with 1, 2, 4, and 8 threads for both modes

## Across-Wires Plan

Across-wires parallelism is still pending; the current A branch is sequential.

- Follow the assignment's batch model, with batch size supplied by `-b`
- Start with a simple correct synchronization strategy for occupancy access and updates
- Both occupancy views must stay consistent when adding parallel updates
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

1. Retest static vs dynamic, chunk size, and threshold with the new implementation
   Keep other parameters fixed and use repeated runs
2. Reuse per-thread result buffers instead of allocating them for every wire
3. Consider a persistent parallel region to reduce repeated team-entry overhead
   Preserve barriers around occupancy updates and per-wire result reduction
4. Measure route counts, sequential/parallel frequency, scoring/update time,
   and cache misses to identify the remaining bottleneck
5. Consider flat storage for each occupancy view to remove vector indirection
   Preserve output/checker compatibility and measure whether it actually helps
6. Add deterministic candidate-index tie-breaking for more controlled comparisons
7. Evaluate across-wires batch size, lock granularity, and locality-aware ordering
   after implementing a correct parallel baseline

## Next Steps

1. Retune W's scheduling knobs using the current scalability snapshot as a baseline
2. Complete few/medium/abundant performance and cost evaluation
3. Implement and measure across-wires parallelism

```bash
make -B
./wireroute -f inputs/timeinput/medium_wires.txt -n 8 -p 0.1 -i 5 -m W -b 1
```
