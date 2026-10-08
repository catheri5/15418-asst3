# Assignment 3 Log

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
- Each iteration revisits all wires, choosing a uniform random route with
  probability `P` or exhaustively finding the best
- `-m W` parallelizes candidate routes for one wire at a time
- `-m A` now parallelizes batches of wires, with searches sequential inside each batch

## Route Generation and Correctness

- Routes include straight paths, horizontal-first and vertical-first paths with
  at most two bends, and both three-bend orientations
- Three-bend intermediate points are strictly inside the endpoint bounding box
  so these candidates do not collapse into one- or two-bend routes
- For non-collinear endpoints, the candidate count is
  `dx + dy + 2 * (dx - 1) * (dy - 1)`, using absolute endpoint differences
- W removes the current wire before scoring; each candidate cell contributes
  `(occupancy[y][x] + 1)^2`
- A leaves old routes in occupancy during batch selection and logically subtracts
  the current wire at cells on its old path before adding the candidate's +1
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
- **Both modes:** `route_from_index()` maps an index to one candidate, preserving
  the original enumeration order without storing all routes
  W generates/scores candidates in parallel; A generates/scores them sequentially
  inside each worker's batch
- **Exact route count:** `count_routes()` supplies the loop bounds and threshold
  decision without generating any candidates first
- **Axis-specific traversal:** cost helpers handle horizontal and vertical
  segments separately, changing only one coordinate per inner loop
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
- Per-thread result buffers allocate once before the iteration/wire loops
  Reset costs before each parallel search; active threads overwrite their route slots
- Selected settings after retuning: `schedule(static, 1)`, threshold multiplier 4
  The initial scalability snapshot below used chunk 8, not the selected chunk 1
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

### Step 2: Result-Buffer Reuse

Moved per-thread best-cost and best-route vector allocation outside the
iteration/wire loops. Costs reset to the current wire's starting best before
each parallel search, so unused thread slots cannot select stale results.
This reuses storage, not route-cost computations. The user observed no
meaningful computation-time change; retained because the change is simple,
not because a speedup was established.

### Tuning Methodology

The following sweeps use user-reported medians of three runs per configuration
on medium input, eight threads, `-m W -p 0.1 -i 5 -b 1`. Computation includes
initial route placement and occupancy filling. Initialization times and routing
statistics are the supplied outputs. All completed sweep outputs passed
validation and had maximum occupancy 3. Vary one knob at a time.

### Step 3: Static vs Dynamic

Chunk size 8, threshold multiplier 4.

| Schedule | Init (s) | Compute (s) | Cost | Validation |
| --- | ---: | ---: | ---: | --- |
| static | 0.0411107240 | 1.5045051730 | 267645 | passed |
| dynamic | 0.0426034640 | 1.5355032590 | 267681 | passed |

Kept static: about 2.0% less computation time. The advantage is modest, not
evidence of a decisive scheduling win. Static avoids dynamic work-dispatch
overhead, but this experiment does not isolate the cause of the difference.

### Step 4: Static Chunk Size

Static scheduling, threshold multiplier 4.

| Chunk | Init (s) | Compute (s) | Cost | Validation |
| ---: | ---: | ---: | ---: | --- |
| 1 | 0.0425755850 | 1.4861931990 | 267735 | passed |
| 4 | 0.0416819200 | 1.5134411650 | 267641 | passed |
| 8 | 0.0416265330 | 1.5096487180 | 267645 | passed |
| 16 | 0.0402956340 | 1.4991312180 | 267423 | passed |
| 32 | 0.0409317700 | 1.5020431010 | 267425 | passed |

Selected chunk 1: lowest measured median, about 1.6% less computation time
than chunk 8. The full spread is about 1.8%, so chunk size had little effect.
`static, 1` assigns candidates round-robin without a dynamic task queue.

For a single wire, candidates have the same Manhattan length and evaluate
the same number of cells. Different path lengths across wires do not explain
this W-mode chunk result. Candidate orientation, bend count, and cache access
can differ, but their performance effects were not isolated. Different costs
also reflect different tie-breaking trajectories, limiting direct comparisons.
Unchunked `schedule(static)` was suggested but has not been measured here.

### Step 5: Threshold Multiplier

Static scheduling, chunk size 1. At eight threads, multipliers 2, 4, 8, 16,
and 32 correspond to parallel thresholds of 16, 32, 64, 128, and 256 candidates.

| Multiplier | Init (s) | Compute (s) | Cost | Validation |
| ---: | ---: | ---: | ---: | --- |
| 2 | 0.0423112400 | 1.5172221380 | 267735 | passed |
| 4 | 0.0411906730 | 1.4862542700 | 267735 | passed |
| 8 | 0.0457532980 | 1.4884520000 | 267735 | passed |
| 16 | 0.0426234770 | 1.4891024920 | 267735 | passed |
| 32 | 0.0408274140 | 1.5063717380 | 267735 | passed |

The multiplier-8 computation value is a user-supplied correction to the
originally pasted 1.4844516550 s, not a recomputed median from raw runs.

Kept multiplier 4: lowest corrected median. Multipliers 4, 8, and 16 are
within about 0.2%; the entire sweep spans about 2.1%. These results do not
establish a strong threshold effect. All configurations returned the same cost.
The candidate-count threshold retains a sequential fallback for small searches.

An earlier threshold sweep had an unconfirmed chunk setting and is excluded
from this controlled comparison. A roughly 68-second multiplier-32 run is
recorded as an unexplained anomaly, separate from the representative result
above; its cause was not established.

### Final Tuning Decisions

- Keep static scheduling, chunk size 1, and threshold multiplier 4
- Keep reusable result buffers, with no measured speedup attributed to them
- Keep the sequential fallback based on exact candidate count
- Scheduling, chunk, and threshold differences were modest; do not overstate
  the precision of the winning settings
- These are medium-input tuning decisions; rerun scalability with the selected
  settings and evaluate few/abundant before treating them as general conclusions

### Tentative GHC W Performance

These are the later supplied W-mode timing outputs for `-p 0.1 -i 5 -b 1`.
All use 2048 x 2048 grids and passed validation. Compiled W knobs and repetition
counts still need confirmation; do not assume these are three-run medians.
The first abundant output omitted its thread-count line and is assumed to be
one thread from the supplied order. Times below retain the supplied precision;
speedups use each input's own one-thread baseline.

**few_wires.txt (128 wires)**

| Threads | Init (s) | Compute (s) | Compute speedup | Total speedup | Max occ | Cost |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0.0389524350 | 2.8215567840 | 1.00x | 1.00x | 2 | 29240 |
| 2 | 0.0438834310 | 1.5392203620 | 1.83x | 1.81x | 2 | 29240 |
| 4 | 0.0410573770 | 0.8890573200 | 3.17x | 3.08x | 2 | 29240 |
| 8 | 0.0439076010 | 0.5551978270 | 5.08x | 4.77x | 2 | 29240 |

**medium_wires.txt (2048 wires)**

| Threads | Init (s) | Compute (s) | Compute speedup | Total speedup | Max occ | Cost |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0.0456166490 | 9.3428048030 | 1.00x | 1.00x | 3 | 267723 |
| 2 | 0.0459864000 | 4.8679772060 | 1.92x | 1.91x | 3 | 267957 |
| 4 | 0.0455521940 | 2.6185937190 | 3.57x | 3.52x | 3 | 267771 |
| 8 | 0.0459823180 | 1.4779354030 | 6.32x | 6.16x | 3 | 267735 |

**abundant_wires.txt (16384 wires)**

| Threads | Init (s) | Compute (s) | Compute speedup | Total speedup | Max occ | Cost |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 (assumed) | 0.0616014830 | 24.3147525540 | 1.00x | 1.00x | 4 | 1539328 |
| 2 | 0.0624526220 | 12.6260013450 | 1.93x | 1.92x | 4 | 1539380 |
| 4 | 0.0620621000 | 6.7649289190 | 3.59x | 3.57x | 4 | 1538772 |
| 8 | 0.0614162910 | 3.7514132830 | 6.48x | 6.39x | 4 | 1539104 |

Eight-thread computation speedups are 5.08x, 6.32x, and 6.48x for few,
medium, and abundant respectively. No drop-off is observed through eight threads.
Final graphs, cache-miss data, and PSC results remain pending.

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

## Current Across-Wires Baseline

- One persistent OpenMP parallel region covers all annealing iterations
- Workers take batches; each batch contains up to `-b` wires
- Choose every route in the batch before committing any of that batch's updates
  No nested within-wire parallel search
- `cell_on_wire()` checks whether a candidate cell also lies on the old route
- `calculate_wire_cost_across()` fully scores candidates using
  `(occupancy_value - cell_on_old_route + 1)^2`
  This factors out only the current wire without physically removing it early
- Occupancy reads and increment/decrement updates are OpenMP atomics
  Concurrent readers can observe ongoing commits, as permitted by the handout
  No frozen per-iteration snapshot or extra delayed updates are introduced
- `add_wire_to_occupancy_across()` updates both raw occupancy views
  The views can temporarily differ during a concurrent commit but agree when
  all updates complete
- Each thread owns its RNG and reusable temporary batch-route vector
- The worksharing barrier completes all batches before the next iteration
- Partial final batches and batch sizes larger than the wire count are handled
- One-thread, batch-1 comparisons preserved the earlier baseline outputs
  Batch decisions were checked against an independent reference, including
  exhaustive/random selection and partial batches
- Atomic helper stress checks passed; full OpenMP scheduling/performance
  measurements on GHC are still pending

### Across-Wires Knobs

The chunk-size constant is in main's student-writable initialization area, beside W's knobs:

```cpp
const int A_WIRE_CHUNK_SIZE = 1;
```

- Switch `dynamic` to `static` directly in the A-mode worksharing pragma
- Chunk size is the number of batches per scheduling assignment, not wires
  Each batch still chooses/commits its own `-b` wires before moving to the next
- Batch size remains the runtime `-b` parameter; it controls delayed-update
  granularity and therefore may affect final routing quality
- The A loop uses `schedule(dynamic, A_WIRE_CHUNK_SIZE)`
  The earlier runtime-schedule setting was removed to match the W-style pragma
- No A threshold multiplier yet; the W route-count threshold is not an A knob
- A results can vary with work assignment, thread-specific RNG streams, and
  concurrent occupancy updates, not only equal-cost tie-breaking

### Next Experiments

Quick GHC commands from the synced `code` directory:

```bash
make -B
./wireroute -f inputs/debug/sample_8_8wires.txt -n 4 -p 0.1 -i 5 -m A -b 2
./wireroute -f inputs/debug/sample_8_8wires.txt -n 4 -p 0.1 -i 5 -m A -b 3
```

Expected: `Validate Passed: no mismatches.` Batch 3 checks the partial final
batch of an eight-wire input. These are correctness smoke tests, not meaningful
performance measurements. No user-run outcomes have been supplied yet.

1. Validate small inputs with 1/2/4/8 threads and batches 1/2/3/8
   Check partial final batches before focusing on runtime
2. Establish medium A timing at 1 and 8 threads, dynamic, chunk 1, `B=1`
   Use `-p 0.1 -i 5 -m A -b 1`, then compare static and dynamic at eight threads
   Use medians of three runs, keeping other knobs fixed
3. Keep the chosen schedule and chunk 1 while testing batch sizes 1/2/4/8/16
   Track cost and maximum occupancy as well as computation time
4. Then try chunk sizes 1/2/4/8 with batch size fixed
   Larger chunks may reduce scheduling overhead but leave fewer assignments
   Keep enough batches/tasks to occupy the requested threads
5. Only after measuring the baseline, consider update lock granularity,
   longest-work-first ordering, or locality-aware batching
   Nearby wires may reuse data but also overlap and contend more
6. Rebuild with `make -B` after editing constants or syncing code

For comparison with the handout reference, keep `-p 0.1 -i 5 -b 1`.
Batch-size tuning should be reported separately from that fixed-parameter baseline.

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

1. Consider a persistent parallel region for W to reduce repeated team-entry overhead
   A already uses one region across all iterations
   Preserve barriers around occupancy updates and per-wire result reduction
2. Measure route counts, sequential/parallel frequency, scoring/update time,
   and cache misses to identify the remaining bottleneck
3. Consider flat storage for each occupancy view to remove vector indirection
   Preserve output/checker compatibility and measure whether it actually helps
4. Add deterministic candidate-index tie-breaking for more controlled comparisons
5. Evaluate across-wires batch size, lock granularity, and locality-aware ordering
   after validating and measuring the new parallel baseline

## Next Steps

1. Run the quick A-mode GHC checks above; log actual outcomes when available
2. Measure and tune A scheduling, batch size, and chunk size one at a time
3. Confirm tentative W measurement settings and repeat counts before finalizing
4. Complete GHC graphs/cache measurements, routing images, A sensitivity, and PSC work

The roadmap has been refreshed to match the implemented indexed W search,
dual occupancy views, completed W tuning, and batched atomic A baseline.
Its old route-vector/reserve and sequential-A resume instructions are removed.

```bash
make -B
./wireroute -f inputs/timeinput/medium_wires.txt -n 8 -p 0.1 -i 5 -m A -b 1
```
