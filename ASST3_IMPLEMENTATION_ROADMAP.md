# Assignment 3 Implementation Roadmap

## Resume Here

- Baseline representation, route enumeration, and within-wires parallelism are implemented
- W tuning selected static scheduling, chunk 1, and threshold multiplier 4
- Tentative GHC W timing results for all three inputs at 1/2/4/8 threads are recorded
- A now has a batched OpenMP baseline with atomic occupancy reads/updates
- Next action: run the small GHC A checks below, then measure A before optimizing it
- Final graphs, cache-miss analysis, routing images, sensitivity studies, and PSC work remain

Detailed experiment numbers live in `ASST3_WIREROUTE_LOG.md`; the report contains
the design discussion, tuning summary, and tentative W performance tables.

## 1. Completed Baseline Work

- `Wire` stores up to five named `Point` keypoints plus `num_pts`
- `make_wire_from_points()` removes adjacent duplicate points
- `to_validate_format()` adapts wires to the checker/output format
- Input reading stores endpoints; computation constructs legal initial routes and fills occupancy
- Exhaustive enumeration includes both orientations and all legal routes up to three bends
- `count_routes()` gives exact loop bounds without constructing candidates
- `route_from_index()` constructs one candidate at a time in the original enumeration order
- Every candidate is scored fully; no early cutoff, prefix sums, or reused route costs
- Fixed `P` selects a uniformly random candidate in the random branch
- Axis-specific loops simplify traversal
- Row and column occupancy views give contiguous horizontal and vertical reads
- Updates maintain both views; each bend and the final endpoint are counted once

Do not spend time adding `reserve()` to the legacy route vector: current W/A
computation paths no longer store all candidate routes.

## 2. Current Within-Wires Design

- Iterations and the outer wire loop remain sequential
- Remove the old route from both occupancy views
- Generate and score candidates in parallel only above the route-count threshold
- Accumulate private thread bests and combine them after the parallel region
- Keep occupancy read-only during scoring and insert the chosen route afterward
- Per-thread result vectors allocate once and reset costs per search
- Buffer reuse showed no measurable improvement; do not attribute a speedup to it

Selected student-initialization constants:

```cpp
const int W_WIRE_PARALLEL_THRESHOLD_MULTIPLIER = 4;
const int W_WIRE_CHUNK_SIZE = 1;
```

The pragma is `schedule(static, W_WIRE_CHUNK_SIZE)`. A W chunk contains candidate
routes for the same wire, not different wires. Those candidates have equal
Manhattan length; chunk 1's small advantage is not evidence of wire-length balancing.

Static/dynamic, chunks 1/4/8/16/32, and multipliers 2/4/8/16/32 were retested
using medium input and medians of three runs. Differences were modest.
Equal-cost tie-breaking can change later routes and final cost.

## 3. Current Across-Wires Baseline

- One persistent parallel region covers all iterations
- Tasks are batches of up to `-b` wires
- Each worker searches its batch's wires sequentially, without nested W parallelism
- Choose all routes in a batch before applying that batch's updates
- During scoring, old routes remain in occupancy
- `cell_on_wire()` identifies the current wire's old contribution
- `calculate_wire_cost_across()` uses `(value - on_old_route + 1)^2`
- `add_wire_to_occupancy_across()` atomically removes old routes and inserts chosen routes
- Atomic reads avoid C++ data races with concurrent updates
- Both occupancy views are updated; they agree after commits complete
- Each worker owns its RNG and reusable batch-route buffer
- An end-of-worksharing-loop barrier finishes all commits before the next iteration
- Partial final batches and `B` larger than the wire count are handled

Helper correctness and batch-reference checks passed. Full OpenMP execution and
performance on GHC still need user-run checks; no A timing results have been supplied.

## 4. Quick GHC Check

From the `code` directory after syncing:

```bash
make -B
./wireroute -f inputs/debug/sample_8_8wires.txt -n 4 -p 0.1 -i 5 -m A -b 2
./wireroute -f inputs/debug/sample_8_8wires.txt -n 4 -p 0.1 -i 5 -m A -b 3
```

Expected: `Validate Passed: no mismatches.` The batch-3 case tests the partial
final batch because the input has eight wires. These are quick correctness
checks, not performance benchmarks. Outcomes are pending.

Before timing, expand small-input checks to 1/2/4/8 threads and batch sizes
1/2/3/8. Include `P=0`, `P=0.1`, and `P=1` to cover both route-choice branches.

## 5. Across-Wires Knobs and First Measurements

Constants beside the W knobs in main's student-writable initialization area:

```cpp
const omp_sched_t A_WIRE_SCHEDULE = omp_sched_dynamic;
const int A_WIRE_CHUNK_SIZE = 1;
```

`omp_set_schedule()` configures the calling thread before the parallel region.
Workers inherit it, and A's loop uses `schedule(runtime)`.

| Knob | Meaning | First values to test |
| --- | --- | --- |
| Schedule | How batches are assigned | `omp_sched_static`, `omp_sched_dynamic` |
| Chunk | Batches grouped per scheduling assignment | 1, 2, 4, 8 |
| `-b` | Wires chosen before each batch commits | 1, 2, 4, 8, 16 |

Chunk size and batch size are different: a chunk of four batches still commits
each batch separately. It does not delay all four batches' updates together.
No across-wires threshold multiplier has been introduced.

Recommended experiment order:

1. Establish an untuned A medium baseline at 1 and 8 threads, dynamic, chunk 1, `B=1`
2. Compare static vs dynamic at eight threads, keeping chunk 1 and `B=1`
3. Fix the schedule and chunk 1, then sweep `B=1/2/4/8/16`
4. Fix schedule and batch size, then sweep chunk 1/2/4/8
5. Check scaling at 1/2/4/8 threads with the chosen configuration

Use medium input, `P=0.1`, five iterations, and medians of three runs for tuning.
Track validation, occupancy, and routing cost as well as time. A results can
vary due to concurrent updates and which thread's RNG stream handles each wire.
Keep enough batches and assignments to occupy the requested threads.
Rebuild with `make -B` whenever constants change.

```bash
./wireroute -f inputs/timeinput/medium_wires.txt -n 8 -p 0.1 -i 5 -m A -b 1
```

For direct comparison with the handout reference, use `P=0.1`, five iterations,
and `B=1`. Keep a fixed-parameter baseline separate from batch-size experiments.

## 6. Later Across-Wires Experiments

- Measure scoring, update, scheduling, and synchronization costs before adding complexity
- Compare atomic updates with global/row/tile locking only if contention warrants it
- Preserve safe concurrent reads when changing synchronization
  Locking only writers does not make plain concurrent reads/writes race-free
- Use increment/decrement updates rather than overwriting occupancy values
- Explore longest-work-first ordering using route count times path length as a work estimate
- Compare original order against sorted order, including sorting overhead
- Try locality-aware batching by midpoint or quadrant as a measured experiment
- Handle long cross-region wires explicitly rather than dropping them
- Nearby wires can share cached data but can also overlap and contend more
- Keep the assignment's update timing; no frozen iteration snapshots or extra staleness

## 7. Finish the GHC and PSC Evaluation

The twelve tentative GHC W outputs cover few/medium/abundant at 1/2/4/8 threads.
Confirm their compiled knobs and repetition counts, and the first abundant run's
inferred one-thread label, before treating them as final results.

Remaining required deliverables:

- GHC: both W and A, all three timing inputs, 1/2/4/8 threads
- Total and computation speedup graphs using each configuration's own one-thread baseline
- GHC total and mean per-thread cache-miss plots with interpretation
- Medium-input routing/occupancy images at eight threads for both modes
- A probability sensitivity at 1/8 threads with `P=0.01/0.1/0.5`
- A problem-size sensitivity using `problemsize/gridsize` and `problemsize/numwires`
- PSC: both modes, all three timing inputs, 1/2/4/8/16/32/64/128 threads
- PSC total/computation speedup plots and comparison with GHC
- Final design discussion explaining observed limits, not just predicted bottlenecks

Record the input, exact command, compiled knobs, machine, repeat count, timing
definition, validation, cost, and occupancy for every result set.
Computation includes initial route placement and occupancy filling; printing,
validation, and writing output remain outside the starter computation timer.

## 8. Optional Improvements

- Persistent parallel region for W; A already has one
- Deterministic candidate-index tie-breaking for cleaner W comparisons
- Flat storage for both occupancy views, if indirection or cache data warrants it
- Instrument candidate counts and sequential/parallel search frequency

Custom distributed queues, work-stealing deques, lock-free structures, and
complicated task graphs remain deferred. Use the existing OpenMP baseline to
find a measured bottleneck first.

## Documentation

- Keep experiment history and handoff details in the partner log
- Add report material only when it supports the writeup prompts or measured conclusions
- Keep exploratory anomalies separate from representative measurements
- Preserve approximate historical timings as approximate; do not invent precision
