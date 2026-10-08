/**
 * Parallel VLSI Wire Routing via OpenMP
 * Jason Tang (jasont2), Catherine Li (catheri5)
 */

#include "wireroute.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include <omp.h>
#include <unistd.h>


void print_stats(const std::vector<std::vector<int>> &occupancy) {
  int max_occupancy = 0;
  long long total_cost = 0;

  for (const auto &row : occupancy) {
    for (const int count : row) {
      max_occupancy = std::max(max_occupancy, count);
      total_cost += count * count;
    }
  }

  std::cout << "Max occupancy: " << max_occupancy << '\n';
  std::cout << "Total cost: " << total_cost << '\n';
}

/* This function write the output into 2 files
(1) It write occupancy grids into a file
(2) It convert wires from Wire to validate_wire_t by to_validate_format
(2) It write wires into another file
*/
void write_output(
    const std::vector<Wire> &wires, const int num_wires,
    const std::vector<std::vector<int>> &occupancy, const int dim_x,
    const int dim_y,
    std::string wires_output_file_path = "outputs/wire_output.txt",
    std::string occupancy_output_file_path = "outputs/occ_output.txt") {

  std::ofstream out_occupancy(occupancy_output_file_path, std::fstream::out);
  if (!out_occupancy) {
    std::cerr << "Unable to open file: " << occupancy_output_file_path << '\n';
    exit(EXIT_FAILURE);
  }
  out_occupancy << dim_x << ' ' << dim_y << '\n';

  for (const auto &row : occupancy) {
    for (size_t i = 0; i < row.size(); ++i)
      out_occupancy << row[i] << (i == row.size() - 1 ? "" : " ");
    out_occupancy << '\n';
  }
  out_occupancy.close();

  std::ofstream out_wires(wires_output_file_path, std::fstream::out);
  if (!out_wires) {
    std::cerr << "Unable to open file: " << wires_output_file_path << '\n';
    exit(EXIT_FAILURE);
  }

  out_wires << dim_x << ' ' << dim_y << '\n';
  out_wires << num_wires << '\n';

  for (const auto &wire : wires) {
    // NOTICE: we convert to keypoint representation here, using
    // to_validate_format which need to be defined in the bottom of this file
    validate_wire_t keypoints = wire.to_validate_format();
    for (int i = 0; i < keypoints.num_pts; ++i) {
      out_wires << keypoints.p[i].x << ' ' << keypoints.p[i].y;
      if (i < keypoints.num_pts - 1)
        out_wires << ' ';
    }
    out_wires << '\n';
  }

  out_wires.close();
}

/*** HELPER FUNCTIONS ****/

// Build a route from keypoints
static Wire make_wire_from_points(std::initializer_list<Point> points) {
  Wire wire;
  wire.num_pts = 0;

  for (const Point &point : points) {
    // Skip adjacent duplicates when a bend meets an endpoint
    if (wire.num_pts > 0 && 
        wire.pts[wire.num_pts - 1].x == point.x &&
        wire.pts[wire.num_pts - 1].y == point.y) {
      continue;
    }

    wire.pts[wire.num_pts++] = point;
  }

  return wire;
}

// Add or remove a wire using delta +1 or -1
void add_wire_to_occupancy(const Wire &wire,
                           std::vector<std::vector<int>> &occupancy,
                           int delta) {
  for (int i = 1; i < wire.num_pts; i++) {
    int x = wire.pts[i - 1].x;
    int y = wire.pts[i - 1].y;
    const int end_x = wire.pts[i].x;
    const int end_y = wire.pts[i].y;

    const int dx = (end_x > x) ? 1 : (end_x < x) ? -1 : 0;
    const int dy = (end_y > y) ? 1 : (end_y < y) ? -1 : 0;

    // Leave each bend for the next segment
    while (x != end_x || y != end_y) {
      occupancy[y][x] += delta;
      x += dx;
      y += dy;
    }

    if (i == wire.num_pts - 1) {
      // Include the final endpoint once
      occupancy[y][x] += delta;
    }
  }
}

// Fully score a candidate after removing the current wire
long long calculate_wire_cost_minimal(
    const Wire &wire,
    const std::vector<std::vector<int>> &occupancy) {

  long long cost = 0;

  for (int i = 1; i < wire.num_pts; i++) {
    int x = wire.pts[i - 1].x;
    int y = wire.pts[i - 1].y;
    int end_x = wire.pts[i].x;
    int end_y = wire.pts[i].y;

    // One segment only lies either vertically or horizontally
    int dx = (end_x > x) ? 1 : (end_x < x) ? -1 : 0;
    int dy = (end_y > y) ? 1 : (end_y < y) ? -1 : 0;

    while (x != end_x || y != end_y) {
      long long occ = occupancy[y][x] + 1;
      cost += occ * occ;
      x += dx;
      y += dy;
    }

    if (i == wire.num_pts - 1) {
      long long occ = occupancy[y][x] + 1;
      cost += occ * occ;
    }
  }

  return cost;
}

// Original vector-based route generator
std::vector<Wire> generate_routes(Point start, Point end) {
  std::vector<Wire> routes;

  if (start.x == end.x || start.y == end.y) {
    routes.push_back(make_wire_from_points({start, end}));
    return routes;
  }

  int min_x = std::min(start.x, end.x);
  int max_x = std::max(start.x, end.x);
  int min_y = std::min(start.y, end.y);
  int max_y = std::max(start.y, end.y);

  // Horizontal-first routes with at most two bends
  for (int x = min_x; x <= max_x; x++) {
    if (x == start.x) {
      continue;
    }
    routes.push_back(
      make_wire_from_points({start, {x, start.y}, {x, end.y}, end}));
  }

  // Vertical-first routes with at most two bends
  for (int y = min_y; y <= max_y; y++) {
    if (y == start.y) {
      continue;
    }
    routes.push_back(
      make_wire_from_points({start, {start.x, y}, {end.x, y}, end}));
  }

  // Exactly three-bend routes through each interior point
  for (int x = min_x + 1; x < max_x; x++) {
    for (int y = min_y + 1; y < max_y; y++) {
      // horizontal first
      routes.push_back(make_wire_from_points(
          {start, {x, start.y}, {x, y}, {end.x, y}, end}));
      // vertical first
      routes.push_back(make_wire_from_points(
          {start, {start.x, y}, {x, y}, {x, end.y}, end}));
    }
  }

  return routes;
}

// Keep row and column occupancy views in sync for the baseline
static void add_wire_to_occupancy_baseline(
    const Wire &wire,
    std::vector<std::vector<int>> &occupancy,
    std::vector<std::vector<int>> &occupancy_columns,
    int delta) {
  for (int i = 1; i < wire.num_pts; i++) {
    int x = wire.pts[i - 1].x;
    int y = wire.pts[i - 1].y;
    const int end_x = wire.pts[i].x;
    const int end_y = wire.pts[i].y;
    const int dx = (end_x > x) ? 1 : (end_x < x) ? -1 : 0;
    const int dy = (end_y > y) ? 1 : (end_y < y) ? -1 : 0;

    while (x != end_x || y != end_y) {
      occupancy[y][x] += delta;
      occupancy_columns[x][y] += delta;
      x += dx;
      y += dy;
    }

    if (i == wire.num_pts - 1) {
      occupancy[y][x] += delta;
      occupancy_columns[x][y] += delta;
    }
  }
}

// Score baseline segments along one axis at a time (better locality)
static long long calculate_wire_cost_baseline(
    const Wire &wire,
    const std::vector<std::vector<int>> &occupancy,
    const std::vector<std::vector<int>> &occupancy_columns) {
  long long cost = 0;

  for (int i = 1; i < wire.num_pts; i++) {
    const Point start = wire.pts[i - 1];
    const Point end = wire.pts[i];

    if (start.y == end.y) {
      // Keep the row fixed for horizontal access
      const auto &row = occupancy[start.y];
      // Shift reversed segment bounds to exclude its end
      const int offset = start.x > end.x ? 1 : 0;
      const int first = std::min(start.x, end.x) + offset;
      const int limit = std::max(start.x, end.x) + offset;
      for (int x = first; x < limit; x++) {
        const long long occ = row[x] + 1;
        cost += occ * occ;
      }
    } else {
      // Read consecutive cells from the column view
      const auto &column = occupancy_columns[start.x];
      const int offset = start.y > end.y ? 1 : 0;
      const int first = std::min(start.y, end.y) + offset;
      const int limit = std::max(start.y, end.y) + offset;
      for (int y = first; y < limit; y++) {
        const long long occ = column[y] + 1;
        cost += occ * occ;
      }
    }
  }

  // Include the final endpoint once
  if (wire.num_pts > 1) {
    const Point end = wire.pts[wire.num_pts - 1];
    const long long occ = occupancy[end.y][end.x] + 1;
    cost += occ * occ;
  }

  return cost;
}

// Score one candidate and update best route and cost together
static void try_candidate_route(
    const Wire &candidate,
    const std::vector<std::vector<int>> &occupancy,
    const std::vector<std::vector<int>> &occupancy_columns,
    Wire &best, long long &best_cost) {
  const long long cost = calculate_wire_cost_baseline(
      candidate, occupancy, occupancy_columns);
  // Keep the existing best on ties
  if (cost < best_cost) {
    best_cost = cost;
    best = candidate;
  }
}

// Count legal routes for uniform random selection
static long long count_routes(Point start, Point end) {
  const long long dx = std::abs(end.x - start.x);
  const long long dy = std::abs(end.y - start.y);
  if (dx == 0 || dy == 0) {
    return 1;
  }
  // At most two bends plus both three bend orientations
  return dx + dy + 2 * (dx - 1) * (dy - 1);
}

// Generate and score candidates without storing them
[[maybe_unused]] static void try_routes_sequential(
    Point start, Point end,
    const std::vector<std::vector<int>> &occupancy,
    const std::vector<std::vector<int>> &occupancy_columns,
    Wire &best, long long &best_cost) {

  if (start.x == end.x || start.y == end.y) {
    try_candidate_route(make_wire_from_points({start, end}),
                        occupancy, occupancy_columns, best, best_cost);
    return;
  }

  int min_x = std::min(start.x, end.x);
  int max_x = std::max(start.x, end.x);
  int min_y = std::min(start.y, end.y);
  int max_y = std::max(start.y, end.y);

  // Horizontal-first routes with at most two bends
  for (int x = min_x; x <= max_x; x++) {
    if (x == start.x) {
      continue;
    }
    try_candidate_route(
        make_wire_from_points({start, {x, start.y}, {x, end.y}, end}),
        occupancy, occupancy_columns, best, best_cost);
  }

  // Vertical-first routes with at most two bends
  for (int y = min_y; y <= max_y; y++) {
    if (y == start.y) {
      continue;
    }
    try_candidate_route(
        make_wire_from_points({start, {start.x, y}, {end.x, y}, end}),
        occupancy, occupancy_columns, best, best_cost);
  }

  // Exactly three-bend routes through each interior point
  for (int x = min_x + 1; x < max_x; x++) {
    for (int y = min_y + 1; y < max_y; y++) {
      // Horizontal first
      try_candidate_route(make_wire_from_points(
          {start, {x, start.y}, {x, y}, {end.x, y}, end}),
          occupancy, occupancy_columns, best, best_cost);
      // Vertical first
      try_candidate_route(make_wire_from_points(
          {start, {start.x, y}, {x, y}, {x, end.y}, end}),
          occupancy, occupancy_columns, best, best_cost);
    }
  }
}

// Construct one route in the original candidate order
static Wire route_from_index(Point start, Point end, long long choice) {
  if (start.x == end.x || start.y == end.y) {
    return make_wire_from_points({start, end});
  }

  const int min_x = std::min(start.x, end.x);
  const int min_y = std::min(start.y, end.y);
  const long long dx = std::abs(end.x - start.x);
  const long long dy = std::abs(end.y - start.y);

  // First dx choices are horizontal first
  if (choice < dx) {
    const int x = min_x + (start.x == min_x ? 1 : 0) + choice;
    return make_wire_from_points({start, {x, start.y}, {x, end.y}, end});
  }
  choice -= dx;
  // Next dy choices are vertical first
  if (choice < dy) {
    const int y = min_y + (start.y == min_y ? 1 : 0) + choice;
    return make_wire_from_points({start, {start.x, y}, {end.x, y}, end});
  }
  choice -= dy;

  // Two choices per interior point for the two orientations
  const int x = min_x + 1 + (choice / 2) / (dy - 1);
  const int y = min_y + 1 + (choice / 2) % (dy - 1);
  if (choice % 2 == 0) {
    return make_wire_from_points(
        {start, {x, start.y}, {x, y}, {end.x, y}, end});
  }
  return make_wire_from_points(
      {start, {start.x, y}, {x, y}, {x, end.y}, end});
}

// Construct one uniformly chosen route
static Wire choose_random_route(Point start, Point end, std::mt19937 &rng) {
  std::uniform_int_distribution<long long> random_route_dist(
      0, count_routes(start, end) - 1);
  return route_from_index(start, end, random_route_dist(rng));
}

// Check whether a cell contains the current wire's contribution
static bool cell_on_wire(const Wire &wire, int x, int y) {
  for (int i = 1; i < wire.num_pts; i++) {
    const Point start = wire.pts[i - 1];
    const Point end = wire.pts[i];
    if (start.y == end.y) {
      if (y == start.y && x >= std::min(start.x, end.x) &&
          x <= std::max(start.x, end.x)) {
        return true;
      }
    } else if (x == start.x && y >= std::min(start.y, end.y) &&
               y <= std::max(start.y, end.y)) {
      return true;
    }
  }
  return false;
}

struct OverlapRange {
  int first;
  int limit;
};

// Find and merge old-wire overlap ranges for one candidate segment
static int find_segment_overlaps(
    const Wire &current, bool horizontal, int fixed, int first, int limit,
    OverlapRange *overlaps) {
  int count = 0;
  for (int i = 1; i < current.num_pts; i++) {
    const Point start = current.pts[i - 1];
    const Point end = current.pts[i];
    const int start_pos = horizontal ? start.x : start.y;
    const int end_pos = horizontal ? end.x : end.y;
    const int start_fixed = horizontal ? start.y : start.x;
    const int end_fixed = horizontal ? end.y : end.x;
    int overlap_first, overlap_limit;

    if (start_fixed == end_fixed) {
      if (fixed != start_fixed) continue;
      overlap_first = std::min(start_pos, end_pos);
      overlap_limit = std::max(start_pos, end_pos) + 1;
    } else {
      // A perpendicular crossing occupies one cell on this segment
      if (fixed < std::min(start_fixed, end_fixed) ||
          fixed > std::max(start_fixed, end_fixed)) {
        continue;
      }
      overlap_first = start_pos;
      overlap_limit = start_pos + 1;
    }

    overlap_first = std::max(first, overlap_first);
    overlap_limit = std::min(limit, overlap_limit);
    if (overlap_first < overlap_limit) {
      overlaps[count++] = {overlap_first, overlap_limit};
    }
  }

  // Insertion sort for at most four overlap ranges
  for (int i = 1; i < count; i++) {
    const OverlapRange range = overlaps[i];
    int j = i;
    while (j > 0 && overlaps[j - 1].first > range.first) {
      overlaps[j] = overlaps[j - 1];
      j--;
    }
    overlaps[j] = range;
  }
  int merged = 0;
  for (int i = 0; i < count; i++) {
    // Shared bends and adjacent ranges must subtract the old wire only once
    if (merged > 0 && overlaps[i].first <= overlaps[merged - 1].limit) {
      overlaps[merged - 1].limit =
          std::max(overlaps[merged - 1].limit, overlaps[i].limit);
    } else {
      overlaps[merged++] = overlaps[i];
    }
  }
  return merged;
}

// Fully score without changing occupancy until the batch commits
static long long calculate_wire_cost_across(
    const Wire &candidate, const Wire &current,
    const std::vector<std::vector<int>> &occupancy,
    const std::vector<std::vector<int>> &occupancy_columns) {
  long long cost = 0;

  for (int i = 1; i < candidate.num_pts; i++) {
    const Point start = candidate.pts[i - 1];
    const Point end = candidate.pts[i];
    const bool horizontal = start.y == end.y;
    const int start_pos = horizontal ? start.x : start.y;
    const int end_pos = horizontal ? end.x : end.y;
    const int fixed = horizontal ? start.y : start.x;
    const auto &line = horizontal ? occupancy[fixed] : occupancy_columns[fixed];
    const int offset = start_pos > end_pos ? 1 : 0;
    const int first = std::min(start_pos, end_pos) + offset;
    const int limit = std::max(start_pos, end_pos) + offset;

    OverlapRange overlaps[MAX_PTS_PER_WIRE - 1];
    const int count = find_segment_overlaps(
        current, horizontal, fixed, first, limit, overlaps);
    int pos = first;
    for (int r = 0; r < count; r++) {
      // Outside the old route, adding the candidate contributes +1
      for (; pos < overlaps[r].first; pos++) {
        int value;
        #pragma omp atomic read
        value = line[pos];
        const long long occ = value + 1LL;
        cost += occ * occ;
      }

      // On the old route, its -1 and the candidate's +1 cancel
      for (; pos < overlaps[r].limit; pos++) {
        int value;
        #pragma omp atomic read
        value = line[pos];
        const long long occ = value;
        cost += occ * occ;
      }
    }

    for (; pos < limit; pos++) {
      int value;
      #pragma omp atomic read
      value = line[pos];
      const long long occ = value + 1LL;
      cost += occ * occ;
    }
  }

  // Include the final endpoint once
  if (candidate.num_pts > 1) {
    const Point end = candidate.pts[candidate.num_pts - 1];
    int value;
    #pragma omp atomic read
    value = occupancy[end.y][end.x];
    const long long occ = value + 1LL - cell_on_wire(current, end.x, end.y);
    cost += occ * occ;
  }
  return cost;
}

// Protect overlapping updates in both occupancy views
static void add_wire_to_occupancy_across(
    const Wire &wire,
    std::vector<std::vector<int>> &occupancy,
    std::vector<std::vector<int>> &occupancy_columns,
    int delta) {
  for (int i = 1; i < wire.num_pts; i++) {
    int x = wire.pts[i - 1].x;
    int y = wire.pts[i - 1].y;
    const int end_x = wire.pts[i].x;
    const int end_y = wire.pts[i].y;
    const int dx = (end_x > x) ? 1 : (end_x < x) ? -1 : 0;
    const int dy = (end_y > y) ? 1 : (end_y < y) ? -1 : 0;

    while (x != end_x || y != end_y) {
      #pragma omp atomic update
      occupancy[y][x] += delta;
      #pragma omp atomic update
      occupancy_columns[x][y] += delta;
      x += dx;
      y += dy;
    }

    if (i == wire.num_pts - 1) {
      #pragma omp atomic update
      occupancy[y][x] += delta;
      #pragma omp atomic update
      occupancy_columns[x][y] += delta;
    }
  }
}

struct RoutingParams {
  int num_threads;
  double SA_prob;
  int SA_iters;
  int batch_size;
};

// Initialize legal starting routes and build both occupancy views
static void initialize_routes(
    std::vector<Wire> &wires,
    std::vector<std::vector<int>> &occupancy,
    std::vector<std::vector<int>> &occupancy_columns) {
  for (auto &wire : wires) {
    const Point start = wire.pts[0];
    const Point end = wire.pts[1];

    if (start.x == end.x || start.y == end.y) {
      wire.num_pts = 2;
      wire.pts[1] = end;
    } else {
      wire.num_pts = 3;
      wire.pts[1] = {end.x, start.y}; // x axis first
      wire.pts[2] = end;
    }

    add_wire_to_occupancy_baseline(wire, occupancy, occupancy_columns, 1);
  }
}

// Explore candidates for one wire in parallel
static void route_within_wires(
    std::vector<Wire> &wires,
    std::vector<std::vector<int>> &occupancy,
    std::vector<std::vector<int>> &occupancy_columns,
    const RoutingParams &params) {
  // WITHIN WIRE OPTIMIZATION KNOBS
  const int W_WIRE_PARALLEL_THRESHOLD_MULTIPLIER = 4;
  const int W_WIRE_CHUNK_SIZE = 1;

  std::mt19937 rng(0);
  std::uniform_real_distribution<double> route_choice_dist(0.0, 1.0);
  const int num_wires = static_cast<int>(wires.size());

  // Reuse per-thread result buffers across wire searches
  const int nthreads = omp_get_max_threads();
  std::vector<long long> thread_best_cost(nthreads);
  std::vector<Wire> thread_best_route(nthreads);

  for (int iter = 0; iter < params.SA_iters; iter++) {
    for (int w = 0; w < num_wires; w++) {
      Wire curr = wires[w];

      // remove wire from current occupancy matrix
      add_wire_to_occupancy_baseline(curr, occupancy, occupancy_columns, -1);

      Point start = curr.pts[0];
      Point end = curr.pts[curr.num_pts - 1];

      // Count candidates without constructing or storing them
      const long long num_routes = count_routes(start, end);

      // baseline: compare against current wire route
      Wire best = curr;

      // Random route choosing with probability P
      if (route_choice_dist(rng) < params.SA_prob) {
        best = choose_random_route(start, end, rng);

      } else {
        long long best_cost = calculate_wire_cost_baseline(
            curr, occupancy, occupancy_columns);

        // check if above parallel threshold
        const long long threshold =
            1LL * W_WIRE_PARALLEL_THRESHOLD_MULTIPLIER * params.num_threads;
        if (num_routes >= threshold) { // execute in parallel

          // Reset costs so unused thread slots cannot retain old results
          std::fill(thread_best_cost.begin(), thread_best_cost.end(), best_cost);

          #pragma omp parallel 
          {
            int tid = omp_get_thread_num();
            long long local_best_cost = best_cost;
            Wire local_best_route = best;

            #pragma omp for schedule(static, W_WIRE_CHUNK_SIZE) 
            for (long long r = 0; r < num_routes; r++) {
              // Construct and fully score this thread's next candidate
              const Wire candidate = route_from_index(start, end, r);
              try_candidate_route(candidate, occupancy, occupancy_columns,
                                  local_best_route, local_best_cost);
            }

            thread_best_cost[tid] = local_best_cost;
            thread_best_route[tid] = local_best_route;
          }

          // update overall best route variables from local bests
          for (int t = 0; t < nthreads; t++) {
            if (thread_best_cost[t] < best_cost) {
              best_cost = thread_best_cost[t];
              best = thread_best_route[t];
            }
          }
          
        } else { // execute sequentially
          // find lowest cost route in this iteration
          for (long long r = 0; r < num_routes; r++) {
            const Wire candidate = route_from_index(start, end, r);
            try_candidate_route(candidate, occupancy, occupancy_columns,
                                best, best_cost);
          }
        }
      }
      wires[w] = best;
      add_wire_to_occupancy_baseline(best, occupancy, occupancy_columns, 1);

    }
  }
}

// Choose and commit batches of wires in parallel
static void route_across_wires(
    std::vector<Wire> &wires,
    std::vector<std::vector<int>> &occupancy,
    std::vector<std::vector<int>> &occupancy_columns,
    const RoutingParams &params) {
  // ACROSS WIRE OPTIMIZATION KNOBS
  // Change dynamic to static in the omp for to compare schedules
  // Batches per assignment, wires per batch still comes from -b
  const int A_WIRE_CHUNK_SIZE = 1;

  const int num_wires = static_cast<int>(wires.size());
  const int num_batches =
      num_wires / params.batch_size + (num_wires % params.batch_size != 0);

  // Each worker owns its RNG and temporary batch routes
  #pragma omp parallel
  {
    std::mt19937 thread_rng(omp_get_thread_num());
    std::uniform_real_distribution<double> thread_route_choice_dist(0.0, 1.0);
    std::vector<Wire> batch_routes(std::min(params.batch_size, num_wires));

    for (int iter = 0; iter < params.SA_iters; iter++) {
      #pragma omp for schedule(dynamic, A_WIRE_CHUNK_SIZE)
      for (int batch = 0; batch < num_batches; batch++) {
        const int batch_start = batch * params.batch_size;
        const int batch_count = std::min(params.batch_size, num_wires - batch_start);

        // Choose every route before applying this batch's updates
        for (int i = 0; i < batch_count; i++) {
          const Wire curr = wires[batch_start + i];
          const Point start = curr.pts[0];
          const Point end = curr.pts[curr.num_pts - 1];
          Wire best = curr;

          if (thread_route_choice_dist(thread_rng) < params.SA_prob) {
            best = choose_random_route(start, end, thread_rng);
          } else {
            long long best_cost = calculate_wire_cost_across(
                curr, curr, occupancy, occupancy_columns);
            const long long num_routes = count_routes(start, end);
            for (long long r = 0; r < num_routes; r++) {
              const Wire candidate = route_from_index(start, end, r);
              const long long cost = calculate_wire_cost_across(
                  candidate, curr, occupancy, occupancy_columns);
              if (cost < best_cost) {
                best_cost = cost;
                best = candidate;
              }
            }
          }
          batch_routes[i] = best;
        }

        // Commit the batch with atomic increments and decrements
        for (int i = 0; i < batch_count; i++) {
          const int w = batch_start + i;
          add_wire_to_occupancy_across(
              wires[w], occupancy, occupancy_columns, -1);
          add_wire_to_occupancy_across(
              batch_routes[i], occupancy, occupancy_columns, 1);
          wires[w] = batch_routes[i];
        }
      }
      // The worksharing barrier finishes all commits before the next iteration
    }
  }
}

int main(int argc, char *argv[]) {
  const auto init_start = std::chrono::steady_clock::now();

  std::string input_filename;
  int num_threads = 0;
  double SA_prob = 0.1;
  int SA_iters = 5;
  char parallel_mode = '\0';
  int batch_size = 1;

  int opt;
  while ((opt = getopt(argc, argv, "f:n:p:i:m:b:")) != -1) {
    switch (opt) {
    case 'f':
      input_filename = optarg;
      break;
    case 'n':
      num_threads = atoi(optarg);
      break;
    case 'p':
      SA_prob = atof(optarg);
      break;
    case 'i':
      SA_iters = atoi(optarg);
      break;
    case 'm':
      parallel_mode = *optarg;
      break;
    case 'b':
      batch_size = atoi(optarg);
      break;
    default:
      std::cerr << "Usage: " << argv[0]
                << " -f input_filename -n num_threads [-p SA_prob] [-i "
                   "SA_iters] -m parallel_mode -b batch_size\n";
      exit(EXIT_FAILURE);
    }
  }

  // Check if required options are provided
  if (empty(input_filename) || num_threads <= 0 || SA_iters <= 0 ||
      (parallel_mode != 'A' && parallel_mode != 'W') || batch_size <= 0) {
    std::cerr << "Usage: " << argv[0]
              << " -f input_filename -n num_threads [-p SA_prob] [-i SA_iters] "
                 "-m parallel_mode -b batch_size\n";
    exit(EXIT_FAILURE);
  }

  std::cout << "Number of threads: " << num_threads << '\n';
  std::cout << "Simulated annealing probability parameter: " << SA_prob << '\n';
  std::cout << "Simulated annealing iterations: " << SA_iters << '\n';
  std::cout << "Input file: " << input_filename << '\n';
  std::cout << "Parallel mode: " << parallel_mode << '\n';
  std::cout << "Batch size: " << batch_size << '\n';

  std::ifstream fin(input_filename);

  if (!fin) {
    std::cerr << "Unable to open file: " << input_filename << ".\n";
    exit(EXIT_FAILURE);
  }

  int dim_x, dim_y;
  int num_wires;

  /* Read the grid dimension and wire information from file */
  fin >> dim_x >> dim_y >> num_wires;

  std::vector<Wire> wires(num_wires);
  std::vector occupancy(dim_y, std::vector<int>(dim_x));
  std::cout << "Question Spec: dim_x=" << dim_x << ", dim_y=" << dim_y
            << ", number of wires=" << num_wires << '\n';

  // TODO (student code start): Read the wire information from file, 
  // you may need to change this if you define the wire structure differently.

  // Store endpoints during initialization. Initial route selection happens in
  // the computation phase so it is included in computation timing.
  for (auto &wire : wires) {
    int start_x, start_y, end_x, end_y;
    fin >> start_x >> start_y >> end_x >> end_y;
    wire.num_pts = 2;
    wire.pts[0] = {start_x, start_y};
    wire.pts[1] = {end_x, end_y};
  }

  /* Initialize any additional data structures needed in the algorithm */
  // Column view for contiguous vertical scoring in both modes
  std::vector<std::vector<int>> occupancy_columns(dim_x, std::vector<int>(dim_y));

  const RoutingParams routing_params{
      num_threads, SA_prob, SA_iters, batch_size};

  // Student code end
  const double init_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - init_start)
          .count();
  std::cout << "Initialization time (sec): " << std::fixed
            << std::setprecision(10) << init_time << '\n';

  const auto compute_start = std::chrono::steady_clock::now();

  /* TODO (student code start): Implement the wire routing algorithm here and
    feel free to structure the algorithm into different functions.
    Don't use global variables.
    Use OpenMP to parallelize the algorithm.
  */
  omp_set_num_threads(num_threads);
  
  initialize_routes(wires, occupancy, occupancy_columns);

  if (parallel_mode == 'W') {
    route_within_wires(wires, occupancy, occupancy_columns, routing_params);
  } else {
    route_across_wires(wires, occupancy, occupancy_columns, routing_params);
  }

  // Student code end
  // DON'T CHANGE THE FOLLOWING CODE
  const double compute_time =
      std::chrono::duration_cast<std::chrono::duration<double>>(
          std::chrono::steady_clock::now() - compute_start)
          .count();
  std::cout << "Computation time (sec): " << compute_time << '\n';

  /* wire to run check on wires and occupancy */
  wr_checker checker(wires, occupancy);
  checker.validate();

  /* Write wires and occupancy matrix to files */
  print_stats(occupancy);
  write_output(wires, num_wires, occupancy, dim_x, dim_y);
}

/* TODO (student): implement to_validate_format to convert Wire to
  validate_wire_t keypoint representation in order to run checker and
  write output /// DONE 
*/
validate_wire_t Wire::to_validate_format(void) const {
  validate_wire_t w;
  w.num_pts = num_pts;
  for (int i = 0; i < num_pts; i++) {
    w.p[i].x = pts[i].x;
    w.p[i].y = pts[i].y;
  }
  return w;
}
