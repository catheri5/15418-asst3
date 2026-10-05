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

static Wire make_wire_from_points(std::initializer_list<Point> points) {
  Wire wire;
  wire.num_pts = 0;

  for (const Point &point : points) {
    if (wire.num_pts > 0 && 
        wire.pts[wire.num_pts - 1].x == point.x &&
        wire.pts[wire.num_pts - 1].y == point.y) {
      continue;
    }

    wire.pts[wire.num_pts++] = point;
  }

  return wire;
}

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

    while (x != end_x || y != end_y) {
      occupancy[y][x] += delta;
      x += dx;
      y += dy;
    }

    if (i == wire.num_pts - 1) {
      occupancy[y][x] += delta;
    }
  }
}

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
      routes.push_back(make_wire_from_points(
          {start, {x, start.y}, {x, y}, {end.x, y}, end}));
      routes.push_back(make_wire_from_points(
          {start, {start.x, y}, {x, y}, {x, end.y}, end}));
    }
  }

  return routes;
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
  // INITIAL ROUTE 
  for (auto &wire : wires) {
    int start_x, start_y, end_x, end_y;
    fin >> start_x >> start_y >> end_x >> end_y;
    wire.pts[0] = {start_x, start_y};

    // Handle straight line case (no duplicate)
    if (start_x == end_x || start_y == end_y) {
      wire.num_pts = 2;
      wire.pts[1] = {end_x, end_y};
    } else { // Initialize path to 1 bend
      wire.num_pts = 3;
      wire.pts[1] = {end_x, start_y}; // x axis first
      wire.pts[2] = {end_x, end_y};
    }

    add_wire_to_occupancy(wire, occupancy, 1);
  }

  /* Initialize any additional data structures needed in the algorithm */

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
  
  // initialize wires

  // Within wires
  if (parallel_mode == 'W') {
    
    for (int iter = 0; iter < SA_iters; iter++) {
      for (int w = 0; w < num_wires; w++) {
        Wire curr = wires[w];

        // remove wire from current occupancy matrix
        add_wire_to_occupancy(curr, occupancy, -1);

        Point start = curr.pts[0];
        Point end = curr.pts[curr.num_pts - 1];

        std::vector<Wire> routes = generate_routes(start, end);

        // baseline: compare against current wire route
        Wire best = curr;
        long long best_cost = calculate_wire_cost_minimal(curr, occupancy);

        // find lowest cost route in this iteration
        for (Wire &candidate : routes) {
          long long candidate_cost = 
            calculate_wire_cost_minimal(candidate, occupancy);

          if (candidate_cost < best_cost) { // preserve old route if tied
            best = candidate; 
            best_cost = candidate_cost;
          }
        }
        wires[w] = best;
        add_wire_to_occupancy(best, occupancy, 1);

      }
    }
  } else {
   
    // for now
    for (int iter = 0; iter < SA_iters; iter++) {
      for (int w = 0; w < num_wires; w++) {
        Wire curr = wires[w];

        // remove wire from current occupancy matrix
        add_wire_to_occupancy(curr, occupancy, -1);

        Point start = curr.pts[0];
        Point end = curr.pts[curr.num_pts - 1];

        std::vector<Wire> routes = generate_routes(start, end);

        // baseline: compare against current wire route
        Wire best = curr;
        long long best_cost = calculate_wire_cost_minimal(curr, occupancy);

        // find lowest cost route in this iteration
        for (Wire &candidate : routes) {
          long long candidate_cost = 
            calculate_wire_cost_minimal(candidate, occupancy);

          if (candidate_cost < best_cost) { // preserve old route if tied
            best = candidate; 
            best_cost = candidate_cost;
          }
        }
        wires[w] = best;
        add_wire_to_occupancy(best, occupancy, 1);

      }
    }
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
