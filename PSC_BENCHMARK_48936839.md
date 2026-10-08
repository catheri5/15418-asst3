# PSC benchmark data — job 48936839

These files preserve the completed Bridges-2 RM benchmark set for the report.

- `PSC_BENCHMARK_48936839_RAW.csv`: all 144 trials.
- `PSC_BENCHMARK_48936839_SUMMARY.csv`: medians and speedups for each input/mode/thread configuration.

The job used source commit `70c112e`, `p=0.1`, five simulated-annealing iterations, and batch size one. It ran on a full 128-core RM node. Every run passed validation (144/144).

For each input and parallel mode, speedups use that mode/input's one-thread median as the baseline. Total time is calculated for each trial as initialization plus computation, then the median is taken.
