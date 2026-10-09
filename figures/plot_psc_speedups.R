#!/usr/bin/env Rscript
# Render the PSC speedup figure from the saved benchmark summary CSV.

args <- commandArgs(trailingOnly = FALSE)
script_arg <- args[grep("^--file=", args)]
script_path <- sub("^--file=", "", script_arg[1])
root <- normalizePath(file.path(dirname(script_path), ".."))

summary_path <- file.path(root, "PSC_BENCHMARK_48936839_SUMMARY.csv")
output_pdf <- file.path(root, "figures", "psc_speedups.pdf")
output_png <- file.path(root, "figures", "psc_speedups.png")
data <- read.csv(summary_path, comment.char = "#")

threads <- c(1, 2, 4, 8, 16, 32, 64, 128)
input_names <- c("few", "medium", "abundant")
input_labels <- c("Few input", "Medium input", "Abundant input")
colors <- c("#0072B2", "#E69F00", "#009E73")

draw_panel <- function(mode, metric, title) {
  panel <- data[data$mode == mode, ]
  y_max <- max(panel[[metric]], threads) * 1.08
  plot(threads, threads, type = "n", log = "x", xaxt = "n", ylim = c(0, y_max),
       xlab = "Threads", ylab = "Speedup", main = title)
  axis(1, at = threads, labels = threads)
  abline(h = pretty(c(0, y_max)), col = "gray90", lwd = 0.8)
  lines(threads, threads, lty = 2, lwd = 1.2, col = "black")
  for (i in seq_along(input_names)) {
    values <- panel[panel$input == input_names[i], ]
    values <- values[order(values$threads), ]
    lines(values$threads, values[[metric]], type = "b", pch = 16, lwd = 1.8,
          col = colors[i])
  }
  legend("topleft", c(input_labels, "Ideal speedup"), col = c(colors, "black"),
         lty = c(1, 1, 1, 2), pch = c(16, 16, 16, NA), bty = "n", cex = 0.68)
}

render <- function(open_device) {
  open_device()
  par(mfrow = c(2, 2), mar = c(4.6, 4.2, 2.5, 0.8), oma = c(0, 0, 0, 0))
  draw_panel("W", "computation_speedup", "Within-wires: computation speedup")
  draw_panel("A", "computation_speedup", "Across-wires: computation speedup")
  draw_panel("W", "total_speedup", "Within-wires: total speedup")
  draw_panel("A", "total_speedup", "Across-wires: total speedup")
  dev.off()
}

render(function() pdf(output_pdf, width = 9, height = 6.7))
render(function() png(output_png, width = 1980, height = 1474, res = 220))
