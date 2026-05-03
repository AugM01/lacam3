/*
 * post processing, e.g., calculating solution quality
 */
#pragma once
#include "dist_table.hpp"
#include "instance.hpp"
#include "metrics.hpp"
#include "utils.hpp"

#include <zstd.h>

const int LACAM_LOG_VERSION = 1;

bool is_feasible_solution(const Instance &ins, const Solution &solution,
                          const int verbose = 0);
void print_stats(const int verbose, const Deadline *deadline,
                 const Instance &ins, const Solution &solution,
                 const double comp_time_ms);
void make_log(const Instance &ins, const bool flg_no_all, const Solution &solution,
              const std::string &output_name, const double comp_time_ms,
              const std::string &map_name, const int seed,
              const bool log_short = false  // true -> paths not appear
);

bool compress_file_zstd(const std::string& input_path, 
                        int compression_level,
                        bool delete_original);