// NOTE: this file was modified from the original lacam3 repo
// (https://github.com/Kei18/lacam3, commit 1a269b7addc1edc4f1f56b194d307190f5491943 2025-03-09)
// Changes from original:
//   1. compress_file_zstd(): added. Compresses a file in-place using zstd and
//      optionally deletes the original. Used to reduce log file disk usage.
//   2. make_log(): extended with additional parameter logging (LaCAM planner flags,
//      run config, and solution metadata) for compatibility with the FICO-clean
//      experiment runner's log parser.

#include "../include/post_processing.hpp"
#include "../include/dist_table.hpp"
#include "../include/planner.hpp"

/**
 * Compress a file using zstd and optionally delete the original.
 * 
 * @param input_path Path to the uncompressed file
 * @param compression_level Compression level (1-22, recommended: 6)
 * @param delete_original If true, delete the original file after compression
 * @return true if compression succeeded, false otherwise
 */
bool compress_file_zstd(const std::string& input_path, 
                        int compression_level = 6,
                        bool delete_original = true) {
    std::string output_path = input_path + ".zst";
    
    // Open input file
    std::ifstream input(input_path, std::ios::binary);
    if (!input) {
        std::cerr << "Failed to open input file: " << input_path << std::endl;
        return false;
    }
    
    // Read entire file into memory
    input.seekg(0, std::ios::end);
    size_t input_size = input.tellg();
    input.seekg(0, std::ios::beg);
    
    std::vector<char> input_buffer(input_size);
    input.read(input_buffer.data(), input_size);
    input.close();
    
    // Compress
    size_t const output_buffer_size = ZSTD_compressBound(input_size);
    std::vector<char> output_buffer(output_buffer_size);
    
    size_t const compressed_size = ZSTD_compress(
        output_buffer.data(), output_buffer_size,
        input_buffer.data(), input_size,
        compression_level
    );
    
    if (ZSTD_isError(compressed_size)) {
        std::cerr << "Compression error: " << ZSTD_getErrorName(compressed_size) << std::endl;
        return false;
    }
    
    // Write compressed data
    std::ofstream output(output_path, std::ios::binary);
    if (!output) {
        std::cerr << "Failed to open output file: " << output_path << std::endl;
        return false;
    }
    
    output.write(output_buffer.data(), compressed_size);
    output.close();
    
    // Delete original if requested
    if (delete_original) {
        if (std::remove(input_path.c_str()) != 0) {
            std::cerr << "Warning: Failed to delete original file: " << input_path << std::endl;
        }
    }
    
    double ratio = 100.0 * (1.0 - (double)compressed_size / input_size);
    std::cout << "Compressed " << input_path << " to " << output_path << ": " 
              << input_size / (1024.0 * 1024.0) << " MB -> "
              << compressed_size / (1024.0 * 1024.0) << " MB "
              << "(" << ratio << "% saved)" << std::endl;
    
    return true;
}

bool is_feasible_solution(const Instance &ins, const Solution &solution,
                          const int verbose)
{
  if (solution.empty()) return true;

  // check start locations
  if (!is_same_config(solution.front(), ins.starts)) {
    info(1, verbose, "invalid starts");
    return false;
  }

  // check goal locations
  if (!is_same_config(solution.back(), ins.goals)) {
    info(1, verbose, "invalid goals");
    return false;
  }

  for (size_t t = 1; t < solution.size(); ++t) {
    for (size_t i = 0; i < ins.N; ++i) {
      auto v_i_from = solution[t - 1][i];
      auto v_i_to = solution[t][i];
      // check connectivity
      if (v_i_from != v_i_to &&
          std::find(v_i_to->neighbor.begin(), v_i_to->neighbor.end(),
                    v_i_from) == v_i_to->neighbor.end()) {
        info(1, verbose, "invalid move");
        return false;
      }

      // check conflicts
      for (size_t j = i + 1; j < ins.N; ++j) {
        auto v_j_from = solution[t - 1][j];
        auto v_j_to = solution[t][j];
        // vertex conflicts
        if (v_j_to == v_i_to) {
          info(1, verbose, "vertex conflict between agent-", i, " and agent-",
               j, " at vertex-", v_i_to->id, " at timestep ", t);
          return false;
        }
        // swap conflicts
        if (v_j_to == v_i_from && v_j_from == v_i_to) {
          info(1, verbose, "edge conflict");
          return false;
        }
      }
    }
  }

  return true;
}

void print_stats(const int verbose, const Deadline *deadline,
                 const Instance &ins, const Solution &solution,
                 const double comp_time_ms)
{
  auto ceil = [](float x) { return std::ceil(x * 100) / 100; };
  auto dist_table = DistTable(ins);
  const auto makespan = get_makespan(solution);
  const auto makespan_lb = get_makespan_lower_bound(ins, dist_table);
  const auto sum_of_costs = get_sum_of_costs(solution);
  const auto sum_of_costs_lb = get_sum_of_costs_lower_bound(ins, dist_table);
  const auto sum_of_loss = get_sum_of_loss(solution);
  info(1, verbose, deadline, "solved", "\tmakespan: ", makespan,
       " (lb=", makespan_lb, ", ub=", ceil((float)makespan / makespan_lb), ")",
       "\tsum_of_costs: ", sum_of_costs, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_costs / sum_of_costs_lb), ")",
       "\tsum_of_loss: ", sum_of_loss, " (lb=", sum_of_costs_lb,
       ", ub=", ceil((float)sum_of_loss / sum_of_costs_lb), ")");
}

// for log of map_name
static const std::regex r_map_name = std::regex(R"(.+/(.+))");

void make_log(const Instance &ins, const bool flg_no_all, const Solution &solution,
              const std::string &output_name, const double comp_time_ms,
              const std::string &map_name, const int seed, const bool log_short)
{
  // map name
  std::smatch results;
  const auto map_recorded_name =
      (std::regex_match(map_name, results, r_map_name)) ? results[1].str()
                                                        : map_name;

  // for instance-specific values
  auto dist_table = DistTable(ins);

  // log for visualizer
  auto get_x = [&](int k) { return k % ins.G->width; };
  auto get_y = [&](int k) { return k / ins.G->width; };
  std::ofstream log;

  int lacam_version;
  if (flg_no_all) {
    lacam_version = 1;
  } else {
    if (Planner::PIBT_NUM == 0 && !Planner::FLG_REFINER && !Planner::FLG_SCATTER && Planner::RANDOM_INSERT_PROB1 == 0 && Planner::RANDOM_INSERT_PROB2 == 0 && Planner::RECURSIVE_RATE == 0) {
      lacam_version = 2;
    } else {
      lacam_version = 3;
    }
  }

  log.open(output_name, std::ios::out);
  log << "LACAM_LOG_VERSION=" << LACAM_LOG_VERSION << "\n";
  log << "\n====== MAPF INSTANCE INFO ======\n";
  log << "map_file=" << map_name << "\n";
  log << "map_name =" << map_recorded_name << "\n";
  log << "agents =" << ins.N << "\n";
  log << "mapf_mode = ONESHOT\n";
  log << "seed =" << seed << "\n";
  log << "\n====== LACAM METHOD INFO ======\n";
  log << "LACAM VERSION =" << lacam_version << "\n";
  log << "FLG_NO_ALL =" << flg_no_all << "\n";
  log << "FLG_STAR =" << Planner::FLG_STAR << "\n";
  log << "FLG_SWAP =" << Planner::FLG_SWAP << "\n";
  log << "PIBT_NUM =" << Planner::PIBT_NUM << "\n";
  log << "FLG_REFINER =" << Planner::FLG_REFINER << "\n";
  log << "REFINER_NUM =" << Planner::REFINER_NUM << "\n";
  log << "FLG_SCATTER =" << Planner::FLG_SCATTER << "\n";
  log << "SCATTER_MARGIN =" << Planner::SCATTER_MARGIN << "\n";
  log << "RANDOM_INSERT_PROB1 =" << Planner::RANDOM_INSERT_PROB1 << "\n";
  log << "RANDOM_INSERT_PROB2 =" << Planner::RANDOM_INSERT_PROB2 << "\n";
  log << "FLG_RANDOM_INSERT_INIT_NODE =" << Planner::FLG_RANDOM_INSERT_INIT_NODE << "\n";
  log << "RECURSIVE_RATE =" << Planner::RECURSIVE_RATE << "\n";
  log << "RECURSIVE_TIME_LIMIT =" << Planner::RECURSIVE_TIME_LIMIT << "\n";
  log << "CHECKPOINTS_DURATION =" << Planner::CHECKPOINTS_DURATION << "\n";
  log << "\n====== RUN CONFIG INFO ======\n";
  log << "FLG_MULTI_THREAD =" << Planner::FLG_MULTI_THREAD << "\n";
  log << "\n====== LACAM SOLUTION INFO ======\n";
  log << "SOLVER = LACAM" << lacam_version << "\n";
  log << "solved=" << !solution.empty() << "\n";
  log << "soc=" << get_sum_of_costs(solution) << "\n";
  log << "soc_lb=" << get_sum_of_costs_lower_bound(ins, dist_table) << "\n";
  log << "makespan=" << get_makespan(solution) << "\n";
  log << "makespan_lb=" << get_makespan_lower_bound(ins, dist_table) << "\n";
  log << "sum_of_loss=" << get_sum_of_loss(solution) << "\n";
  log << "sum_of_loss_lb=" << get_sum_of_costs_lower_bound(ins, dist_table) << "\n";
  log << "comp_time=" << comp_time_ms << "\n";
  
  log << Planner::MSG << "\n";
  if (log_short) return;
  log << "starts=";
  for (size_t i = 0; i < ins.N; ++i) {
    log << ins.starts[i]->index;
    if (i != ins.N - 1) log << ",";
  }
  log << "\ngoals=";
  for (size_t i = 0; i < ins.N; ++i) {
    log << ins.goals[i]->index;
    if (i != ins.N - 1) log << ",";
  }
  log << "\nsolution=\n";
    for (size_t i=0; i < ins.N; ++i) {
      log << "  Agent " << i << ": [";
  for (size_t t = 0; t < solution.size(); ++t) {
        log << solution[t][i]->index;
        if (t != solution.size() - 1) log << ",";
    }
    log << "]\n";
  }
    // for (size_t t = 0; t < solution.size(); ++t) {
    //   log << t << ": [";
    //   auto C = solution[t];
    //   for (auto v : C) {
    //     // log << "(" << get_x(v->index) << "," << get_y(v->index) << "),";
    //     log << v->index;
    //     if (v != C.back()) log << ", ";
    //   }
    //   log << "]\n";
    // }

  log.close();

  std::cout << "\n\nLACAM LOG AT " << output_name << ":\n" << std::endl;
  std::ifstream log_readback(output_name);
  std::cout << log_readback.rdbuf();

}
