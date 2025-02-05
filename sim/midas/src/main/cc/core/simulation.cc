// See LICENSE for license details.

#include "simulation.h"
#include "bridges/clock.h"
#include "bridges/justl2.h"
#include "bridges/loadmem.h"
#include "bridges/master.h"
#include "core/bridge_driver.h"
#include "core/simif.h"
#include "core/stream_engine.h"
#include "core/timing.h"

#include <cassert>
#include <cinttypes>
#include <cstdio>

simulation_t::simulation_t(widget_registry_t &registry,
                           const std::vector<std::string> &args)
    : registry(registry), clock(registry.get_widget<clockmodule_t>()) {
  bool fastloadmem = false;
  for (auto &arg : args) {
    if (arg.find("+fastloadmem") == 0) {
      fastloadmem = true;
    }
    if (arg.find("+loadmem=") == 0) {
      load_mem_path = arg.c_str() + 9;
    }
    if (arg.find("+zero-out-dram") == 0) {
      do_zero_out_dram = true;
    }
    if (arg.find("+check-fingerprint") == 0) {
      check_fingerprint_only = true;
    }
    if (arg.find("+write-fingerprint=") == 0) {
      write_fingerprint_only = atoi(arg.c_str() + 19);
    }
  }

  if (fastloadmem)
    load_mem_path.clear();
}

void simulation_t::record_start_times() {
  start_hcycle = clock.hcycle();
  start_time = timestamp();
}

void simulation_t::record_end_times() {
  end_time = timestamp();
  end_tcycle = clock.tcycle();
  end_hcycle = clock.hcycle();
}

void simulation_t::print_simulation_performance_summary() {
  // Must call record_start_times and record_end_times before invoking this
  // function
  assert(start_hcycle.has_value() && end_hcycle.has_value() &&
         "simulation not executed");

  const uint64_t hcycles = *end_hcycle - *start_hcycle;
  const double sim_time = diff_secs(end_time, start_time);
  const double sim_speed = ((double)end_tcycle) / (sim_time * 1000.0);
  const double measured_host_frequency =
      ((double)hcycles) / (sim_time * 1000.0);
  const double fmr = ((double)hcycles / end_tcycle);

  fprintf(stderr, "\nEmulation Performance Summary\n");
  fprintf(stderr, "------------------------------\n");
  fprintf(stderr, "Wallclock Time Elapsed: %.1f s\n", sim_time);
  // Provide enough sig-figs to let the report be useful in RTL sim
  fprintf(stderr, "Host Frequency: ");
  if (measured_host_frequency > 1000.0) {
    fprintf(stderr, "%.3f MHz\n", measured_host_frequency / 1000.0);
  } else {
    fprintf(stderr, "%.3f KHz\n", measured_host_frequency);
  }

  fprintf(stderr, "Target Cycles Emulated: %" PRIu64 "\n", end_tcycle);
  fprintf(stderr, "Effective Target Frequency: ");
  if (sim_speed > 1000.0) {
    fprintf(stderr, "%.3f MHz\n", sim_speed / 1000.0);
  } else {
    fprintf(stderr, "%.3f KHz\n", sim_speed);
  }
  fprintf(stderr, "FMR: %.2f\n", fmr);
  fprintf(stderr,
          "Note: The latter three figures are based on the fastest "
          "target clock.\n");      
  
  // DN: Use an optional or pointer to check if it exists
  if (auto *justl2_ptr = registry.get_widget_opt<justl2_t>()) {  // Check if the pointer is not null
      //auto &justl2 = *justl2_ptr;  // Dereference the pointer to use the object
      // uint32_t l2_accesses = justl2_ptr->read_l2_accesses();
      // uint32_t l2_misses = justl2_ptr->read_l2_misses();

     //SM: Read L2 accesses (low and high) and combine into a 64-bit unsigned value
      // uint64_t l2_accesses = ((uint64_t)l2_accesses_high << 32) | l2_accesses_low;
      uint64_t l2_accesses_low = justl2_ptr->read_l2_accesses_low();
      uint64_t l2_accesses_high = justl2_ptr->read_l2_accesses_high();
      uint64_t l2_accesses = (l2_accesses_high << 32) | (l2_accesses_low & 0xFFFFFFFF);
      uint64_t l2_accesses_load = justl2_ptr->read_l2_accesses_load();
      uint64_t l2_accesses_writeback = justl2_ptr->read_l2_accesses_writeback();
      uint64_t optgen_accesses= justl2_ptr->read_optgen_accesses();

      // uint64_t l2_misses = ((uint64_t)l2_misses_high << 32) | l2_misses_low;
      uint64_t l2_misses_low = justl2_ptr->read_l2_misses_low();
      uint64_t l2_misses_high = justl2_ptr->read_l2_misses_high();
      uint64_t l2_misses = (l2_misses_high << 32) | (l2_misses_low & 0xFFFFFFFF);
      uint64_t l2_misses_load = justl2_ptr->read_l2_misses_load();
      uint64_t l2_misses_writeback = justl2_ptr->read_l2_misses_writeback();
      uint64_t optgen_hit = justl2_ptr->read_optgen_hit();
      




      //SM
      // Check for division by zero
      if (l2_accesses != 0) {
          fprintf(stderr,
                  "L2 accesses: %" PRIu64 ", L2 misses: %" PRIu64 ", Miss rate: %f \n",
                  l2_accesses, l2_misses, (double)l2_misses / l2_accesses);
      } else {
          fprintf(stderr, "L2 accesses: %" PRIu64 ", L2 misses: %" PRIu64 ", L2 Miss rate: undefined (division by zero)\n",
                  l2_accesses, l2_misses);
      }

      // Check for division by zero
      if (l2_accesses_load != 0) {
          fprintf(stderr,
                  "l2_accesses_load: %" PRIu64 ", l2_misses_load: %" PRIu64 ", Load Miss rate: %f \n",
                  l2_accesses_load, l2_misses_load, (double)l2_misses_load / l2_accesses_load);
      } else {
          fprintf(stderr, "l2_accesses_load: %" PRIu64 ", l2_misses_load: %" PRIu64 ", L2 load Miss rate: undefined (division by zero)\n",
                  l2_accesses_load, l2_misses_load);
      }

      if (l2_accesses_writeback != 0) {
          fprintf(stderr,
                  "l2_accesses_writeback: %" PRIu64 ", l2_misses_writeback: %" PRIu64 ", writeback Miss rate: %f \n",
                  l2_accesses_writeback, l2_misses_writeback, (double)l2_misses_writeback / l2_accesses_writeback);
      } else {
          fprintf(stderr, "l2_accesses_writeback: %" PRIu64 ", l2_misses_writeback: %" PRIu64 ", L2 writeback Miss rate: undefined (division by zero)\n",
                  l2_accesses_writeback, l2_misses_writeback);
      }

      if (optgen_accesses != 0) {
          fprintf(stderr,
                  "optgen_accesses: %" PRIu64 ", optgen_hit: %" PRIu64 ", optgen hit rate: %f \n",
                  optgen_accesses, optgen_hit, (double)optgen_hit / optgen_accesses);

      } else {
          fprintf(stderr, "optgen_accesses: %" PRIu64 ", optgen_hit: %" PRIu64 ", optgen hit rate: undefined (division by zero)\n",
                  optgen_accesses, optgen_hit);
      }

        }
}

void simulation_t::simulation_init() {
  for (auto *bridge : registry.get_all_bridges()) {
    bridge->init();
  }
}

void simulation_t::simulation_finish() {
  for (auto *bridge : registry.get_all_bridges()) {
    bridge->finish();
  }
}

int simulation_t::execute_simulation_flow() {
  wait_for_init();

  // following fingerprint logic uses 'exit' instead of 'return' to avoid
  // issues w/ deconstructors not having initialized values
  auto &master = registry.get_widget<master_t>();
  if (check_fingerprint_only || write_fingerprint_only.has_value()) {
    if (check_fingerprint_only && write_fingerprint_only.has_value()) {
      fprintf(stderr, "Unable to both check/write FireSim fingerprint\n");
      exit(EXIT_FAILURE);
    }

    if (check_fingerprint_only && master.check_fingerprint()) {
      fprintf(stderr, "Invalid FireSim fingerprint\n");
      exit(EXIT_FAILURE);
    }
    if (write_fingerprint_only.has_value()) {
      master.write_fingerprint(write_fingerprint_only.value());
    }
    exit(EXIT_SUCCESS);
  } else {
    if (master.check_fingerprint()) {
      fprintf(stderr, "Invalid FireSim fingerprint\n");
      exit(EXIT_FAILURE);
    }
  }

  if (auto *stream = registry.get_stream_engine()) {
    stream->init();
  }

  init_dram();

  simulation_init();

  record_start_times();
  fprintf(stderr, "Commencing simulation.\n");
  const int exit_code = simulation_run();
  fprintf(stderr, "\nSimulation complete.\n");
  record_end_times();

  simulation_finish();

  const bool timeout = simulation_timed_out();

  if (exit_code != 0) {
    fprintf(stderr,
            "*** FAILED *** (code = %d) after %" PRIu64 " cycles\n",
            exit_code,
            end_tcycle);
  } else if (timeout) {
    fprintf(stderr,
            "*** FAILED *** simulation timed out after %" PRIu64 " cycles\n",
            end_tcycle);
  } else {
    fprintf(stderr, "*** PASSED *** after %" PRIu64 " cycles\n", end_tcycle);
  }

  print_simulation_performance_summary();

  return timeout ? EXIT_FAILURE : exit_code;
}

void simulation_t::wait_for_init() {
  auto &master = registry.get_widget<master_t>();
  while (!master.is_init_done())
    ;
}

void simulation_t::init_dram() {
  if (auto *loadmem = registry.get_widget_opt<loadmem_t>()) {
    if (do_zero_out_dram) {
      fprintf(stderr,
              "Zeroing out FPGA DRAM. This will take a few seconds...\n");
      loadmem->zero_out_dram();
    }

    if (!load_mem_path.empty()) {
      loadmem->load_mem_from_file(load_mem_path);
    }
  } else {
    if (do_zero_out_dram || !load_mem_path.empty()) {
      fprintf(stderr,
              "Skipping memory initialization: target does not use DRAM\n");
    }
  }
}
