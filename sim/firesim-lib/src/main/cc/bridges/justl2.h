// See LICENSE for license details

#ifndef __JUSTL2_H
#define __JUSTL2_H

#include "core/bridge_driver.h"
#include "bridges/serial_data_justl2.h"


#include <cstdint>
#include <memory>
#include <optional>
#include <signal.h>
#include <string>
#include <vector>

/**
 * Structure carrying the addresses of all fixed MMIO ports.
 *
 * This structure is instantiated when all bridges are populated based on
 * the target configuration.
 */
struct JUSTL2BRIDGEMODULE_struct {
  // uint32_t l2_misses;
  // uint32_t l2_accesses;
  uint64_t l2_misses_low;    // Lower 32 bits of L2 misses
  uint64_t l2_misses_high;   // Upper 32 bits of L2 misses
  uint64_t l2_accesses_low;  // Lower 32 bits of L2 accesses
  uint64_t l2_accesses_high; // Upper 32 bits of L2 accesses
  uint64_t l2_misses_load;    
  uint64_t l2_misses_writeback;   
  uint64_t l2_accesses_load;  
  uint64_t l2_accesses_writeback; 
  uint64_t optgen_accesses;  
  uint64_t optgen_hit; 
  uint64_t out_bits;
  uint64_t out_valid;
  uint64_t out_ready;
  uint64_t in_bits;
  uint64_t in_valid;
  uint64_t in_ready;
};

/**
 * Base class for callbacks handling data coming in and out a UART stream.
 */
class justl2_handler {
public:
  virtual ~justl2_handler() = default;

  virtual std::optional<char> get() = 0;
  virtual void put(char data) = 0;
};

class justl2_t final : public bridge_driver_t {
public:
  /// The identifier for the bridge type used for casts.
  static char KIND;

  /// Creates a bridge which interacts with standard streams or PTY.
  justl2_t(simif_t &simif,
         const JUSTL2BRIDGEMODULE_struct &mmio_addrs,
         int justl2no,
         const std::vector<std::string> &args);

  ~justl2_t() override;

  void tick() override;

private:
  const JUSTL2BRIDGEMODULE_struct mmio_addrs;
  std::unique_ptr<justl2_handler> handler;

  serial_data_t<char> data;

  void send();
  void recv();
  uint64_t read_register(uint32_t addr);

public:
  // uint32_t read_l2_accesses();
  // uint32_t read_l2_misses();  
  uint64_t read_l2_accesses_low();
  uint64_t read_l2_accesses_high();
  uint64_t read_l2_misses_low();
  uint64_t read_l2_misses_high();
  uint64_t read_l2_misses_load();    
  uint64_t read_l2_misses_writeback();   
  uint64_t read_l2_accesses_load();  
  uint64_t read_l2_accesses_writeback(); 
  uint64_t read_optgen_accesses();  
  uint64_t read_optgen_hit(); 
};

#endif // __UART_H
