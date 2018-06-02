// See LICENSE for license details.

#include "sim.h"
#include "mmu.h"
#include "dts.h"
#include "remote_bitbang.h"
#include <map>
#include <iostream>
#include <sstream>
#include <climits>
#include <cstdlib>
#include <cassert>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>

volatile bool ctrlc_pressed = false;
static void handle_signal(int sig)
{
  if (ctrlc_pressed)
    exit(-1);
  ctrlc_pressed = true;
  signal(sig, &handle_signal);
}

sim_t::sim_t(const char* isa, size_t nprocs, bool halted, reg_t start_pc,
             std::vector<std::pair<reg_t, mem_t*>> mems,
             const std::vector<std::string>& args,
             std::vector<int> const hartids, unsigned progsize,
             unsigned max_bus_master_bits, bool require_authentication)
  : htif_t(args), mems(mems), procs(std::max(nprocs, size_t(1))),
    start_pc(start_pc), current_step(0), current_proc(0), debug(false),
    remote_bitbang(NULL),
    debug_module(this, progsize, max_bus_master_bits, require_authentication)
{
  signal(SIGINT, &handle_signal);

  for (auto& x : mems)
    bus.add_device(x.first, x.second);

  debug_module.add_device(&bus);

  debug_mmu = new mmu_t(this, NULL);

  if (hartids.size() == 0) {
    for (size_t i = 0; i < procs.size(); i++) {
      procs[i] = new processor_t(isa, this, i, halted);
    }
  }
  else {
    if (hartids.size() != procs.size()) {
      std::cerr << "Number of specified hartids doesn't match number of processors" << strerror(errno) << std::endl;
      exit(1);
    }
    for (size_t i = 0; i < procs.size(); i++) {
      procs[i] = new processor_t(isa, this, hartids[i], halted);
    }
  }

  clint.reset(new clint_t(procs));
  bus.add_device(CLINT_BASE, clint.get());
}

sim_t::~sim_t()
{
  for (size_t i = 0; i < procs.size(); i++)
    delete procs[i];
  delete debug_mmu;
}

void sim_thread_main(void* arg)
{
  ((sim_t*)arg)->main();
}

reg_t sim_t::my_get_mem(uint32_t addr)
{
  reg_t val;
  // if(args.size() != 1 && args.size() != 2)
  //   throw trap_interactive();

  // std::string addr_str = args[0];
  mmu_t* mmu = debug_mmu;
  // if(args.size() == 2)
  // {
  //   processor_t *p = get_core(args[0]);
  //   mmu = p->get_mmu();
  //   addr_str = args[1];
  // }

  // reg_t addr = strtol(addr_str.c_str(),NULL,16), val;
  // if(addr == LONG_MAX)
  //   addr = strtoul(addr_str.c_str(),NULL,16);

  switch(addr % 8)
  {
    case 0:
      val = mmu->load_uint64(addr);
      break;
    case 4:
      val = mmu->load_uint32(addr);
      break;
    case 2:
    case 6:
      val = mmu->load_uint16(addr);
      break;
    default:
      val = mmu->load_uint8(addr);
      break;
  }
  return val;
}

#define USE_OPENCV 1
#if (USE_OPENCV)
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#define COLS    256
#define ROWS    144
#define DIM     1

cv::Mat preview_image = cv::Mat::zeros(cv::Size(COLS, ROWS), CV_8UC1);;
bool first_time = true;
void sim_t::show_monitor(uint32_t base)
{
  //static  = cv::Mat::zeros(cv::Size(COLS, ROWS), CV_8UC1);
  // if(!first_time){
  //     preview_image.release();
  //    preview_image.create(ROWS, COLS,CV_8UC1);
  //     first_time = false;
  // }

  mmu_t* mmu = debug_mmu;
  reg_t val;
  uint32_t pt_app_status = base;
  uint32_t pt_spike_status = base+1;
  uint32_t pt_n_rows = base+2;
  uint32_t pt_n_cols = base+4;
  uint32_t pt_n_channels = base+6;
  uint32_t pt_data = base+7;

  uint8_t app_status = mmu->load_uint8(pt_app_status);
  uint8_t spike_status = mmu->load_uint8(pt_spike_status);
  uint32_t n_rows = mmu->load_uint16(pt_n_rows);
  uint32_t n_cols = mmu->load_uint16(pt_n_cols);
  uint32_t n_channels = mmu->load_uint8(pt_n_channels);
  uint32_t data = mmu->load_uint8(pt_data);


  printf("app_status   = %d \n", app_status);
  printf("spike_status = %d \n", spike_status);
  printf("n_rows       = %d \n", n_rows);
  printf("n_cols       = %d \n", n_cols);
  printf("n_channels   = %d \n", n_channels);
  printf("data         = 0x%lx \n", data);

  // // Create image and push values

  // // push values
  for(int i = 0; i < n_rows; i++){
    for (int j = 0; j < n_cols; j++) {
  //     //preview_image.at<uchar>(i , j) = mmu->load_uint8(pt_data + i*n_rows + j); //memory[i][j][0];
       preview_image.at<uchar>(i, j) = mmu->load_uint8(pt_data + (i*n_cols + j));
      printf("Element [%d][%d] = 0x%2X \n", i, j, mmu->load_uint8(pt_data + (i*n_cols + j))); //memory[i][j][0];
    }
  }
  // // Show image
  cv::imshow("Spike Monitor", preview_image);
  while(cv::waitKey(1) != 'q')
      ;
    cv::destroyWindow("Spike Monitor");

  // //cv::destroyAllWindows();

}

#endif

void sim_t::main()
{
  if (!debug && log)
    set_procs_debug(true);

  while (!done())
  {
    if (debug || ctrlc_pressed)
      interactive();
    else
      step(INTERLEAVE);
    if (remote_bitbang) {
      remote_bitbang->tick();
    }
    if (monitor){
      #if (USE_OPENCV)
        show_monitor(monitor_base);
      #endif
      // printf("-------------------->>>  At 0x%x = 0x%lx \n", monitor_base, my_get_mem(monitor_base));
    }
  }
}


int sim_t::run()
{
  host = context_t::current();
  target.init(sim_thread_main, this);
  return htif_t::run();
}

void sim_t::step(size_t n)
{
  for (size_t i = 0, steps = 0; i < n; i += steps)
  {
    steps = std::min(n - i, INTERLEAVE - current_step);
    procs[current_proc]->step(steps);

    current_step += steps;
    if (current_step == INTERLEAVE)
    {
      current_step = 0;
      procs[current_proc]->yield_load_reservation();
      if (++current_proc == procs.size()) {
        current_proc = 0;
        clint->increment(INTERLEAVE / INSNS_PER_RTC_TICK);
      }

      host->switch_to();
    }
  }
}

void sim_t::set_debug(bool value)
{
  debug = value;
}

void sim_t::set_log(bool value)
{
  log = value;
}

void sim_t::set_monitor(bool value, uint32_t base_address)
{
  monitor = value;
  monitor_base = base_address;
}

void sim_t::set_histogram(bool value)
{
  histogram_enabled = value;
  for (size_t i = 0; i < procs.size(); i++) {
    procs[i]->set_histogram(histogram_enabled);
  }
}

void sim_t::set_procs_debug(bool value)
{
  for (size_t i=0; i< procs.size(); i++)
    procs[i]->set_debug(value);
}

bool sim_t::mmio_load(reg_t addr, size_t len, uint8_t* bytes)
{
  if (addr + len < addr)
    return false;
  return bus.load(addr, len, bytes);
}

bool sim_t::mmio_store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if (addr + len < addr)
    return false;
  return bus.store(addr, len, bytes);
}

void sim_t::make_dtb()
{
  const int reset_vec_size = 8;

  start_pc = start_pc == reg_t(-1) ? get_entry_point() : start_pc;

  uint32_t reset_vec[reset_vec_size] = {
    0x297,                                      // auipc  t0,0x0
    0x28593 + (reset_vec_size * 4 << 20),       // addi   a1, t0, &dtb
    0xf1402573,                                 // csrr   a0, mhartid
    get_core(0)->get_xlen() == 32 ?
      0x0182a283u :                             // lw     t0,24(t0)
      0x0182b283u,                              // ld     t0,24(t0)
    0x28067,                                    // jr     t0
    0,
    (uint32_t) (start_pc & 0xffffffff),
    (uint32_t) (start_pc >> 32)
  };

  std::vector<char> rom((char*)reset_vec, (char*)reset_vec + sizeof(reset_vec));

  dts = make_dts(INSNS_PER_RTC_TICK, CPU_HZ, procs, mems);
  std::string dtb = dts_compile(dts);

  rom.insert(rom.end(), dtb.begin(), dtb.end());
  const int align = 0x1000;
  rom.resize((rom.size() + align - 1) / align * align);

  boot_rom.reset(new rom_device_t(rom));
  bus.add_device(DEFAULT_RSTVEC, boot_rom.get());
}

char* sim_t::addr_to_mem(reg_t addr) {
  auto desc = bus.find_device(addr);
  if (auto mem = dynamic_cast<mem_t*>(desc.second))
    if (addr - desc.first < mem->size())
      return mem->contents() + (addr - desc.first);
  return NULL;
}

// htif

void sim_t::reset()
{
  make_dtb();
}

void sim_t::idle()
{
  target.switch_to();
}

void sim_t::read_chunk(addr_t taddr, size_t len, void* dst)
{
  assert(len == 8);
  auto data = debug_mmu->load_uint64(taddr);
  memcpy(dst, &data, sizeof data);
}

void sim_t::write_chunk(addr_t taddr, size_t len, const void* src)
{
  assert(len == 8);
  uint64_t data;
  memcpy(&data, src, sizeof data);
  debug_mmu->store_uint64(taddr, data);
}

void sim_t::proc_reset(unsigned id)
{
  debug_module.proc_reset(id);
}
