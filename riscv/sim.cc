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

// #define USE_OPENCV 1
// #if (USE_OPENCV)
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "cvplot.h"

#define COLS    256
#define ROWS    144
#define DIM     1
#define IMSHOW_TIMEOUT  10
#define OSCILLO_TIMEOUT  10

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
  mmu_t* mmu = debug_mmu;

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

void sim_t::show_monitor(uint32_t base, bool verbose)
{
  cv::Mat new_preview_image(cv::Size(COLS, ROWS), CV_8UC1, cv::Scalar(0,0,0));
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

  if(verbose)
  {
    printf("[INFO] ------------------------------------\n"
           "[INFO] >> At 0x%x = 0x%lx \n", base, my_get_mem(base));
    printf("[INFO] app_status   = %d \n", app_status);
    printf("[INFO] spike_status = %d \n", spike_status);
    printf("[INFO] n_rows       = %d \n", n_rows);
    printf("[INFO] n_cols       = %d \n", n_cols);
    printf("[INFO] n_channels   = %d \n", n_channels);
    printf("[INFO] data         = 0x%ux \n", data);
    printf("[INFO] ------------------------------------\n");
  }
  // Create image and push values
  for(unsigned int i = 0; i < n_rows; i++){
    for (unsigned int j = 0; j < n_cols; j++) {
      uint8_t x = 0xFF & mmu->load_uint8(pt_data + (i*n_cols + j));
      new_preview_image.at<uchar>(i, j) = x;
    }
  }
  cv::imshow("monitor", new_preview_image);
  cv::waitKey(IMSHOW_TIMEOUT);
}

void sim_t::show_oscillo(uint32_t base, bool verbose)
{
  // Byte length of Input_signals
  #define INPUT_BYTES       1

  // Modify this to handle the number of bytes of the output signal
  // As saved in oscillo.s output signal section
  #define OUTPUT_BYTES      2
  
  // If you would like to shift the Output Signal left before plotting then put this here
  // Useful for high values signals
  // Or for comparison with other signals.
  #define OUTPUT_VIS_SHIFT  0

  // Window WIDTH and HEIGHT
  #define HORIZONTAL    1100
  #define VERTICAL      500

  // If you want to keep the X axis (temps) moving (the signal is represented with real-time in X axis)
  // then turn this to 1 
  #define MOVING_T      0

  // When app-status == 0x00 this means you shut-down the signal plotter
  // If you would like that the signal plotter's windows stays open and preserve the last samples
  // then turn this to 1
  #define PRESERVE_LAST_SAMPLES 0

  std::vector<std::pair<float, float>> in_signal_data, out_signal_data;
  static uint32_t in_signal_t, out_signal_t;
  mmu_t* mmu = debug_mmu;

  uint32_t pt_app_status = base;
  uint32_t pt_spike_status = base+1;
  uint32_t pt_n_rows = base+2;
  uint32_t pt_data = base+4;

  uint8_t app_status = mmu->load_uint8(pt_app_status);
  uint8_t spike_status = mmu->load_uint8(pt_spike_status);

  if(!(app_status & 0x03)) // if no input && no output
  { 
    in_signal_data.clear();
    out_signal_data.clear();
    
    #if (!PRESERVE_LAST_SAMPLES)
      cv::destroyAllWindows();
    #endif
    
    return ;
  }else{

    uint16_t in_n_rows = mmu->load_uint16(pt_n_rows);
    uint8_t in_data = mmu->load_uint8(pt_data);

    if(verbose){
      printf("[INFO] ------------------------------------\n"
             "[INFO] >> At 0x%x = 0x%lx \n", base, my_get_mem(base));
      printf("[INFO] app_status   = 0x%x \n", app_status);
      printf("[INFO] spike_status = 0x%x \n", spike_status);
    }

    if(app_status & 0x01) // Input valid
    {
      if(verbose){
        printf("[INFO / INPUT] ------------------------------------\n");
        printf("[INFO / INPUT] in_n_rows       = %d \n", in_n_rows);
        printf("[INFO / INPUT] in_data[0]      = 0x%x \n", in_data);
      }

      uint32_t actual_val = 0;
      #if (!MOVING_T)
      in_signal_t = 0;
      #endif

      for(unsigned int i = 0; i < in_n_rows*INPUT_BYTES; i += INPUT_BYTES){
        // if (signal_t >= 300 ){
        //   signal_data.erase(signal_data.begin());
        // }
        for(int j = 0; j<INPUT_BYTES; j++){
          actual_val |= (((uint32_t)(mmu->load_uint8(pt_data + i+ j))) << (j*8));
        }

        in_signal_data.push_back({(float)in_signal_t, (float)(actual_val)}); 
        //in_signal_data.push_back({(float)in_signal_t, (float)(0xFF & mmu->load_uint8(pt_data + i))});
        /* TO DO: (maybe ?) shift signal to permit writing directly in the first element  */
        in_signal_t++;
        actual_val = 0;
      }
    }else{ // Clear Input
      in_signal_data.clear();
    }

    if(app_status & 0x02) // Output valid
    {
      //uint32_t out_n_rows = mmu->load_uint16(pt_data + in_n_rows);
      uint16_t out_n_rows = (0xFF & mmu->load_uint8(pt_data + in_n_rows)) |
                            (0xFF & mmu->load_uint8(pt_data + in_n_rows + 1)) << 8;
      uint8_t out_data = mmu->load_uint8(pt_data + in_n_rows + 2);
      uint32_t actual_val = 0;
      #if (!MOVING_T)
      out_signal_t = 0;
      #endif
      if(verbose){  
        printf("[INFO / OUTPUT] ------------------------------------\n");
        printf("[INFO / OUTPUT] out_n_rows       = %d \n", out_n_rows);
        printf("[INFO / OUTPUT] out_data[0]      = 0x%x \n", out_data);
      }

      for(unsigned int i = 0; i < out_n_rows*OUTPUT_BYTES; i+=OUTPUT_BYTES){
        // if (out_signal_t >= 3000 ){
        //    out_signal_data.erase(out_signal_data.begin());
        // }
        for(int j = 0; j<OUTPUT_BYTES; j++){
          actual_val |= (((uint32_t)(mmu->load_uint8(pt_data + in_n_rows + 2 + i + j))) << (j*8));
        }
        // actual_val = (0xFF & mmu->load_uint8(pt_data + in_n_rows + 2 + i)) |
        //              (0xFF & mmu->load_uint8(pt_data + in_n_rows + 2 + i+1)) << 8 |
        //              (0xFF & mmu->load_uint8(pt_data + in_n_rows + 2 + i+2)) << 16 |
        //              (0xFF & mmu->load_uint8(pt_data + in_n_rows + 2 + i+3)) << 24 ;

        // printf("output_signal[ %d ] = 0x%x \n", i, actual_val);
        out_signal_data.push_back({(float)out_signal_t, (float)(actual_val >> OUTPUT_VIS_SHIFT)});
        out_signal_t++;
        actual_val = 0;
        /* TO DO: (maybe ?) shift signal to permit writing directly in the first element  */
      }
    }else{ // Clear Input
      out_signal_data.clear();
    }

    // // Populate signal_data && refresh the oscilloscope
    // for(unsigned int i = 0; i < n_rows; i++){
    //   // if (signal_t >= 300 ){
    //   //   signal_data.erase(signal_data.begin());
    //   // }
    //   signal_data.push_back({(float)signal_t, (float)(0xFF & mmu->load_uint8(pt_data + i))});
    //   signal_t++;
    //   /* TO DO: (maybe ?) shift signal to permit writing directly in the first element  */
    // }

    auto oscillo_name = "input-spike-oscillo";
    cvplot::setWindowTitle(oscillo_name, "Spike Signal Plotter");
    cvplot::resizeWindow(oscillo_name, HORIZONTAL, VERTICAL);
    auto &view = cvplot::Window::current().view(oscillo_name);
    auto &figure = cvplot::figure(oscillo_name);


    figure.square(true);
    figure.origin(false, false);
    figure.series("Input Signal")
            .set(in_signal_data)
            .type(cvplot::DotLine)
            .color(cvplot::Blue);

    figure.series("Output Signal")
        .set(out_signal_data)
        .type(cvplot::DotLine)
        .color(cvplot::Red);

    figure.show(false);
    view.finish();
    view.flush();

  }


}

void sim_t::interactive_show_monitor(const std::string& cmd, const std::vector<std::string>& args)
{
  // cv::Mat new_preview_image(cv::Size(COLS, ROWS), CV_8UC1, cv::Scalar(0,0,0));

  cv::Mat new_preview_image = cv::Mat::zeros(cv::Size(COLS, ROWS), CV_8UC1);
  // mmu_t* mmu = debug_mmu;
  // fprintf(stderr, "Hello, World!  0x%016" PRIx64 "\n", mmu->load_uint64(monitor_base));   
  //cv::Mat preview_image(cv::Size(COLS, ROWS), CV_8UC1, cv::Scalar(0,0,0));
  mmu_t* mmu = debug_mmu;
  reg_t val;
  uint32_t base = monitor_base;
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

  
  fprintf(stderr, "[INFO] ------------------------------------\n"
                  "[INFO] >> At 0x%x = 0x%lx \n", monitor_base, my_get_mem(monitor_base));
  fprintf(stderr, "[INFO] app_status   = %d \n", app_status);
  fprintf(stderr, "[INFO] spike_status = %d \n", spike_status);
  fprintf(stderr, "[INFO] n_rows       = %d \n", n_rows);
  fprintf(stderr, "[INFO] n_cols       = %d \n", n_cols);
  fprintf(stderr, "[INFO] n_channels   = %d \n", n_channels);
  fprintf(stderr, "[INFO] data         = 0x%ux \n", data);
  fprintf(stderr, "[INFO] ------------------------------------\n");

  for(unsigned int i = 0; i < n_rows; i++){
    for (unsigned int j = 0; j < n_cols; j++) {
        uint8_t x = 0xFF & mmu->load_uint8(pt_data + (i*n_cols + j));
       new_preview_image.at<uchar>(i, j) = x;
    }
  }

  cv::imshow("monitor_d", new_preview_image);
  cv::waitKey(0);
  cv::destroyWindow("monitor_d");
}


void sim_t::main()
{
  bool monitor_verbose = true;
  bool oscillo_verbose = true;

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
      show_monitor(monitor_base, monitor_verbose);
      monitor_verbose = false;
    }  
    if (oscillo){
      show_oscillo(oscillo_base, oscillo_verbose);
      cv::waitKey(OSCILLO_TIMEOUT);
      oscillo_verbose = false;
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
  // monitor = value;
  if (base_address == 0xffffffff)
  {
    monitor_base = -1;
    monitor = false;
    printf("[INFO] >> Monitor deactivated.\n");
    return;
  }
  if(monitor){
    monitor = true;
    if(base_address < 0x80000000)
      monitor_base = 0x81000000;      
    else
      monitor_base = base_address;

    printf("[INFO] >> Monitor at 0x%x \n", monitor_base);

    garbage = cv::Mat::zeros(cv::Size(COLS, ROWS), CV_8UC1);
  }
}

void sim_t::set_oscillo(bool value, uint32_t base_address)
{
  if (base_address == 0xffffffff)
  {
    oscillo_base = -1;
    oscillo = false;
    printf("[INFO] >> Oscillator deactivated.\n");
    return;
  }

  if(value){
    oscillo = true;
    if(base_address < 0x80000000)
      oscillo_base = 0x82000000;
    else
      oscillo_base = base_address;

    printf("[INFO] >> Oscillator at 0x%x \n", oscillo_base);
  }
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
