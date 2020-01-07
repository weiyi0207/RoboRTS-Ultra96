/*
-- (c) Copyright 2018 Xilinx, Inc. All rights reserved.
--
-- This file contains confidential and proprietary information
-- of Xilinx, Inc. and is protected under U.S. and
-- international copyright and other intellectual property
-- laws.
--
-- DISCLAIMER
-- This disclaimer is not a license and does not grant any
-- rights to the materials distributed herewith. Except as
-- otherwise provided in a valid license issued to you by
-- Xilinx, and to the maximum extent permitted by applicable
-- law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
-- WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
-- AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
-- BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
-- INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
-- (2) Xilinx shall not be liable (whether in contract or tort,
-- including negligence, or under any other theory of
-- liability) for any loss or damage of any kind or nature
-- related to, arising under or in connection with these
-- materials, including for any direct, or any indirect,
-- special, incidental, or consequential loss or damage
-- (including loss of data, profits, goodwill, or any type of
-- loss or damage suffered as a result of any action brought
-- by a third party) even if such damage or loss was
-- reasonably foreseeable or Xilinx had been advised of the
-- possibility of the same.
--
-- CRITICAL APPLICATIONS
-- Xilinx products are not designed or intended to be fail-
-- safe, or for use in any application requiring fail-safe
-- performance, such as life-support or safety devices or
-- systems, Class III medical devices, nuclear facilities,
-- applications related to the deployment of airbags, or any
-- other applications that could lead to death, personal
-- injury, or severe property or environmental damage
-- (individually and collectively, "Critical
-- Applications"). Customer assumes the sole risk and
-- liability of any use of Xilinx products in Critical
-- Applications, subject only to applicable laws and
-- regulations governing limitations on product liability.
--
-- THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
-- PART OF THIS FILE AT ALL TIMES.
*/
#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <future>
#include <thread>
#include <signal.h>
#include <memory>
#include <fstream>
#include <string>
#include <iostream>
#include <ostream>
#include <chrono>
#include <cassert>
#include <glog/logging.h>
#include <xilinx/base/dpu/time_measure.hpp>
#include "./stat_samples.hpp"
#include "./image_list.hpp"
#include <xilinx/base/env_config.hpp>
DEF_ENV_PARAM(DEEPHI_DPU_CONSUMING_TIME, "0");
namespace xilinx {
namespace benchmark {


struct BenchMarkResult{
    long ret;
    StatSamples e2eSamples;
    StatSamples dpuSamples;
};

int g_num_of_threads = 1;
int g_num_of_seconds = 30;
std::string g_list_name = "image.list";
std::string g_report_file_name = "";
long g_total = 0;
double g_e2e_mean = 0.0;
double g_dpu_mean = 0.0;
bool g_stop = false;
template <typename T>
inline BenchMarkResult thread_main_for_performance(
    const ImageList* image_list,
    std::unique_ptr<T>&& model) {
  long ret = 0;
  StatSamples e2e_stat_samples(10000);
  StatSamples dpu_stat_samples(10000);
  for (ret = 0; ! g_stop; ++ret) {
      xilinx::base::TimeMeasure::getThreadLocalForDpu().reset();
      auto start = std::chrono::steady_clock::now();
      model->run((*image_list)[ret]);
      auto end = std::chrono::steady_clock::now();
      auto end2endtime =
          int(std::chrono::duration_cast<std::chrono::microseconds>(
              end - start).count());
      auto dputime = xilinx::base::TimeMeasure::getThreadLocalForDpu().get();

      e2e_stat_samples.addSample(end2endtime);
      dpu_stat_samples.addSample(dputime);
  }
  return BenchMarkResult{ret,std::move(e2e_stat_samples),std::move(dpu_stat_samples)};
}

static void signal_handler(int signal) { g_stop = true; }
static void usage() {
  std::cout << "usage: env dpbenchmark \n"
               " -l <log_file_name> \n"
               " -t <num_of_threads> \n"
               " -s <num_of_seconds> \n"
               " <image list file> \n" << std::endl;
}
inline void parse_opt(int argc, char * argv[]) {
  int opt = 0;

  while ((opt = getopt(argc, argv, "t:s:l:")) != -1) {
    switch (opt) {
      case 't':
        g_num_of_threads = std::stoi(optarg);
        break;
      case 's':
        g_num_of_seconds = std::stoi(optarg);
        break;
      case 'l':
        g_report_file_name = optarg;
        break;
      default:
        usage();
        exit(1);
    }
  }
  if (optind >= argc) {
    std::cerr << "Expected argument after options\n";
    exit(EXIT_FAILURE);
  }
  g_list_name = argv[optind];
  return;
}

static void report(std::ostream * p_out) {
    std::ostream & out = *p_out;
    float fps = ((float)g_total) / ((float)g_num_of_seconds);
    out << "FPS=" << fps << "\n";
    out << "E2E_MEAN=" << g_e2e_mean << "\n";
    out << "DPU_MEAN=" << g_dpu_mean << "\n";
    out << std::flush;
    return;
}


template <typename T>
inline int main_for_performance(int argc, char* argv[], T factory_method) {
  parse_opt(argc, argv);
  ENV_PARAM(DEEPHI_DPU_CONSUMING_TIME) = 1;
  auto lazy_load_image = false;
  auto image_list =
      std::unique_ptr<ImageList>(new ImageList(g_list_name, lazy_load_image));
  if(image_list->empty()) {
    LOG(FATAL) << "list of images are empty [" << image_list->to_string()
               << "]";
  }
  auto model = factory_method();
  using model_t = typename decltype(model)::element_type;
  auto width = model->getInputWidth();
  auto height = model->getInputHeight();
  image_list->resize_images(width, height);
  //
  std::vector<std::future<BenchMarkResult>> results;
  results.reserve(g_num_of_threads);

  for(int i = 0; i < g_num_of_threads; ++i) {
    // every thread should have its own model object.
    if (i == 0) {
      // the first thread reuse the model which is already created.
      results.emplace_back(std::async(thread_main_for_performance<model_t>, //
                                      image_list.get(),                     //
                                      std::move(model)));
    } else {
      results.emplace_back(std::async(thread_main_for_performance<model_t>, //
                                      image_list.get(),                     //
                                      factory_method()));
    }
  }
  signal(SIGALRM, signal_handler);
  alarm(g_num_of_seconds);


  for(int i = 0; i < g_num_of_seconds; i = i + 5) {
    LOG(INFO) << "waiting for " << i << "/" << g_num_of_seconds << " seconds, " << g_num_of_threads
              << " threads running";
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  }
  LOG(INFO) << "waiting for threads terminated";
  long total = 0;

  StatSamples e2eStatSamples(0);
  StatSamples dpuStatSamples(0);
  for(auto &r : results) {
    auto result = r.get();
    total = total + result.ret;
    e2eStatSamples.merge(result.e2eSamples);
    dpuStatSamples.merge(result.dpuSamples);
  }

  g_e2e_mean = e2eStatSamples.getMean();
  g_dpu_mean = dpuStatSamples.getMean();

  g_total = total;
  std::ostream* report_fs = &std::cout;
  auto fs = std::unique_ptr<std::ostream>{};
  if(!g_report_file_name.empty()) {
      LOG(INFO) << "writing report to " << g_report_file_name;
      fs = std::unique_ptr<std::ostream>{
          new std::ofstream(g_report_file_name.c_str(), std::ofstream::out)};
      report_fs = fs.get();
  } else {
      LOG(INFO) << "writing report to <STDOUT>";
  }

  report(report_fs);
  return 0;
}

}
}
