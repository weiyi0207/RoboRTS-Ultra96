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
#include <xilinx/base/queue/bounded_queue.hpp>
#include <opencv2/core.hpp>
#include <thread>
#include <unistd.h>
#include <type_traits>
#ifndef USE_DRM
#define USE_DRM 0
#endif
#if USE_DRM
#include "./dpdrm.hpp"
#endif

namespace xilinx {
namespace demo {
struct FrameInfo {
  int channel_id;
  unsigned long frame_id;
  cv::Mat mat;
};
using queue_t = xilinx::BoundedQueue<FrameInfo>;
struct MyThread {
  static void main_proxy(MyThread* me) { return me->main(); }
  void main() {
    LOG(INFO) << "thread [" << name() << "] is started";
    while (!stop_) {
      auto run_ret = run();
      if (!stop_) {
        stop_ = run_ret != 0;
      }
    }
    LOG(INFO) << "thread [" << name() << "] is ended";
  }

  virtual int run() = 0;

  virtual std::string name() = 0;

  explicit MyThread() : stop_(false), thread_{nullptr} {}

  virtual ~MyThread() {  //
  }

  void start() {
    LOG(INFO) << "thread [" << name() << "] is starting";
    thread_ = std::unique_ptr<std::thread>(new std::thread(main_proxy, this));
  }

  void stop() { stop_ = true; }

  void wait() {
    if (thread_) {
      LOG(INFO) << "waiting for [" << name() << "] ended";
      thread_->join();
    }
  }
  bool is_stopped() { return stop_; }

  bool stop_;
  std::unique_ptr<std::thread> thread_;
};

struct DecodeThread : public MyThread {
  DecodeThread(int channel_id, const std::string& video_file, queue_t* queue)
      : MyThread{},
        channel_id_{channel_id},
        video_file_{video_file},
        frame_id_{0},
        queue_{queue} {}
  virtual ~DecodeThread() {}

  virtual int run() override {
    auto is_camera = video_file_.size() == 1 && video_file_[0] >= '0' &&
                     video_file_[0] <= '9';
    auto video_stream = std::unique_ptr<cv::VideoCapture>(
        is_camera ? new cv::VideoCapture(std::stoi(video_file_))
                  : new cv::VideoCapture(video_file_));
    auto & cap = *video_stream.get();
    if (!cap.isOpened()) {
      LOG(ERROR) << "cannot open file " << video_file_;
      return -1;
    }
    while (!is_stopped()) {
      cv::Mat image;
      cap >> image;
      auto video_ended = image.empty();
      if (video_ended) {
        return 0;
      }
      LOG(INFO) << "decode queue size " << queue_->size();
      while (!queue_->push(FrameInfo{channel_id_, ++frame_id_, image},
                           std::chrono::milliseconds(500))) {
        if (is_stopped()) {
          return -1;
        }
      }
    }
    return false;
  }

  virtual std::string name() override {
    return std::string {
      "DedodeThread-"
    }
    +std::to_string(channel_id_);
  }

  int channel_id_;
  std::string video_file_;
  unsigned long frame_id_;
  queue_t* queue_;
};

struct GuiThread : public MyThread {
  static std::shared_ptr<GuiThread> instance() {
    std::weak_ptr<GuiThread> the_instance;
    std::shared_ptr<GuiThread> ret;
    if (the_instance.expired()) {
      ret = std::make_shared<GuiThread>();
      the_instance = ret;
    }
    ret = the_instance.lock();
    assert(ret != nullptr);
#if USE_DRM
    xilinx::demo::imshow_open();
#endif
    return ret;
  }

  GuiThread()
      : MyThread{},
        queue_{new queue_t{
            10}  // assuming GUI is not bottleneck, 10 is high enough
        },
        inactive_counter_{0} {}
  virtual ~GuiThread() {  //
#if USE_DRM
    xilinx::demo::imshow_close();
#endif
  }

  virtual int run() override {
    FrameInfo frame;
    if (!queue_->pop(frame, std::chrono::milliseconds(500))) {
      inactive_counter_++;
      if (inactive_counter_ > 10) {
        // inactive for 5 second, stop
        return 1;
      } else {
        return 0;
      }
    }
    inactive_counter_ = 0;
#if USE_DRM
    xilinx::demo::imshow(frame.mat);
    // TODO test key
#else
    cv::imshow(std::string { "CH-" } + std::to_string(frame.channel_id),
               frame.mat);
    auto key = cv::waitKey(1);
    if (key == 27) {
      return 1;
    }
#endif
    LOG(INFO) << " gui queue size " << queue_->size();
    while (!queue_->empty()) {
      queue_->pop(frame);
    }
    return 0;
  }

  virtual std::string name() override {
    return std::string{"GUIThread"};
  }

  queue_t* getQueue() { return queue_.get(); }

  std::unique_ptr<queue_t> queue_;
  int inactive_counter_;
};

template <typename dpu_model_type_t, typename ProcessResult>
struct DpuThread : public MyThread {
  DpuThread(std::unique_ptr<dpu_model_type_t>&& dpu_model,
            const ProcessResult& processor, queue_t* queue_in,
            queue_t* queue_out)
      : MyThread{},
        dpu_model_{std::move(dpu_model)},
        processor_{processor},
        queue_in_{queue_in},
        queue_out_{queue_out} {}
  virtual ~DpuThread() {}

  virtual int run() override {
    FrameInfo frame;
    if (!queue_in_->pop(frame, std::chrono::milliseconds(500))) {
      return 0;
    }
    if (dpu_model_) {
      auto result = dpu_model_->run(frame.mat);
      frame.mat = processor_(frame.mat, result, false);
    }
    LOG(INFO) << "dpu queue size " << queue_out_->size();
    while (!queue_out_->push(frame, std::chrono::milliseconds(500))) {
      if (is_stopped()) {
        return -1;
      }
    }
    return 0;
  }

  virtual std::string name() override {
    return std::string{"DPU-"};
  }
  std::unique_ptr<dpu_model_type_t> dpu_model_;
  const ProcessResult& processor_;
  queue_t* queue_in_;
  queue_t* queue_out_;
};

struct SortingThread : public MyThread {
  SortingThread(queue_t* queue_in, queue_t* queue_out)
      : MyThread{}, queue_in_{queue_in}, queue_out_{queue_out}, frame_id_{0} {}
  virtual ~SortingThread() {}
  virtual int run() override {
    FrameInfo frame;
    frame_id_++;
    auto frame_id = frame_id_;
    auto cond =
        std::function<bool(const FrameInfo&)>{[frame_id](const FrameInfo& f) {
          // sorted by frame id
          return f.frame_id <= frame_id;
        }};
    if (!queue_in_->pop(frame, cond, std::chrono::milliseconds(500))) {
      return 0;
    }
    auto now = std::chrono::steady_clock::now();
    float fps = -1.0f;
    long duration = 0;
    if (!points_.empty()) {
      auto end = points_.back();
      duration = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - end).count();
      float duration2 = (float)duration;
      float total = (float)points_.size();
      fps = total / duration2 * 1000.0f;
      auto x = 10;
      auto y = 20;
      cv::putText(frame.mat, std::string("FPS: ") + std::to_string(fps),
                  cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(20, 20, 180), 2, 1);
    }
    LOG(INFO) << "frame id " << frame.frame_id << " sorting queue size "
              << queue_out_->size() << "   FPS: " << fps;
    points_.push_front(now);
    if (duration > 2000) {  // sliding window for 2 seconds.
      points_.pop_back();
    }
    while (!queue_out_->push(frame, std::chrono::milliseconds(500))) {
      if (is_stopped()) {
        return -1;
      }
    }
    return 0;
  }

  virtual std::string name() override {
    return std::string{"DPU-"};
  }
  queue_t* queue_in_;
  queue_t* queue_out_;
  unsigned long frame_id_;
  std::deque<std::chrono::time_point<std::chrono::steady_clock>> points_;
};
inline void usage_video(const char* progname) {
  std::cout << "usage: " << progname << "      -t <num_of_threads>\n"
            << "      <video file name>\n" << std::endl;
  return;
}
/*
  global command line options
 */
static int g_num_of_threads = 1;
static std::string g_avi_file = "";

inline void parse_opt(int argc, char* argv[]) {
  int opt = 0;

  while ((opt = getopt(argc, argv, "r:t:s:l:m:p:")) != -1) {
    switch (opt) {
      case 't':
        g_num_of_threads = std::stoi(optarg);
        break;
      default:
        usage_video(argv[0]);
        exit(1);
    }
  }
  if (optind >= argc) {
    std::cerr << "Expected argument after options\n";
    exit(EXIT_FAILURE);
  }
  g_avi_file = argv[optind];
  return;
}

template <typename FactoryMethod, typename ProcessResult>
int main_for_video_demo(int argc, char* argv[],
                        const FactoryMethod& factory_method,
                        const ProcessResult& process_result) {
  parse_opt(argc, argv);
  auto channel_id = 0;
  auto decode_queue = std::unique_ptr<queue_t>{new queue_t{5}};
  auto decode_thread = std::unique_ptr<DecodeThread>(
      new DecodeThread{channel_id, g_avi_file, decode_queue.get()});
  using ptr_dpu_model_type_t = decltype(factory_method().get());
  using dpu_model_type_t =
      typename std::remove_pointer<ptr_dpu_model_type_t>::type;
  auto dpu_thread = std::vector<
      std::unique_ptr<DpuThread<dpu_model_type_t, ProcessResult>>>{};
  auto sorting_queue =
      std::unique_ptr<queue_t>(new queue_t(5 * g_num_of_threads));
  auto gui_thread = GuiThread::instance();
  auto gui_queue = gui_thread->getQueue();
  for (int i = 0; i < g_num_of_threads; ++i) {
    dpu_thread.emplace_back(new DpuThread<dpu_model_type_t, ProcessResult>{
        factory_method(),   process_result,
        decode_queue.get(), sorting_queue.get()});
  }
  auto sorting_thread = std::unique_ptr<SortingThread>(
      new SortingThread(sorting_queue.get(), gui_queue));
  // start everything
  decode_thread->start();
  for (auto& dpu : dpu_thread) {
    dpu->start();
  }
  sorting_thread->start();
  gui_thread->start();
  gui_thread->wait();
  // stop everything
  decode_thread->stop();
  for (auto& dpu : dpu_thread) {
    dpu->stop();
  }
  sorting_thread->stop();
  // wait everything
  decode_thread->wait();
  for (auto& dpu : dpu_thread) {
    dpu->wait();
  }
  sorting_thread->wait();
  LOG(INFO) << "BYEBYE";
  return 0;
}
static inline void usage_jpeg(const char* progname) {
  std::cout << "usage : " << progname << " <img_url> [<img_url> ...]"
            << std::endl;
}
template <typename FactoryMethod, typename ProcessResult>
int main_for_jpeg_demo(int argc, char* argv[],
                       const FactoryMethod& factory_method,
                       const ProcessResult& process_result) {
  if (argc <= 1) {
    usage_jpeg(argv[0]);
    exit(1);
  }
  auto model = factory_method();
  for (int i = 1; i < argc; ++i) {
    auto image_file_name = std::string{argv[i]};
    auto image = cv::imread(image_file_name);
    if (image.empty()) {
      LOG(FATAL) << "cannot load " << image_file_name << std::endl;
      abort();
    }
    auto result = model->run(image);
    image = process_result(image, result, true);
    auto out_file =
        image_file_name.substr(0, image_file_name.size() - 4) + "_result.jpg";
    cv::imwrite(out_file, image);
    LOG(INFO) << "result image write to " << out_file;
  }
  LOG(INFO) << "BYEBYE";
  return 0;
}
}
}
