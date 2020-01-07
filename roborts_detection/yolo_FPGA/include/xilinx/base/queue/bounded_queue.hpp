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
#ifndef DEEPHI_BOUNDED_QUEUE_HPP_
#define DEEPHI_BOUNDED_QUEUE_HPP_

#include "shared_queue.hpp"

namespace xilinx {
/**
 * A thread safe queue with a size limit.
 * It will block on push if full, and on pop if empty.
 */
template <typename T> class BoundedQueue : public SharedQueue<T> {

public:
  explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}

  /**
   * Return the maxium size of the queue.
   */
  std::size_t capacity() const { return this->capacity_; }

  /**
   * Copy the value to the end of this queue.
   * This is blocking.
   */
  void push(const T &new_value) override {
    std::unique_lock<std::mutex> lock(this->mtx_);
    this->cond_not_full_.wait(
        lock, [this]() { return this->internal_size() < this->capacity_; });
    this->internal_push(new_value);
    this->cond_not_empty_.notify_one();
  }

  /**
   * Copy the value to the end of this queue.
   * This will fail and return false if blocked for more than rel_time.
   */
  bool push(const T &new_value, const std::chrono::milliseconds &rel_time) {
    std::unique_lock<std::mutex> lock(this->mtx_);
    if (this->cond_not_full_.wait_for(lock, rel_time, [this]() {
          return this->internal_size() < this->capacity_;
        }) == false) {
      return false;
    }
    this->internal_push(new_value);
    this->cond_not_empty_.notify_one();
    return true;
  }

  /**
   * Look at the top of the queue. i.e. the element that would be poped.
   * This will fail and return false if blocked for more than rel_time.
   */
  bool top(T &value, const std::chrono::milliseconds &rel_time) {
    std::unique_lock<std::mutex> lock(this->mtx_);
    if (this->cond_not_empty_.wait_for(lock, rel_time, [this]() {
          return !this->internal_empty();
        }) == false) {
      return false;
    }
    this->internal_top(value);
    return true;
  }

  /**
   * Return the first element in the queue and remove it from the queue.
   * This is blocking.
   */
  void pop(T &value) override {
    std::unique_lock<std::mutex> lock(this->mtx_);
    this->cond_not_empty_.wait(lock,
                               [this]() { return !this->internal_empty(); });
    this->internal_pop(value);
    this->cond_not_full_.notify_one();
  }

  /**
   * Return the first element in the queue and remove it from the queue.
   * This will fail and return false if blocked for more than rel_time.
   */
  bool pop(T &value, const std::chrono::milliseconds &rel_time) override {
    std::unique_lock<std::mutex> lock(this->mtx_);
    if (this->cond_not_empty_.wait_for(lock, rel_time, [this]() {
          return !this->internal_empty();
        }) == false) {
      return false;
    }
    this->internal_pop(value);
    this->cond_not_full_.notify_one();
    return true;
  }

  /**
   * Return the first element in the queue that satisfies cond, and remove it
   * from the queue.
   * This is blocking.
   */
  bool pop(T &value, std::function<bool(const T &)> &cond) override {
    std::lock_guard<std::mutex> lock(this->mtx_);
    if (this->internal_empty())
      return false;
    auto it =
        std::find_if(this->internal_.begin(), this->internal_.end(), cond);
    if (it != this->internal_.end()) {
      value = std::move(*it);
      this->internal_.erase(it);
      this->cond_not_full_.notify_one();
      return true;
    }
    return false;
  }

  /**
   * Return the first element in the queue that satisfies cond, and remove it
   * from the queue.
   * This will fail and return false if blocked for more than rel_time.
   */
  bool pop(T &value, std::function<bool(const T &)> &cond,
           const std::chrono::milliseconds &rel_time) override {
    auto now = std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(this->mtx_);
    // Wait until not empty
    if (!this->cond_not_empty_.wait_for(
            lock, rel_time, [this]() { return !this->internal_empty(); })) {
      return false;
    }
    auto it =
        std::find_if(this->internal_.begin(), this->internal_.end(), cond);
    while (it == this->internal_.end()) {
      if (this->cond_not_empty_.wait_until(lock, now + rel_time) ==
          std::cv_status::timeout) {
        break;
      }
      it = std::find_if(this->internal_.begin(), this->internal_.end(), cond);
    }
    it = std::find_if(this->internal_.begin(), this->internal_.end(), cond);
    if (it != this->internal_.end()) {
      value = std::move(*it);
      this->internal_.erase(it);
      this->cond_not_full_.notify_one();
      return true;
    }
    return false;
  }

protected:
  std::size_t capacity_;

  std::condition_variable cond_not_full_;
};
}

#endif
