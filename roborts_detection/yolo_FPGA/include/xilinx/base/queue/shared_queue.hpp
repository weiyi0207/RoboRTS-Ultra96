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
#ifndef DEEPHI_SHARED_QUEUE_HPP_
#define DEEPHI_SHARED_QUEUE_HPP_

#include <algorithm>
#include <functional>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>

namespace xilinx {
/**
 * A thread safe queue.
 */
template <typename T> class SharedQueue {

public:
  /**
   * Return the size of the queue.
   */
  virtual std::size_t size() const {
    std::lock_guard<std::mutex> lock(this->mtx_);
    return this->internal_size();
  }

  /**
   * Return true if the queue is empty, and false otherwise.
   */
  virtual bool empty() const {
    std::lock_guard<std::mutex> lock(this->mtx_);
    return this->internal_empty();
  }

  /**
   * Copy the value to the end of this queue.
   */
  virtual void push(const T &new_value) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    this->internal_push(new_value);
    this->cond_not_empty_.notify_one();
  }

  /**
   * Get the first element in the queue and remove it from the queue.
   * This is blocking
   */
  virtual void pop(T &value) {
    std::unique_lock<std::mutex> lock(this->mtx_);
    this->cond_not_empty_.wait(lock,
                               [this]() { return !(this->internal_empty()); });
    this->internal_pop(value);
  }

  /**
   * Get the first element in the queue and remove it from the queue.
   * This will fail and return false if blocked for more than rel_time.
   */
  virtual bool pop(T &value, const std::chrono::milliseconds &rel_time) {
    std::unique_lock<std::mutex> lock(this->mtx_);
    if (this->cond_not_empty_.wait_for(lock, rel_time, [this]() {
          return !(this->internal_empty());
        }) == false) {
      return false;
    }
    this->internal_pop(value);
    return true;
  }

  /**
   * Get the first element in the queue that satisfies cond, and remove it
   * from the queue.
   * This will fail and return false if no such element.
   */
  virtual bool pop(T &value, std::function<bool(const T &)> &cond) {
    std::lock_guard<std::mutex> lock(this->mtx_);
    if (this->internal_empty())
      return false;
    auto it = std::find_if(internal_.begin(), internal_.end(), cond);
    if (it != internal_.end()) {
      value = std::move(*it);
      internal_.erase(it);
      return true;
    }
    return false;
  }

  /**
   * Get the first element in the queue that satisfies cond, and remove it
   * from the queue.
   * This will fail and return false if no such element, or blocked for more
   * than rel_time.
   */
  virtual bool pop(T &value, std::function<bool(const T &)> &cond,
                   const std::chrono::milliseconds &rel_time) {
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
      return true;
    }
    return false;
  }

protected:
  inline virtual std::size_t internal_size() const {
    return this->internal_.size();
  }
  inline virtual bool internal_empty() const { return this->internal_.empty(); }
  inline virtual void internal_push(const T &new_value) {
    this->internal_.emplace_back(new_value);
  }
  inline virtual void internal_pop(T &value) {
    value = std::move(this->internal_.front());
    this->internal_.pop_front();
  }
  inline virtual void internal_top(T &value) {
    value = this->internal_.front();
  }

  mutable std::mutex mtx_;
  std::condition_variable cond_not_empty_;

  std::deque<T> internal_;
};
}

#endif
