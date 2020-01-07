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
#ifndef DEEPHI_RING_QUEUE_HPP_
#define DEEPHI_RING_QUEUE_HPP_

#include <condition_variable>
#include <mutex>
#include <vector>

namespace xilinx {

/**
 * A thread safe queue with a size limit.
 * When it is full, it will overwrite the earliest element.
 */
template <typename T> class RingQueue {

public:
  explicit RingQueue(std::size_t capacity)
      : capacity_(capacity), buffer_(capacity), size_(0U), front_(0U),
        rear_(0U) {}
  /**
   * Return the maxium size of the queue.
   */
  std::size_t capacity() const { return capacity_; }

  /**
   * Return the size of the queue.
   */
  std::size_t size() const { return size_; }

  /**
   * Copy the value to the end of this queue.
   */
  void push(const T &new_value) {
    std::lock_guard<std::mutex> lock(mtx_);
    buffer_[rear_] = new_value;
    rear_ = (rear_ + 1) % capacity_;
    if (size_ >= capacity_) {
      front_ = rear_;
    } else {
      ++size_;
    }
  }

  /**
   * Get the first element in the queue and remove it from the queue.
   * Return false if the queue is empty
   */
  bool pop(T &value) {
    std::lock_guard<std::mutex> lock(mtx_);
    bool res = false;
    if (size_ > 0) {
      value = buffer_[front_];
      front_ = (front_ + 1) % capacity_;
      --size_;
      res = true;
    }
    return res;
  }

  /**
   * Look at the top of the queue. i.e. the element that would be poped.
   * Warning: the returned pointer can become dangled
   */
  T *pop() {
    std::lock_guard<std::mutex> lock(mtx_);
    T *res = nullptr;
    if (size_ > 0) {
      res = &buffer_[front_];
      front_ = (front_ + 1) % capacity_;
      --size_;
    }
    return res;
  }

  /**
   * Return the first element in the queue and remove it from the queue.
   * Warning: the returned pointer can become dangled
   */
  T *top() {
    std::lock_guard<std::mutex> lock(mtx_);
    T *res = nullptr;
    if (size_ > 0) {
      res = &buffer_[front_];
    }
    return res;
  }

private:
  std::size_t capacity_;
  std::vector<T> buffer_;
  std::size_t size_;
  std::size_t front_;
  std::size_t rear_;

  mutable std::mutex mtx_;
};
}

#endif
