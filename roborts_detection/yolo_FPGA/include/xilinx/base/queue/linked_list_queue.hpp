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
#ifndef DEEPHI_LINKED_LIST_QUEUE_HPP_
#define DEEPHI_LINKED_LIST_QUEUE_HPP_

#include <cassert>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <time.h>
namespace xilinx {
// Work in progress
template <typename T> class LinkedListQueue {
  using mutex_type = std::mutex;

public:
  class Cons {
  public:
    Cons(std::unique_ptr<T> &&car, std::unique_ptr<Cons> &&cdr)
        : car_(std::move(car)), cdr_(std::move(cdr)) {}
    ~Cons() {}
    Cons() = delete;
    Cons(const Cons &) = delete;
    Cons &operator()(const Cons &) = delete;

    std::unique_ptr<T> &Car() { return car_; }

  private:
    std::unique_ptr<T> car_;
    std::unique_ptr<Cons> cdr_;
    friend class LinkedListQueue;
  };

public:
  LinkedListQueue()
      : size_{0}, lock_for_recv_{}, lock_for_queue_{}, not_empty{},
        header_{nullptr}, tail_{&header_} {}
  ~LinkedListQueue() {
    // 这里有一个 stack overflow 的问题，如果队列里面元素太多，
    // 析构 header_ 的时候会导致 stack overflow
  }
  // 这个函数不会导致递归拿锁。
  template <typename... Args> void send(Args &&... args) {
    // 注意要先构造对象，再拿锁。构造过程中，也许（不太可能）会调
    // 用 send/receieve 函数，但是不会递归拿锁。
    send_elt(std::unique_ptr<Cons>(new Cons(
        std::unique_ptr<T>(new T(std::forward<Args>(args)...)), nullptr)));
  }

  int size() const { return size_; }

  void send_unique_ptr(std::unique_ptr<T> obj) {
    // 注意要先构造对象，再拿锁。构造过程中，也许（不太可能）会调
    // 用 send/receieve 函数，但是不会递归拿锁。
    // 如果拿锁构造对象的话，也许会出现循环加锁？
    std::unique_ptr<typename LinkedListQueue<T>::Cons> x =
        std::unique_ptr<Cons>(
            new Cons(std::unique_ptr<T>(std::move(obj)), nullptr));
    send_elt(std::move(x));
  }
  // __attribute__ ((noinline)) is necessary, otherwise, '-Os' will result in
  // the following error
  // terminate called after throwing an instance of 'std::system_error'
  //   what():  Operation not permitted
  //  Aborted
  // but 'O2' and 'O0' has no such problem.
  // if add someting like std::cerr << "hello", no such error again, I guess it
  // might due the function inline.
  void __attribute__((noinline))
  send_elt(std::unique_ptr<typename LinkedListQueue<T>::Cons> p_new_elt) {
    //
    // 这里可以优化，变成无锁算法？
    // 不行，因为也许多个线程同时添加元素。compare and set
    // 也许可以，但是没有继续研究了。
    std::unique_lock<mutex_type> guard(lock_for_queue_);
    // 在结尾添加这个新构造的对象，tail_ 是一个 Cons_ 的指针。
    *tail_ = std::move(p_new_elt);
    tail_ = &((*tail_)->cdr_);
    size_++;
    not_empty.notify_one();
    guard.unlock();
    return;
  }

  std::unique_ptr<T> receive(long timeout_ms = -1) {
    return receive(nullptr, timeout_ms);
  }

  // 这个函数不会导致递归拿锁。
  std::unique_ptr<T> receive(const std::function<bool(const T &p)> &cond,
                             long timeout_ms = -1) {
    std::lock_guard<mutex_type> only_one_thread_can_enter_recv(lock_for_recv_);
    /*
      THIS IS THE PREMATURE OPTIMIZATION, COMMENT IT OUT FOR YOUR REFERENCE.

      // first search without locking
      // tail_ might not be up to date, but it is always consistent,
      // i.e. tail_ == &last_elt.cdr; see send();

      for (p = &header_; p != tail_; p = &(*p)->cdr_) {
      if (!cond || cond(*((*p)->Car()))) {
        // is it necessary to lock?
        std::lock_guard<mutex_type> guard(lock_for_queue_);
        std::unique_ptr<MessageType> ret = std::move((*p)->Car());
        // deconstruct the element;
        RemoveElementFromList(p);
        return ret;
      }
    }
    */
    // search with lock
    auto start = timeout_ms == 0 ? 0 : get_time_ms();
    // 注意这个锁放在了 while 循环外面。原来放在里面的，因为有
    // condition_variable not_empty。这个 not_empty.wait() 会在循
    // 环结尾，释放这个锁。
    std::unique_lock<mutex_type> guard(lock_for_queue_);
    long wait_ms_left = timeout_ms;
    std::unique_ptr<Cons> *p = &header_;
    while (true) {
      // *p is likely to be null, i.e. that last element, waiting for
      // *some one invoking send() in the context of another thread.
      while (*p) {
        // *p is unique_ptr<Cons>, it means: `while it is not the
        // last element`.
        // if cond is empty function, assuming it is true for any
        // message.
        if (!cond || cond(*((*p)->Car()))) {
          std::unique_ptr<T> ret = std::move((*p)->Car());
          RemoveElementFromList(p);
          size_--;
          return ret;
        } else {
          p = &(*p)->cdr_;
        }
      }
      if (timeout_ms < 0) {
        // 释放锁， 永远等，再次拿锁
        not_empty.wait(guard, [this, p]() { return size_ > 0 && *p; });
        continue; // 继续，有可能得到满足条件的值
      } else if (timeout_ms == 0) {
        // 不等待，立刻返回
        break;
      } else {
        auto now = get_time_ms();
        wait_ms_left -= now - start;
        start = now;
        if (wait_ms_left <= 0) {
          break; // return nullptr
        }
        // 释放锁，等到队列非空之后，再次拿锁
        auto is_empty =
            not_empty.wait_for(guard, std::chrono::milliseconds(wait_ms_left),
                               [this]() { return size_ > 0; });
        p = tail_;
        if (is_empty) {
          break; // return nullptr
        } else {
          continue; // 再来一次有可能满足条件.
        }
      }
    }
    return nullptr;
  }

private:
  static long get_time_ms(void) {
    struct timespec tv;
    auto r = clock_gettime(CLOCK_MONOTONIC, &tv);
    long ret = -1;
    if (r == 0) {
      ret = ((long)tv.tv_sec) * 1000;
      ret += ((long)tv.tv_nsec) / 1000000;
    } else {
      assert(false && "todo handle error here?");
    }
    return ret;
  }
  int size_;
  mutex_type lock_for_recv_;
  mutex_type lock_for_queue_;
  std::condition_variable not_empty;

private:
  std::unique_ptr<Cons> header_;
  std::unique_ptr<Cons> *tail_;
  void RemoveElementFromList(std::unique_ptr<Cons> *p) {
    if (!((*p)->cdr_)) {
      // if it is the last element, update the tail_ pointer.
      assert(tail_ == &((*p)->cdr_));
      tail_ = p;
    }
    *p = std::move((*p)->cdr_);
  }
};
}

#endif
