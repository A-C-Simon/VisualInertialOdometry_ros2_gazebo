#pragma once
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

namespace orb_fast {
// Jobs are synchronous pairs; both finish before run returns. Jobs must not
// recursively call this pool. Callers must finish before destroying the pool.
class StereoWorkers {
public:
  struct Job {
    void (*function)(void *);
    void *context;
  };
  StereoWorkers() {
    try {
      for (unsigned side = 0; side < 2; ++side)
        workers_[side] = std::thread(&StereoWorkers::work, this, side);
    } catch (...) {
      stop();
      throw;
    }
  }
  ~StereoWorkers() { stop(); }
  StereoWorkers(const StereoWorkers &) = delete;
  StereoWorkers &operator=(const StereoWorkers &) = delete;
  void run(Job left, Job right) {
    std::unique_lock<std::mutex> dispatch(dispatch_);
    std::unique_lock<std::mutex> lock(state_);
    jobs_[0] = left;
    jobs_[1] = right;
    errors_[0] = errors_[1] = nullptr;
    finished_ = 0;
    ++generation_;
    ready_.notify_all();
    done_.wait(lock, [this] { return finished_ == 2; });
    auto error = errors_[0] ? errors_[0] : errors_[1];
    lock.unlock();
    if (error)
      std::rethrow_exception(error);
  }

private:
  void stop() {
    std::unique_lock<std::mutex> dispatch(dispatch_);
    {
      std::lock_guard<std::mutex> lock(state_);
      stopping_ = true;
    }
    ready_.notify_all();
    for (auto &worker : workers_)
      if (worker.joinable())
        worker.join();
  }
  void work(unsigned side) {
    unsigned long long seen = 0;
    std::unique_lock<std::mutex> lock(state_);
    for (;;) {
      ready_.wait(lock, [&] { return stopping_ || generation_ != seen; });
      if (stopping_)
        return;
      seen = generation_;
      auto job = jobs_[side];
      lock.unlock();
      std::exception_ptr error;
      try {
        job.function(job.context);
      } catch (...) {
        error = std::current_exception();
      }
      lock.lock();
      errors_[side] = error;
      ++finished_;
      done_.notify_one();
    }
  }
  std::mutex dispatch_, state_;
  std::condition_variable ready_, done_;
  Job jobs_[2]{};
  std::exception_ptr errors_[2];
  unsigned finished_ = 0;
  unsigned long long generation_ = 0;
  bool stopping_ = false;
  std::thread workers_[2];
};
} // namespace orb_fast
