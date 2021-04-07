#if !defined(__BARRIER_H__)
#define __BARRIER_H__

#include <condition_variable>
#include <cstddef>
#include <mutex>

class Barrier {
 public:
  Barrier(size_t expected);
  void wait();
  void release();

 private:
  std::condition_variable cv;
  std::mutex mm;
  const size_t expected;
  size_t count;
  size_t triggerCounter;
};

#endif  // __BARRIER_H__