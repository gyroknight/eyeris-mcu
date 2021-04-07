#include "Barrier.hh"

Barrier::Barrier(size_t expected)
    : expected(expected), count(0), triggerCounter(0) {}

void Barrier::wait() {
  std::unique_lock<std::mutex> lock(mm);
  size_t currentTrigger = triggerCounter;
  if (++count == expected) {
    triggerCounter++;
    count = 0;
    lock.unlock();
    cv.notify_all();
  } else {
    cv.wait(lock, [this, currentTrigger] {
      return currentTrigger != triggerCounter;
    });
  }
}

void Barrier::release() {
  {
    std::lock_guard<std::mutex> lock(mm);
    triggerCounter++;
  }
  cv.notify_all();
}
