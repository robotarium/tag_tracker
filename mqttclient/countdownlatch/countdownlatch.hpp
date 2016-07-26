#ifndef COUNT_DOWN_LATCH_H
#define COUNT_DOWN_LATCH_H

#include <atomic>
#include <mutex>
#include <condition_variable>

class CountDownLatch {

private:
  std::atomic<int> count;
  mutable std::mutex m;
  std::condition_variable c;

public:
  CountDownLatch(int count) {
    this->count = count;
    std::mutex m();
    std::condition_variable c();
  }

  ~CountDownLatch(void)
  {}

  void count_down(void)
  {
    std::lock_guard<std::mutex> lock(m);

    //If we have more to count down, do so
    if(count > 0) {
      count--;
    }

    //If we've counted down, notify everyone waiting
    if(count <= 0) {
      c.notify_all();
    }
  }

  void wait(void)
  {
    //Get a lock on the mutex
    std::unique_lock<std::mutex> lock(m);

    // While our condition isn't satisfied, wait on the lock.  Protects against
    // Spurious wake-ups
    while(count > 0)
    {
      c.wait(lock);
    }
  }
};

#endif
