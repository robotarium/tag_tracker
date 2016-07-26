#ifndef PROMISE_H
#define PROMISE_h

#include <mutex>
#include <condition_variable>

template <class T> class Promise {

private:
  T cv;
  mutable std::mutex m;
  std::condition_variable c;

public:
  Promise(void) {
    cv = NULL;
    m = std::mutex();
    c = std::condition_variable();
  }

  ~Promise(void) {}

  int wait() {
    std::unique_lock<std::mutex> lock(m);

    // While our condition isn't satisfied, wait on the lock.  Protects against
    // Spurious wake-ups
    while(cv == NULL)
    {
      c.wait(lock);
    }
    return cv;
  }

  void fulfill(T val) {
    std::lock_guard<std::mutex> lock(m);
    if(cv != NULL) {
      throw 1;
    }
    
    cv = val;
    c.notify_all();
  }
};
#endif
