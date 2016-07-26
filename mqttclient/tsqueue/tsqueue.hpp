#ifndef TSQUEUE_H
#define TSQUEUE_H

#include <queue>
#include <mutex>
#include <condition_variable>

template <class T> class ThreadSafeQueue {

private:
  std::queue<T> q;
  mutable std::mutex m;
  std::condition_variable c;

public:
  ThreadSafeQueue(void) {

    q = std::queue<T>();
    std::mutex m();
    std::condition_variable c();
  }

  ~ThreadSafeQueue(void)
  {}

  void enqueue(T t)
  {
    std::lock_guard<std::mutex> lock(m);
    q.push(t);
    c.notify_one();
  }

  T dequeue(void)
  {

    //Get a lock on the mutex
    std::unique_lock<std::mutex> lock(m);

    // While our condition isn't satisfied, wait on the lock.  Protects against
    // Spurious wake-ups
    while(q.empty())
    {
      c.wait(lock);
    }
    T val = q.front();
    q.pop();
    return val;
  }
};

#endif
