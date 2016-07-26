#include <countdownlatch.hpp>

#include <thread>
#include <iostream>

int main(void) {

  int count = 15000;

  std::thread threads[count];

  CountDownLatch l(count);

  auto lambda = [&l]() {l.count_down();};
  auto waitl = [&l]() {l.wait();};

  for (int i = 0; i < count; i++) {
    threads[i] = std::thread(lambda);
  }

  std::thread t3(waitl);

  t3.join();

  for (int i = 0; i < count; i++) {
    threads[i].join();
  }

  std::cout << "Successfully counted down: " << count << " threads!" << std::endl;
}
