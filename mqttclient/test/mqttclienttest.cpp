#include <mqttclient.hpp>
#include <iostream>


void callback(std::string message) {
  std::cout << message << std::endl;
}

//std::function<void(std::string)> stdf_callback = &callback;

std::function<void(std::string)> stdf_callback = &callback;

int main(void) {

  MQTTClient m("localhost", 1884);
  m.start();

  m.subscribe("/tracker/overhead", stdf_callback);

  std::cin.ignore();

  m.async_publish("/tracker/overhead", "hi!");
  m.async_publish("/tracker/overhead", "hi!");
  m.async_publish("/tracker/overhead", "hi!");

  std::cin.ignore();

  m.unsubscribe("/tracker/overhead");

  std::cin.ignore();

  m.async_publish("/tracker/overhead", "hi!");
  m.async_publish("/tracker/overhead", "hi!");
  m.async_publish("/tracker/overhead", "hi!");

  std::cin.ignore();

  return 0;
}
