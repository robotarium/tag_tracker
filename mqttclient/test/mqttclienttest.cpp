#include <mqttclient.hpp>
#include <iostream>


void callback(std::string topic, std::string message) {
  std::cout << topic << ": " << message << std::endl;
}

//std::function<void(std::string)> stdf_callback = &callback;

std::function<void(std::string, std::string)> stdf_callback = &callback;

int main(void) {

	//MQTTClient m("localhost", 1884);
  MQTTClient m("192.168.1.2", 1884);

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
