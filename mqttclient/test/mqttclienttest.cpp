#include <mqttclient3.hpp>
#include <iostream>
int main(void) {

  MQTTClientThree m("localhost", 8080);
  m.start();

  m.async_publish("/tracker/overhead", "hi!");

  return 0;
}
