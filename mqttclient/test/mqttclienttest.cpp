#include <mqttclient.hpp>
#include <iostream>
int main(void) {

  MQTTClient m("localhost", 8080);
  m.start();

  m.async_publish("/tracker/overhead", "hi!");

  return 0;
}
