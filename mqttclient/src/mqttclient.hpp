#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <string>
#include <mosquitto.h>
#include <thread>
#include <tsqueue.hpp>
#include <promise.hpp>
#include <iostream>

class MQTTClient {

private:

  class DataTuple {
  public:
    std::string topic;
    std::string message;
    DataTuple(std::string topic, std::string message) {
      this->topic = topic;
      this->message = message;
    }
  };

  std::string host;
  ThreadSafeQueue<std::shared_ptr<DataTuple>> q;
  std::shared_ptr<std::thread> runner;
  int port;
  struct mosquitto *mosq;

public:

  MQTTClient(std::string host, int port) {

    ThreadSafeQueue<std::shared_ptr<DataTuple>> q();

    this->host = host;
    this->port = port;

    int keepalive = 20;
    bool clean_session = true;

    mosquitto_lib_init();

    mosq = mosquitto_new("overhead_tracker", clean_session, NULL);

    if(!mosq){
      std::cerr << "Error: Couldn't allocate memory" << std::endl;
      throw 1;
    }

    if(mosquitto_connect_bind(mosq, host.c_str(), port, keepalive, NULL)) {
      std::cerr << "Unable to connect to MQTT broker at " << host << ":" << port << std::endl;
      throw 2;
    }

  	mosquitto_loop_start(mosq);
  }

  ~MQTTClient(void) {
    q.enqueue(NULL);
    runner->join();
    runner.reset();
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
  }

  void start() {
    runner = std::make_shared<std::thread>(&MQTTClient::publish_loop, this);
  }

  void publish_loop(void) {
    while(true) {
      std::shared_ptr<DataTuple> message = q.dequeue();
      if(message == NULL) {
        std::cout << "Stopping MQTT client publisher..." << std::endl;
        break;
      }
      mosquitto_publish(mosq, NULL, message->topic.c_str() , message->message.length(), message->message.c_str(), 1, false);
    }
  }

  //TODO: Implement subscribe method!
  void subscribe(std::string topic) { }

  void async_publish(std::string topic, std::string message) {
    std::shared_ptr<DataTuple> msg = std::make_shared<DataTuple>(topic, message);
    q.enqueue(msg);
  }
};
#endif
