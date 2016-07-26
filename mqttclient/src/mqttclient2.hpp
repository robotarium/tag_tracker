#ifndef MQTT_CLIENT_TWO_H
#define MQTT_CLIENT_TWO_H

#include <string>
#include "MQTTClient.h"
#include <thread>
#include <tsqueue.hpp>
#include <promise.hpp>
#include <iostream>

#define ADDRESS     "tcp://192.168.1.22:8080"
#define CLIENTID    "ExampleClientPub"
#define QOS         1
#define TIMEOUT     10000L

class MQTTClientTwo {

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
  MQTTClient client;
  MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
  MQTTClient_deliveryToken token;
  MQTTClient_message pubmsg = MQTTClient_message_initializer;
  int port;

public:

  MQTTClientTwo(std::string host, int port) {

    ThreadSafeQueue<std::shared_ptr<DataTuple>> q();

    this->host = host;
    this->port = port;

    int keepalive = 60;
    bool clean_session = true;
    int rc;

    MQTTClient_create(&client, ADDRESS, CLIENTID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;

    if ((rc = MQTTClient_connect(client, &conn_opts)) != MQTTCLIENT_SUCCESS)
    {
        printf("Failed to connect, return code %d\n", rc);
        exit(-1);
    }
  }

  ~MQTTClientTwo(void) {
    q.enqueue(NULL);
    runner->join();
    runner.reset();
    MQTTClient_disconnect(client, 1000L);
    MQTTClient_destroy(&client);
  }

  void start() {
    runner = std::make_shared<std::thread>(&MQTTClientTwo::publish_loop, this);
  }

  void publish_loop(void) {
    while(true) {
      std::shared_ptr<DataTuple> message = q.dequeue();
      if(message == NULL) {
        std::cout << "Stopping MQTT client publisher..." << std::endl;
        break;
      }
      MQTTClient_publish(client, message->topic.c_str(), message->message.length(), (void*) message->message.c_str(), QOS, 0, &token);
    }
  }

  void subscribe(std::string topic) { }

  void async_publish(std::string topic, std::string message) {
    std::shared_ptr<DataTuple> msg = std::make_shared<DataTuple>(topic, message);
    q.enqueue(msg);
  }
};
#endif
