#ifndef MQTT_CLIENT_THREE_H
#define MQTT_CLIENT_THREE_H

#include <string>
#include <client.h>
#include <thread>
#include <tsqueue.hpp>
#include <promise.hpp>
#include <iostream>

#include <iostream>
#include <cstdlib>
#include <map>
#include <vector>
#include <cstring>
#include "mqtt/client.h"
#include "mqtt/ipersistable.h"

const std::string ADDRESS("tcp://localhost:8080");
const std::string CLIENTID("SyncPublisher");
const std::string TOPIC("hello");

const std::string PAYLOAD1("Hello World!");

const char* PAYLOAD2 = "Hi there!";
const char* PAYLOAD3 = "Is anyone listening?";

const int QOS = 1;
const int TIMEOUT = 10000;

/////////////////////////////////////////////////////////////////////////////

class sample_mem_persistence : virtual public mqtt::iclient_persistence
{
	bool open_;
	std::map<std::string, mqtt::ipersistable_ptr> store_;

public:
	sample_mem_persistence() : open_(false) {}

	// "Open" the store
	virtual void open(const std::string& clientId, const std::string& serverURI) {
		std::cout << "[Opening persistence for '" << clientId
			<< "' at '" << serverURI << "']" << std::endl;
		open_ = true;
	}

	// Close the persistent store that was previously opened.
	virtual void close() {
		std::cout << "[Closing persistence store.]" << std::endl;
		open_ = false;
	}

	// Clears persistence, so that it no longer contains any persisted data.
	virtual void clear() {
		std::cout << "[Clearing persistence store.]" << std::endl;
		store_.clear();
	}

	// Returns whether or not data is persisted using the specified key.
	virtual bool contains_key(const std::string &key) {
		return store_.find(key) != store_.end();
	}

	// Gets the specified data out of the persistent store.
	virtual mqtt::ipersistable_ptr get(const std::string& key) const {
		std::cout << "[Searching persistence for key '"
			<< key << "']" << std::endl;
		auto p = store_.find(key);
		if (p == store_.end())
			throw mqtt::persistence_exception();
		std::cout << "[Found persistence data for key '"
			<< key << "']" << std::endl;

		return p->second;
	}
	/**
	 * Returns the keys in this persistent data store.
	 */
	virtual std::vector<std::string> keys() const {
		std::vector<std::string> ks;
		for (const auto& k : store_)
			ks.push_back(k.first);
		return ks;
	}

	// Puts the specified data into the persistent store.
	virtual void put(const std::string& key, mqtt::ipersistable_ptr persistable) {
		std::cout << "[Persisting data with key '"
			<< key << "']" << std::endl;

		store_[key] = persistable;
	}

	// Remove the data for the specified key.
	virtual void remove(const std::string &key) {
		std::cout << "[Persistence removing key '" << key << "']" << std::endl;
		auto p = store_.find(key);
		if (p == store_.end())
			throw mqtt::persistence_exception();
		store_.erase(p);
		std::cout << "[Persistence key removed '" << key << "']" << std::endl;
	}
};

/////////////////////////////////////////////////////////////////////////////

class callback : public virtual mqtt::callback
{
public:
	virtual void connection_lost(const std::string& cause) {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;
	}

	// We're not subscrived to anything, so this should never be called.
	virtual void message_arrived(const std::string& topic, mqtt::message_ptr msg) {
	}

	virtual void delivery_complete(mqtt::idelivery_token_ptr tok) {
		std::cout << "\n\t[Delivery complete for token: "
			<< (tok ? tok->get_message_id() : -1) << "]" << std::endl;
	}
};

// --------------------------------------------------------------------------


class MQTTClientThree {

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

  sample_mem_persistence persist;
	mqtt::connect_options connOpts;
  std::shared_ptr<mqtt::client> client;
  int port;

public:

  MQTTClientThree(std::string host, int port) {

    client = std::make_shared<mqtt::client>(ADDRESS, CLIENTID, &persist);

    ThreadSafeQueue<std::shared_ptr<DataTuple>> q();

    this->host = host;
    this->port = port;

    int keepalive = 60;
    bool clean_session = true;
    int rc;

  	connOpts.set_keep_alive_interval(20);
  	connOpts.set_clean_session(true);

    std::cout << "Connecting..." << std::flush;
    client->connect(connOpts);
    std::cout << "OK" << std::endl;
  }

  ~MQTTClientThree(void) {
    q.enqueue(NULL);
		client->disconnect();
    runner->join();
  }

  void start() {
    runner = std::make_shared<std::thread>(&MQTTClientThree::publish_loop, this);
  }

  void publish_loop(void) {
    while(true) {
      std::shared_ptr<DataTuple> message = q.dequeue();
      if(message == NULL) {
        std::cout << "Stopping MQTT client publisher..." << std::endl;
        break;
      }
      //MQTTClient_publish(client, message->topic.c_str(), message->message.length(), (void*) message->message.c_str(), QOS, 0, &token);

      std::cout << "Sending message..." << std::flush;
      mqtt::message_ptr pubmsg = std::make_shared<mqtt::message>(message->message);
      pubmsg->set_qos(QOS);
      client->publish(message->topic, pubmsg);
      std::cout << "OK" << std::endl;
    }
  }

  void subscribe(std::string topic) { }

  void async_publish(std::string topic, std::string message) {
    std::shared_ptr<DataTuple> msg = std::make_shared<DataTuple>(topic, message);
    q.enqueue(msg);
  }
};
#endif
