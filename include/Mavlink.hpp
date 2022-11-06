#pragma once

#include <atomic>
#include <functional>
#include <queue>
#include <mutex>
#include <memory>
#include <unordered_map>

#include <Connection.hpp>
#include <ThreadSafeQueue.hpp>

#include <mavlink.h>

namespace mavlink
{

using MessageCallback = std::function<void(const mavlink_message_t&)>;

struct MavCommand {
	MavCommand() = default;
	MavCommand(uint8_t sysid, uint8_t compid, uint16_t command)
		: sender_sysid(sysid)
		, sender_compid(compid)
		, command(command)
	{}
	uint8_t sender_sysid {};
	uint8_t sender_compid {};
	uint16_t command {};
};

struct ConfigurationSettings {
	std::string connection_url {};
	uint8_t sysid {};
	uint8_t compid {};
	uint8_t mav_type {};
	uint8_t mav_autopilot {};
};

class Mavlink
{
public:
	Mavlink(const ConfigurationSettings& settings);
	~Mavlink();

	void start();
	void stop();

	uint8_t sysid() const { return _settings.sysid; };
	uint8_t compid() const { return _settings.compid; };

	//-----------------------------------------------------------------------------
	// Message senders
	void send_message(const mavlink_message_t& message);
	void send_heartbeat();

	void send_status_text(std::string&& message, int severity = MAV_SEVERITY_CRITICAL);
	void send_command_ack(const MavCommand& mav_cmd, int mav_result);

	//-----------------------------------------------------------------------------
	// Helpers
	void subscribe_to_message(uint16_t message_id, const MessageCallback& callback);

	float get_rangefinder_distance_cm() { return static_cast<float>(_rangefinder_distance_cm); };

	bool connected() const { return _connection->connected(); };

	bool command_queue_pop(MavCommand* command) { return _command_queue.pop_front(command); };

private:
	// TESTING -- move to user code once this is a library
	void setup_subscriptions();

	//-----------------------------------------------------------------------------
	// Message handlers
	void handle_message(const mavlink_message_t& message);
	void handle_command_long(const mavlink_message_t& message);
	void handle_distance_sensor(const mavlink_message_t& message);
	void handle_param_request_list(const mavlink_message_t& message);
	void handle_param_set(const mavlink_message_t& message);
	//-----------------------------------------------------------------------------
	// Message senders
	void send_param_value(std::string name, float value, uint16_t index, uint16_t total_count);

private:
	ConfigurationSettings _settings {};

	std::unique_ptr<Connection> _connection {};

	std::atomic<uint16_t> _rangefinder_distance_cm {};

	ThreadSafeQueue<MavCommand> _command_queue {10};

	// Mavlink message ID --> callback(mavlink_message_t)
	std::mutex _subscriptions_mutex {};
	std::unordered_map<uint16_t, MessageCallback> _message_subscriptions {};
};

} // end namespace mavlink