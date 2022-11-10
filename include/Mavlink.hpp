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
	bool emit_heartbeat {};
};

struct Parameter {

	std::string name {};

	union {
		float float_value;
		int int_value;
	};

	uint16_t index {};
	uint16_t total_count {};
	uint8_t type {}; // MAV_PARAM_TYPE
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

	ConfigurationSettings settings() { return _settings; };

	void subscribe_to_message(uint16_t message_id, const MessageCallback& callback);
	void handle_message(const mavlink_message_t& message);

	//-----------------------------------------------------------------------------
	// Message senders
	void send_message(const mavlink_message_t& message);
	void send_heartbeat();
	void send_status_text(std::string&& message, int severity = MAV_SEVERITY_CRITICAL);
	void send_command_ack(const MavCommand& mav_cmd, MAV_RESULT mav_result);

	//-----------------------------------------------------------------------------
	// Helpers
	void enable_parameters(std::function<std::vector<Parameter>(void)> request_list_cb,
			       std::function<bool(Parameter*)> set_cb);

	bool connected() const { return _connection->connected(); };

private:
	//-----------------------------------------------------------------------------
	// Message handlers
	void handle_param_request_list(const mavlink_message_t& message);
	void handle_param_set(const mavlink_message_t& message);
	//-----------------------------------------------------------------------------
	// Message senders
	void send_param_value(const Parameter& param);

private:
	ConfigurationSettings _settings {};

	std::unique_ptr<Connection> _connection {};

	// Mavlink parameter callbacks
	std::function<std::vector<Parameter>(void)> _mav_param_request_list_cb;
	std::function<bool(Parameter* param)> _mav_param_set_cb;

	std::mutex _subscriptions_mutex {};
	std::unordered_map<uint16_t, MessageCallback> _message_subscriptions {}; // Mavlink message ID --> callback(mavlink_message_t)
};

} // end namespace mavlink