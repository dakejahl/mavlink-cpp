#pragma once

#include <atomic>
#include <functional>
#include <queue>
#include <mutex>
#include <memory>
#include <unordered_map>

#include <ConnectionResult.hpp>
#include <ThreadSafeQueue.hpp>

#include <mavlink.h>

namespace mavlink
{

using MessageCallback = std::function<void(const mavlink_message_t&)>;

struct ConfigurationSettings {
	std::string connection_url {};
	uint8_t sysid {};
	uint8_t compid {};
	uint8_t target_sysid {};
	uint8_t target_compid {};
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

// This class holds data for the DO_WINCH and DO_GRIPPER commands
struct MavlinkCommand {
	MavlinkCommand(uint16_t source_system, uint16_t source_component, const mavlink_command_long_t& msg)
		: source_system(source_system)
		, source_component(source_component)
		, command(msg.command)
		, param1(msg.param1)
		, param2(msg.param2)
		, param3(msg.param3)
		, param4(msg.param4)
		, param5(msg.param5)
		, param6(msg.param6)
		, param7(msg.param7)
	{}

	uint8_t source_system {};
	uint8_t source_component {};
	uint16_t command {};
	float param1 {};
	float param2 {};
	float param3 {};
	float param4 {};
	float param5 {};
	float param6 {};
	float param7 {};
};

class Connection;

class Mavlink
{
public:
	Mavlink(const ConfigurationSettings& settings);
	~Mavlink();

	ConnectionResult start();
	void stop();

	uint8_t sysid() const { return _settings.sysid; };
	uint8_t compid() const { return _settings.compid; };

	void subscribe_to_message(uint16_t message_id, const MessageCallback& callback);
	void handle_message(const mavlink_message_t& message);

	bool connected();

	//-----------------------------------------------------------------------------
	// Message senders
	void send_message(const mavlink_message_t& message);
	void send_heartbeat();
	void send_status_text(std::string&& message, MAV_SEVERITY severity = MAV_SEVERITY_CRITICAL);
	void send_command_ack(const mavlink::MavlinkCommand& mav_cmd, MAV_RESULT mav_result);

	//-----------------------------------------------------------------------------
	// Helpers
	void enable_parameters(std::function<std::vector<Parameter>(void)> request_list_cb,
			       std::function<bool(Parameter*)> set_cb);

private:
	ConfigurationSettings settings() { return _settings; };

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

	friend class UdpConnection;
	friend class SerialConnection;
};

} // end namespace mavlink