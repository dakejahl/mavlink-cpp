#include <Mavlink.hpp>

#include <UdpConnection.hpp>
// #include <parameters.hpp>

namespace mavlink
{

static constexpr uint64_t HEARTBEAT_INTERVAL_MS = 1000; // 1Hz

Mavlink::Mavlink(const ConfigurationSettings& settings)
	: _settings(settings)
{
	setup_subscriptions();
}

Mavlink::~Mavlink()
{
	stop();
}

void Mavlink::start()
{
	if (_settings.connection_url.find("serial:") != std::string::npos) {
		// TODO: create serial

	} else if (_settings.connection_url.find("udp:") != std::string::npos) {

		// We provide the connection class with a message handler callback function
		_connection = std::make_unique<UdpConnection>(_settings.connection_url, [this](const mavlink_message_t& message) { handle_message(message); });

		// Spawns thread -- all connection handling happens in that thread context
		_connection->start();

	} else {
		LOG("Invalid connection string: %s\nNo connection started", _settings.connection_url.c_str());
	}
}

void Mavlink::stop()
{
	// Waits for connection threads to join
	if (_connection.get()) _connection->stop();
}

void Mavlink::handle_message(const mavlink_message_t& message)
{
	std::scoped_lock<std::mutex> lock(_subscriptions_mutex);

	if (_message_subscriptions.contains(message.msgid)) {
		_message_subscriptions[message.msgid](message);
	}
}

void Mavlink::subscribe_to_message(uint16_t message_id, const MessageCallback& callback)
{
	std::scoped_lock<std::mutex> lock(_subscriptions_mutex);

	if (_message_subscriptions.find(message_id) == _message_subscriptions.end()) {
		_message_subscriptions.emplace(message_id, callback);

	} else {
		LOG(RED_TEXT "Mavlink::subscribe_to_message failed, callback already registered" NORMAL_TEXT);
	}
}

void Mavlink::send_message(const mavlink_message_t& message)
{
	if (_connection->connected()) {
		if (!_connection->queue_message(message)) {
			LOG(RED_TEXT "Queueing message failed! Message queue full" NORMAL_TEXT);
		}
	}
}

void Mavlink::send_heartbeat()
{
	static uint64_t last_heartbeat_ms = 0;

	uint64_t time_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
			    now().time_since_epoch()).count();

	if (time_now > last_heartbeat_ms + HEARTBEAT_INTERVAL_MS) {

		mavlink_heartbeat_t hb = {};
		hb.type = _settings.mav_type;
		hb.autopilot = _settings.mav_autopilot;
		hb.system_status = MAV_STATE_ACTIVE; // TODO: report failsafes like this? Or something?

		mavlink_message_t message;
		mavlink_msg_heartbeat_encode(_settings.sysid, _settings.compid, &message, &hb);

		send_message(message);

		last_heartbeat_ms = time_now;
	}
}

// TODO: EVERYTHING BELOW THIS GOES INTO USER FILE FOR LIBRARY

void Mavlink::setup_subscriptions()
{
	subscribe_to_message(MAVLINK_MSG_ID_COMMAND_LONG, 		[this](const mavlink_message_t& message) { handle_command_long(message); });
	subscribe_to_message(MAVLINK_MSG_ID_DISTANCE_SENSOR, 	[this](const mavlink_message_t& message) { handle_distance_sensor(message); });
	subscribe_to_message(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, [this](const mavlink_message_t& message) { handle_param_request_list(message); });
	subscribe_to_message(MAVLINK_MSG_ID_PARAM_SET, 			[this](const mavlink_message_t& message) { handle_param_set(message); });
}

void Mavlink::handle_command_long(const mavlink_message_t& message)
{
	mavlink_command_long_t msg;
	mavlink_msg_command_long_decode(&message, &msg);

	bool for_us = msg.target_system == _settings.sysid && msg.target_component == _settings.compid;
	bool for_system = msg.target_system == _settings.sysid && msg.target_component == 0;
	bool broadcast = msg.target_system == 0 && msg.target_component == 0;

	if (for_us || for_system || broadcast) {
		auto mav_cmd = MavCommand(message.sysid, message.compid, msg.command);

		if (!_command_queue.push_back(mav_cmd)) {
			LOG(RED_TEXT "Command %u temporarily rejected" NORMAL_TEXT, mav_cmd.command);
			send_command_ack(mav_cmd, MAV_RESULT_TEMPORARILY_REJECTED);
		}
	}
}

void Mavlink::handle_distance_sensor(const mavlink_message_t& message)
{
	mavlink_distance_sensor_t msg;
	mavlink_msg_distance_sensor_decode(&message, &msg);

	bool from_autopilot = message.sysid == _settings.sysid;

	if (from_autopilot) {
		_rangefinder_distance_cm.store(msg.current_distance);
	}
}

void Mavlink::handle_param_request_list(const mavlink_message_t& message)
{
	mavlink_param_request_list_t msg;
	mavlink_msg_param_request_list_decode(&message, &msg);

	bool for_us = msg.target_system == _settings.sysid && msg.target_component == _settings.compid;
	bool for_system = msg.target_system == _settings.sysid && msg.target_component == 0;
	bool broadcast = msg.target_system == 0 && msg.target_component == 0;
	bool handle_message = for_us || for_system || broadcast;

	if (!handle_message) {
		return;
	}

	LOG(GREEN_TEXT "Sending parameters to GCS" NORMAL_TEXT);

	// size_t count = 0;
	// auto params = params::parameters();

	// for (auto& [name, value] : params) {
	// 	send_param_value(name, value, count++, params.size());
	// }
}

void Mavlink::handle_param_set(const mavlink_message_t& message)
{
	mavlink_param_set_t msg;
	mavlink_msg_param_set_decode(&message, &msg);

	bool for_us = msg.target_system == _settings.sysid && msg.target_component == _settings.compid;

	if (!for_us) {
		return;
	}

	size_t length = strlen(msg.param_id);

	if (length > 16) {
		length = 16;
	}

	std::string name = std::string(msg.param_id, length);
	float value = msg.param_value;

	// bool success = params::set_parameter(name, value);

	// if (success) {
	// 	auto params = params::parameters();
	// 	send_param_value(name, value, std::distance(params.begin(), params.find(name)), params.size());
	// }
}

void Mavlink::send_param_value(std::string name, float value, uint16_t index, uint16_t total_count)
{
	mavlink_param_value_t pv = {};
	pv.param_count = total_count;
	pv.param_index = index;
	pv.param_value = value;
	pv.param_type = MAV_PARAM_TYPE_REAL32;

	mavlink_message_t message;
	mavlink_msg_param_value_encode(_settings.sysid, _settings.compid, &message, &pv);

	send_message(message);
}

void Mavlink::send_command_ack(const MavCommand& mav_cmd, int result)
{
	mavlink_command_ack_t ack = {};
	ack.command = mav_cmd.command;
	ack.result = result;
	ack.target_system = mav_cmd.sender_sysid;
	ack.target_component = mav_cmd.sender_compid;

	mavlink_message_t message;
	mavlink_msg_command_ack_encode(_settings.sysid, _settings.compid, &message, &ack);

	send_message(message);
}

void Mavlink::send_status_text(std::string&& text, int severity)
{
	mavlink_statustext_t status = {};

	status.severity = severity;
	std::string fulltext = "REEL: " + text;
	LOG("statustext: %s", fulltext.c_str());
	memcpy(status.text,  fulltext.c_str(), fulltext.size() > 49 ? 49 : fulltext.size());
	status.text[49] = '\0'; // Add null terminator

	mavlink_message_t message;
	mavlink_msg_statustext_encode(_settings.sysid, _settings.compid, &message, &status);

	send_message(message);
}

} // end namespace mavlink
