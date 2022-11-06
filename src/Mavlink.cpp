#include <Mavlink.hpp>

#include <UdpConnection.hpp>
// #include <parameters.hpp>

namespace mavlink
{

Mavlink::Mavlink(uint8_t sysid, uint8_t compid, std::string connection_string)
	: _sysid(sysid)
	, _compid(compid)
	, _connection_string(connection_string)
{
	setup_subscriptions();
}

Mavlink::~Mavlink()
{
	stop();
}

void Mavlink::start()
{
	if (_connection_string.find("serial:") != std::string::npos) {
		// TODO: create serial

	} else if (_connection_string.find("udp:") != std::string::npos) {

		// We provide the connection class with a message handler callback function
		_connection = std::make_unique<UdpConnection>(_connection_string, [this](const mavlink_message_t& message) { handle_message(message); });
		_connection->start();

	} else {
		LOG("Invalid connection string: %s\nNo connection started", _connection_string.c_str());
	}
}

void Mavlink::stop()
{
	// Waits for connection threads to join
	_connection->stop();
}

void Mavlink::handle_message(const mavlink_message_t& message)
{
	if (_message_subscriptions.contains(message.msgid)) {
		_message_subscriptions[message.msgid](message);
	}
}

void Mavlink::subscribe_to_message(uint16_t message_id, const MessageCallback& callback)
{
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

	bool for_us = msg.target_system == _sysid && msg.target_component == _compid;
	bool for_system = msg.target_system == _sysid && msg.target_component == 0;
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

	bool from_autopilot = message.sysid == _sysid;

	if (from_autopilot) {
		_rangefinder_distance_cm.store(msg.current_distance);
	}
}

void Mavlink::handle_param_request_list(const mavlink_message_t& message)
{
	mavlink_param_request_list_t msg;
	mavlink_msg_param_request_list_decode(&message, &msg);

	bool for_us = msg.target_system == _sysid && msg.target_component == _compid;
	bool for_system = msg.target_system == _sysid && msg.target_component == 0;
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

	bool for_us = msg.target_system == _sysid && msg.target_component == _compid;

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
	mavlink_msg_param_value_encode(_sysid, _compid, &message, &pv);

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
	mavlink_msg_command_ack_encode(_sysid, _compid, &message, &ack);

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
	mavlink_msg_statustext_encode(_sysid, _compid, &message, &status);

	send_message(message);
}

} // end namespace mavlink
