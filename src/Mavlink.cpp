#include <Mavlink.hpp>

#include <UdpConnection.hpp>

namespace mavlink
{

Mavlink::Mavlink(const ConfigurationSettings& settings)
	: _settings(settings)
{}

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
		_connection = std::make_unique<UdpConnection>(this);

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
	mavlink_heartbeat_t hb = {};
	hb.type = _settings.mav_type;
	hb.autopilot = _settings.mav_autopilot;
	hb.system_status = MAV_STATE_ACTIVE; // TODO: report failsafes like this? Or something?

	mavlink_message_t message;
	mavlink_msg_heartbeat_encode(_settings.sysid, _settings.compid, &message, &hb);

	send_message(message);
}

void Mavlink::enable_parameters(std::function<std::vector<Parameter>(void)> request_list_cb,
				std::function<bool(Parameter*)> set_cb)
{
	_mav_param_request_list_cb = std::move(request_list_cb);
	_mav_param_set_cb = std::move(set_cb);

	subscribe_to_message(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, [this](const mavlink_message_t& message) { handle_param_request_list(message); });
	subscribe_to_message(MAVLINK_MSG_ID_PARAM_SET, 			[this](const mavlink_message_t& message) { handle_param_set(message); });
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

	size_t count = 0;

	// Param request_list callback
	std::vector<Parameter> mavlink_parameters = _mav_param_request_list_cb();

	for (auto& p : mavlink_parameters) {
		send_param_value(p);
	}
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

	Parameter param = {
		.name = std::string(msg.param_id, length),
		.float_value = msg.param_value,
		.type = msg.param_type,
	};
	// Param set callback -- it will modify 'param' to set the index on success
	bool success = _mav_param_set_cb(&param);

	if (success) {
		send_param_value(param);
	}
}

void Mavlink::send_param_value(const Parameter& param)
{
	mavlink_param_value_t pv = {};
	sprintf(pv.param_id, "%s", param.name.c_str());
	pv.param_value = param.float_value;
	pv.param_index = param.index;
	pv.param_count = param.total_count;
	pv.param_type = param.type;

	mavlink_message_t message;
	mavlink_msg_param_value_encode(_settings.sysid, _settings.compid, &message, &pv);

	send_message(message);
}

void Mavlink::send_command_ack(const MavCommand& mav_cmd, MAV_RESULT result)
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
	LOG("statustext: %s", text.c_str());
	memcpy(status.text,  text.c_str(), text.size() > 49 ? 49 : text.size());
	status.text[49] = '\0'; // Add null terminator

	mavlink_message_t message;
	mavlink_msg_statustext_encode(_settings.sysid, _settings.compid, &message, &status);

	send_message(message);
}

} // end namespace mavlink
