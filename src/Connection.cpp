#include <Connection.hpp>

namespace mavlink
{

Connection::Connection(uint64_t connection_timeout_ms)
	: _connection_timeout_ms(connection_timeout_ms)
{}

bool Connection::connected()
{
	return _connected && !connection_timed_out();
}

bool Connection::connection_timed_out()
{
	return millis() > _last_received_heartbeat_ms + _connection_timeout_ms;
}

bool Connection::queue_message(const mavlink_message_t& message)
{
	return _message_outbox_queue.push_back(message);
}

bool Connection::should_handle_message(const mavlink_message_t& message)
{
	bool handle = false;

	if (_target_sysid == 0) {
		// Handle messages from all components of all systems
		handle == true;

	} else if (_target_compid == 0)  {
		// Handle messages from all components of a system
		handle = message.sysid == _target_sysid;

	} else {
		// Handle messages from only one component of a system
		handle = message.sysid == _target_sysid && message.compid == _target_compid;
	}

	return handle;
}

} // end namespace mavlink
