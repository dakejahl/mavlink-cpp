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
	return millis() > _last_heartbeat_ms + _connection_timeout_ms;
}

bool Connection::queue_message(const mavlink_message_t& message)
{
	return _message_outbox_queue.push_back(message);
}

} // end namespace mavlink
