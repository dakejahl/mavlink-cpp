#pragma once

#include <mavlink.h>
#include <ThreadSafeQueue.hpp>
#include <helpers.hpp>

namespace mavlink
{

class Connection
{
public:
	Connection(uint64_t connection_timeout_ms, uint8_t target_sysid = 1, uint8_t target_compid = 1);

	bool connected();
	bool connection_timed_out();
	bool queue_message(const mavlink_message_t& message);

	virtual void start() = 0;
	virtual void stop() = 0;
	virtual bool send_message(const mavlink_message_t& message) = 0;

protected:
	ThreadSafeQueue<mavlink_message_t> _message_outbox_queue {100};

	uint8_t _target_sysid {};
	uint8_t _target_compid {};

	bool _initialized {};
	bool _connected {};

	uint64_t _last_heartbeat_ms {};
	uint64_t _connection_timeout_ms {};

	bool _emit_heartbeat {};
};

} // end namespace mavlink