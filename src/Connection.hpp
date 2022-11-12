#pragma once

#include <mavlink.h>

#include <ConnectionResult.hpp>
#include <ThreadSafeQueue.hpp>
#include <helpers.hpp>

namespace mavlink
{

class Connection
{
public:
	Connection(uint64_t connection_timeout_ms);

	bool connected();
	bool connection_timed_out();
	bool queue_message(const mavlink_message_t& message);

	virtual ConnectionResult start() = 0;
	virtual void stop() = 0;
	virtual bool send_message(const mavlink_message_t& message) = 0;

	static constexpr uint64_t HEARTBEAT_INTERVAL_MS = 1000; // 1Hz

protected:
	ThreadSafeQueue<mavlink_message_t> _message_outbox_queue {100};

	uint8_t _target_sysid {1};
	uint8_t _target_compid {1};

	bool _initialized {};
	bool _connected {};

	uint64_t _last_received_heartbeat_ms {};
	uint64_t _last_sent_heartbeat_ms {};

	uint64_t _connection_timeout_ms {};

	bool _emit_heartbeat {};
};

} // end namespace mavlink