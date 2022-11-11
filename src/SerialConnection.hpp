#pragma once

#include <mutex>
#include <memory>
#include <atomic>
#include <thread>

#include <Connection.hpp>
#include <helpers.hpp>

#define LINUX

#if defined(WINDOWS)
#include "windows_include.h"
#endif

namespace mavlink
{

static constexpr uint64_t SERIAL_CONNECTION_TIMEOUT_MS = 2000;

class Mavlink;

class SerialConnection : public Connection
{

public:
	SerialConnection(Mavlink* parent);
	~SerialConnection();

	ConnectionResult start() override;
	void stop() override;

	bool send_message(const mavlink_message_t& message) override;

	// Non-copyable
	SerialConnection(const SerialConnection&) = delete;
	const SerialConnection& operator=(const SerialConnection&) = delete;

private:
	ConnectionResult setup_port();
	void start_recv_thread();
	void receive_thread_main();
	void receive();

#if defined(LINUX)
	static int define_from_baudrate(int baudrate);
#endif

	std::string _serial_node {};
	int         _baudrate {};
	bool        _flow_control {};

	std::mutex _mutex = {};
#if !defined(WINDOWS)
	int _fd = -1;
#else
	HANDLE _handle;
#endif

	std::unique_ptr<std::thread> _recv_thread{};
	std::atomic_bool _should_exit{false};

	Mavlink* _parent {};
};

} // namespace mavsdk
