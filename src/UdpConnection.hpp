#pragma once

#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <functional>

#include <time.h>

#include <Connection.hpp>
#include <helpers.hpp>

namespace mavlink
{

static constexpr uint64_t UDP_CONNECTION_TIMEOUT_MS = 2000;

class Mavlink;

class UdpConnection : public Connection
{
public:
	UdpConnection(Mavlink* parent);

	ConnectionResult start() override;
	void stop() override;
	bool send_message(const mavlink_message_t& message) override;

	// Non-copyable
	UdpConnection(const UdpConnection&) = delete;
	const UdpConnection& operator=(const UdpConnection&) = delete;

private:
	ConnectionResult setup_port();

	void receive_thread_main();
	void send_thread_main();

	void receive();

	// Our IP and port
	std::string _our_ip {};
	int _our_port {};

	// Autopilot IP and port
	std::string _remote_ip {};
	int _remote_port {};

	// Connection
	int _socket_fd {-1};
	std::unique_ptr<std::thread> _recv_thread {};
	std::unique_ptr<std::thread> _send_thread {};
	std::atomic_bool _should_exit {false};
	char _receive_buffer[2048] {}; // Enough for MTU 1500 bytes.

	// Mavlink internal data
	char* _datagram {};
	unsigned _datagram_len {};

	// std::function<void(const mavlink_message_t& message)> _message_handler;
	Mavlink* _parent {};
};

} // end namespace mavlink
