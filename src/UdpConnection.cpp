#include "UdpConnection.hpp"
#include "MessageParser.hpp"

#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>

namespace mavlink
{

UdpConnection::UdpConnection(std::string conn, std::function<void(const mavlink_message_t& message)> message_callback)
	: Connection(UDP_CONNECTION_TIMEOUT_MS)
	, _message_handler(message_callback)
{
	std::string udp = "udp:";
	conn.erase(conn.find(udp), udp.length());

	size_t index = conn.find(':');
	_our_ip = conn.substr(0, index);
	conn.erase(0, index + 1);
	_our_port = std::stoi(conn);
}

void UdpConnection::start()
{
	setup_port();

	_recv_thread = std::make_unique<std::thread>(&UdpConnection::receive_thread_main, this);
	_send_thread = std::make_unique<std::thread>(&UdpConnection::send_thread_main, this);
}

void UdpConnection::stop()
{
	_should_exit = true;

	_recv_thread->join();
	_send_thread->join();

	shutdown(_socket_fd, SHUT_RDWR);
	close(_socket_fd);
}

void UdpConnection::setup_port()
{
	LOG("Setting up port");
	_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_socket_fd < 0) {
		LOG("socket error");
		return;
	}

	struct sockaddr_in addr = {};

	addr.sin_family = AF_INET;

	inet_pton(AF_INET, _our_ip.c_str(), &(addr.sin_addr));

	addr.sin_port = htons(_our_port);

	if (bind(_socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
		LOG("bind error");
		return;
	}

	_initialized = true;
}

bool UdpConnection::send_message(const mavlink_message_t& message)
{
	struct sockaddr_in dest_addr {};
	dest_addr.sin_family = AF_INET;

	inet_pton(AF_INET, _remote_ip.c_str(), &dest_addr.sin_addr.s_addr);
	dest_addr.sin_port = htons(_remote_port);

	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	uint16_t buffer_len = mavlink_msg_to_send_buffer(buffer, &message);

	const auto send_len = sendto(_socket_fd, reinterpret_cast<char*>(buffer), buffer_len,
				     0, reinterpret_cast<const sockaddr*>(&dest_addr), sizeof(dest_addr));

	return send_len == buffer_len;
}

void UdpConnection::send_thread_main()
{
	while (!_should_exit) {
		if (_initialized && _connected) {

			mavlink_message_t message;
			if (_message_outbox_queue.pop_front(&message)) {
				if (!send_message(message)) {
					LOG(RED_TEXT "Send message failed!" NORMAL_TEXT);
				}
			}

			// TODO: extend ThreadSafeQueue to optionally block-wait for items using condition variable
			// rate limit 100Hz
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		} else {
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
}

void UdpConnection::receive_thread_main()
{
	while (!_should_exit) {
		if (!_initialized) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			setup_port();

		} else {
			receive();

			if (_connected && connection_timed_out()) {
				LOG(RED_TEXT "Connection timed out" NORMAL_TEXT);
				_connected = false;
			}
		}
	}
}

void UdpConnection::receive()
{
	struct sockaddr_in src_addr = {};
	socklen_t src_addr_len = sizeof(src_addr);

	const ssize_t recv_len = recvfrom(
					 _socket_fd,
					 _receive_buffer,
					 sizeof(_receive_buffer),
					 0,
					 reinterpret_cast<struct sockaddr*>(&src_addr),
					 &src_addr_len);

	if (recv_len == 0) {
		// This can happen when shutdown is called on the socket, therefore we check _should_exit again.
		return;
	}

	if (recv_len < 0) {
		// This happens on destruction when close(_socket_fd) is called, therefore be quiet.
		return;
	}

	mavlink_message_t message;
	auto parser = MessageParser(_receive_buffer, recv_len);

	while (parser.parse(&message)) {
		if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT && message.sysid == _target_sysid && message.compid == _target_compid) {
			if (connection_timed_out() && !_connected) {
				_remote_ip = inet_ntoa(src_addr.sin_addr);
				_remote_port = ntohs(src_addr.sin_port);
				_connected = true;
				LOG(GREEN_TEXT "Connected to autopilot on: %s:%d (with sysid: %d)" NORMAL_TEXT, _remote_ip.c_str(), _remote_port, message.sysid);
			}

			_last_heartbeat_ms = std::chrono::duration_cast<std::chrono::milliseconds>
					     (std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		}

		// Call the message handler callback
		_message_handler(message);
	}
}

} // end namespace mavlink
