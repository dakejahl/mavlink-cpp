#include "UdpConnection.hpp"
#include "MessageParser.hpp"
#include "Mavlink.hpp"

#include <arpa/inet.h>
#include <unistd.h>

#include <algorithm>
#include <iostream>

namespace mavlink
{

// TODO: overload constructor to pass in connection target -- target.sysid and target.compid (instead of 1/1 for autopilot)
UdpConnection::UdpConnection(Mavlink* parent)
	: Connection(UDP_CONNECTION_TIMEOUT_MS)
	, _parent(parent)
{
	ConfigurationSettings settings = _parent->settings();

	std::string udp = "udp:";
	std::string conn = settings.connection_url;

	_emit_heartbeat = settings.emit_heartbeat;

	conn.erase(conn.find(udp), udp.length());

	size_t index = conn.find(':');
	_our_ip = conn.substr(0, index);
	conn.erase(0, index + 1);
	_our_port = std::stoi(conn);

	// Set the target system/component
	if (settings.target_sysid && settings.target_compid) {
		_target_sysid = settings.target_sysid;
		_target_compid = settings.target_compid;
	}
}

ConnectionResult UdpConnection::start()
{
	auto result = setup_port();

	if (result != ConnectionResult::Success) {
		return result;
	}

	_recv_thread = std::make_unique<std::thread>(&UdpConnection::receive_thread_main, this);
	_send_thread = std::make_unique<std::thread>(&UdpConnection::send_thread_main, this);

	return ConnectionResult::Success;
}

void UdpConnection::stop()
{
	_should_exit = true;

	// Close socket and wait for receiving thread
	shutdown(_socket_fd, SHUT_RDWR);
	close(_socket_fd);
	_recv_thread->join();

	// Clear outbox and wake up sending thread
	_message_outbox_queue.clear();
	_send_thread->join();
}

ConnectionResult UdpConnection::setup_port()
{
	LOG("Initializing UDP connection");
	_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);

	if (_socket_fd < 0) {
		LOG("socket error");
		return ConnectionResult::SocketError;
	}

	struct sockaddr_in addr = {};

	addr.sin_family = AF_INET;

	inet_pton(AF_INET, _our_ip.c_str(), &(addr.sin_addr));

	addr.sin_port = htons(_our_port);

	if (bind(_socket_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
		LOG("bind error");
		return ConnectionResult::BindError;
	}

	_initialized = true;

	return ConnectionResult::Success;
}

bool UdpConnection::send_message(const mavlink_message_t& message)
{
	// LOG("[UdpConnection] sending message %u", message.msgid);
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
	LOG("[UdpConnection] Starting sending thread");

	while (!_should_exit) {
		if (_initialized && _connected) {

			std::optional<mavlink_message_t> message = _message_outbox_queue.pop_front(/* blocking */ true);

			if (message) {
				if (!send_message(message.value())) {
					LOG(RED_TEXT "Send message failed!" NORMAL_TEXT);
				}
			}

		} else {
			LOG("[UdpConnection] waiting for connection");
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}

	LOG("[UdpConnection] Exiting send thread");
}

void UdpConnection::receive_thread_main()
{
	LOG("[UdpConnection] Starting receive thread");

	while (!_should_exit) {
		if (!_initialized) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
			setup_port();

		} else {
			receive(); // Note: this blocks when not receiving any data

			if (_connected && connection_timed_out()) {
				LOG(RED_TEXT "Connection timed out" NORMAL_TEXT);
				_connected = false;
			}

			// Check if it's time to send a heartbeat. Only send heartbeats if we're still connected to an autopilot
			if (_connected && _emit_heartbeat) {
				auto now = millis();

				if (now > _last_sent_heartbeat_ms + Connection::HEARTBEAT_INTERVAL_MS) {
					_parent->send_heartbeat();
					_last_sent_heartbeat_ms = now;
				}
			}
		}
	}

	LOG("[UdpConnection] Exiting receive thread");
}

void UdpConnection::receive()
{
	struct sockaddr_in src_addr = {};
	socklen_t src_addr_len = sizeof(src_addr);

	// NOTE: This function blocks -- thus during destruction we call shutdown/close on the socket before joining the thread
	// TODO: this isn't actually returning if there's no data coming in
	// We need to find a way to signal to the thread to unblock from recvfrom
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
				LOG(GREEN_TEXT "Connected to %s:%d -- sysid %u compid %u" NORMAL_TEXT, _remote_ip.c_str(), _remote_port, message.sysid, message.compid);
			}

			_last_received_heartbeat_ms = millis();
		}

		// Call the message handler callback only if we're still connect to the target system
		if (_connected) {
			_parent->handle_message(message);
		}
	}
}

} // end namespace mavlink
